/*
 *  main.c  —  MA4830 Major CA : Integration entry point
 *
 *  Responsibilities (and nothing more):
 *    1. Parse and validate the four command-line arguments.
 *    2. Acquire I/O privileges and set up the PCI-DAS1602.
 *    3. Install a SIGINT handler so Ctrl+C shuts down cleanly.
 *    4. Seed the shared state with the initial configuration.
 *    5. Spawn the five worker threads (see threads.c).
 *    6. Wait for stop_signal and join every thread.
 *    7. Detach the PCI device and exit.
 *
 *  Usage:
 *      ./main <type> <freq> <amp> <mean>
 *
 *      type : sine | square | triangle | sawtooth
 *      freq : output frequency in Hz   (> 0)
 *      amp  : initial amplitude         (0.0 – 0.5; pot overrides live)
 *      mean : DC offset                 (0.0 – 1.0)
 *
 *  Runtime commands are documented in threads.c (user_cmd_thread).
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/neutrino.h>
#include <hw/inout.h>

#include "waveform.h"
#include "fileio.h"
#include "threads.h"

/* ── SIGINT Handler ──────────────────────────────────────────────────────── */

/*
 * On Ctrl+C we simply request a shutdown.  All five threads check
 * stop_signal in their outer loops and exit within one polling interval,
 * after which the supervisor joins them and releases hardware resources.
 */
static void handle_sigint(int sig)
{
    (void)sig;
    stop_signal = 1;
}

static void install_sigint_handler(void)
{
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = handle_sigint;
    sigemptyset(&sa.sa_mask);
    /* No SA_RESTART — we want blocked syscalls to return EINTR so the
     * thread can notice stop_signal without waiting for its timeout.  */
    sa.sa_flags = 0;
    sigaction(SIGINT,  &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
}

/* ── Argument Validation ─────────────────────────────────────────────────── */

static int validate_args(int argc, char *argv[],
                         float *freq_out, float *amp_out, float *mean_out)
{
    float f, a, m;

    if (argc != 5) {
        printf("Usage: %s <type> <freq> <amp> <mean>\n\n", argv[0]);
        printf("  type : sine | square | triangle | sawtooth\n");
        printf("  freq : output frequency in Hz          (e.g. 5.0)\n");
        printf("  amp  : initial amplitude  0.0 – 0.5    (e.g. 0.25)\n");
        printf("  mean : DC offset          0.0 – 1.0    (e.g. 0.5)\n\n");
        printf("Runtime:  change | freq <n> | mean <n> | "
               "record | playback | stop | q\n");
        return 0;
    }

    if (!valid_wave_type(argv[1])) {
        fprintf(stderr, "Error: type '%s' is invalid. "
                "Use: sine | square | triangle | sawtooth\n", argv[1]);
        return 0;
    }

    f = atof(argv[2]);
    if (f <= 0.0f) {
        fprintf(stderr, "Error: freq must be > 0 (got '%s')\n", argv[2]);
        return 0;
    }

    a = atof(argv[3]);
    if (a < 0.0f || a > 0.5f) {
        fprintf(stderr, "Error: amp must be 0.0 – 0.5 (got '%s')\n", argv[3]);
        return 0;
    }

    m = atof(argv[4]);
    if (m < 0.0f || m > 1.0f) {
        fprintf(stderr, "Error: mean must be 0.0 – 1.0 (got '%s')\n", argv[4]);
        return 0;
    }

    *freq_out = f;
    *amp_out  = a;
    *mean_out = m;
    return 1;
}

/* ── Main ────────────────────────────────────────────────────────────────── */

int main(int argc, char *argv[])
{
    uintptr_t   iobase[DA_NUM_BARS];
    void       *pci_hdl;
    pthread_t   adc_tid, wave_tid, file_tid, user_tid, dio_tid;
    dio_args_t  dio_args;
    float       init_freq, init_amp, init_mean;

    /* 1. Validate arguments before touching hardware */
    if (!validate_args(argc, argv, &init_freq, &init_amp, &init_mean))
        return EXIT_FAILURE;

    /* 2. Acquire hardware I/O privileges */
    if (ThreadCtl(_NTO_TCTL_IO, 0) == -1) {
        perror("ThreadCtl");
        return EXIT_FAILURE;
    }

    /* 3. Install SIGINT handler */
    install_sigint_handler();

    /* 4. PCI setup and ADC initialisation */
    pci_hdl = pci_setup(iobase);
    adc_init(iobase);

    /* 5. Seed shared state under mutex */
    cfg_set_type(argv[1]);
    cfg_set_freq(init_freq);
    cfg_set_amp (init_amp);
    cfg_set_mean(init_mean);
    set_mode(MODE_NORMAL);
    stop_signal = 0;

    /* 6. Configure DIO Port A as input, snapshot initial pattern */
    out8(DIO_CTLREG(iobase), 0x90);
    dio_args.iobase          = iobase;
    dio_args.initial_pattern = in8(DIO_PORTA(iobase));

    printf("\nMA4830 Major CA — type=%s  freq=%.2f Hz  amp=%.2f  mean=%.2f\n",
           argv[1], init_freq, init_amp, init_mean);
    printf("DIO Port A initial pattern: 0x%02x\n",
           (unsigned int)dio_args.initial_pattern);
    printf("Exit: 'q', Ctrl+C, or change DIO Port A pattern\n\n");

    /* 7. Spawn all five threads */
    if (pthread_create(&adc_tid,  NULL, adc_amp_thread,         iobase)     != 0 ||
        pthread_create(&wave_tid, NULL, waveform_output_thread, iobase)     != 0 ||
        pthread_create(&file_tid, NULL, file_write_thread,      iobase)     != 0 ||
        pthread_create(&user_tid, NULL, user_cmd_thread,        iobase)     != 0 ||
        pthread_create(&dio_tid,  NULL, dio_monitor_thread,     &dio_args)  != 0)
    {
        perror("pthread_create");
        stop_signal = 1;
        pthread_join(adc_tid,  NULL);
        pthread_join(wave_tid, NULL);
        pthread_join(file_tid, NULL);
        pthread_join(user_tid, NULL);
        pthread_join(dio_tid,  NULL);
        pci_detach_device(pci_hdl);
        return EXIT_FAILURE;
    }

    /* 8. Wait for stop_signal (Ctrl+C, 'q', or DIO change) */
    while (!stop_signal) {
        usleep(100000);
    }

    /* 9. Clean shutdown */
    pthread_join(wave_tid, NULL);
    pthread_join(adc_tid,  NULL);
    pthread_join(file_tid, NULL);
    pthread_join(user_tid, NULL);
    pthread_join(dio_tid,  NULL);

    disable_raw_mode();   /* restore terminal — must be after user_tid joins */
    pci_detach_device(pci_hdl);
    printf("Done.\n");
    return EXIT_SUCCESS;
}