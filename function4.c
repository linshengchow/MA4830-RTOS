/*
 * function4.c  –  MA4830 CA2 : Function 4
 *
 * Integrates ADC input with real-time waveform generation:
 *
 *   1. ADC thread   – reads the potentiometer on channel 0 every 50 ms and
 *                     updates a shared amplitude variable (0.0 – 0.5).
 *
 *   2. Waveform thread – outputs a continuous waveform on the DAC, reading
 *                        the shared amplitude each cycle so the knob controls
 *                        the amplitude live.
 *
 *   3. Main (DIO monitor) – configures Port A as digital input, snapshots the
 *                           initial bit pattern, then polls every 50 ms.
 *                           When the pattern changes the program exits cleanly
 *                           (alternative to Ctrl-C).
 *
 * Reuses:
 *   pci_setup(), adc_init(), adc_read(), dac_write()  –  from fileio.c
 *   run_waveform() macros (DA_CTLREG, DA_DATA, DA_FIFOCLR, NUM_POINTS, …)  –  from waveform.h
 *   DIO_PORTA, DIO_CTLREG macros                                           –  from waveform.h
 *   data_mutex, shared_pot_value, stop_signal                               –  from fileio.c / fileio.h
 *
 * Compilation:
 *   qcc -o function4 function4.c fileio.c waveform.c -lm -lpthread
 *
 * Usage:
 *   ./function4 <type> <freq> <mean>
 *   e.g.  ./function4 sine 5.0 0.5
 *         ./function4 square 2.0 0.5
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <time.h>
#include <pthread.h>
#include <sys/neutrino.h>
#include <hw/inout.h>

#include "waveform.h"   /* DA_*, DIO_*, NUM_POINTS, CLK_PERIOD_NS, WAVE_*   */
#include "fileio.h"     /* pci_setup, adc_init, adc_read, shared state       */

/* ── Shared Amplitude ────────────────────────────────────────────────────── */
/*
 * Written exclusively by adc_amp_thread; read by waveform_output_thread.
 * volatile ensures the compiler never caches the value across loop iterations.
 */
static volatile float g_amp = 0.25f;   /* initial 25 % amplitude            */

/* ══════════════════════════════════════════════════════════════════════════ */
/* Thread 1 : ADC → Amplitude                                                */
/* ══════════════════════════════════════════════════════════════════════════ */

/*
 * adc_amp_thread
 *
 * Calls adc_read() (fileio.c) every 50 ms and maps the raw 16-bit ADC value
 * (0x0000 – 0xFFFF) to an amplitude fraction of 0.0 – 0.5.
 * shared_pot_value (fileio.h extern) is also updated so other parts of the
 * program can inspect the raw ADC reading.
 */
static void *adc_amp_thread(void *arg)
{
    uintptr_t     *iobase = (uintptr_t *)arg;
    unsigned short raw;

    printf("[ADC thread] started\n");

    while (!stop_signal) {
        raw = adc_read(iobase);   /* trigger + wait EOC */

        pthread_mutex_lock(&data_mutex);
        shared_pot_value = raw;
        g_amp = (raw / 65535.0f) * 0.5f;        /* scale to 0.0 – 0.5       */
        pthread_mutex_unlock(&data_mutex);

        printf("ADC Chan: 00  Raw: %04x  Amp: %.4f\n",
               (unsigned int)raw, (float)g_amp);

        usleep(50000);   /* 50 ms update rate – smooth pot response           */
    }

    printf("[ADC thread] stopped\n");
    return NULL;
}

/* ══════════════════════════════════════════════════════════════════════════ */
/* Thread 2 : Waveform Output                                                */
/* ══════════════════════════════════════════════════════════════════════════ */

typedef struct {
    uintptr_t *iobase;
    char      *type;
    float      freq;
    float      mean;
} wave_args_t;

/*
 * waveform_output_thread
 *
 * Outputs a continuous waveform on the DAC.  Uses the same macros and
 * constants as waveform.c (DA_CTLREG, DA_DATA, DA_FIFOCLR, NUM_POINTS,
 * DA_CTLREG_VAL, CLK_PERIOD_NS) but rebuilds the sample table each cycle
 * so that amplitude changes from g_amp take effect without any lag.
 * Exits when stop_signal is set by the main DIO monitor.
 */
static void *waveform_output_thread(void *arg)
{
    /* All declarations must appear before any statements (C89 rule) */
    wave_args_t         *w = (wave_args_t *)arg;
    uintptr_t           *iobase;
    char                *type;
    float                freq;
    float                mean;
    uint16_t             data[NUM_POINTS];
    int                  i;
    float                val;
    float                amp;
    long                 delay_ns;
    struct timespec      interval;
    struct _clockperiod  clk;

    iobase = w->iobase;
    type   = w->type;
    freq   = w->freq;
    mean   = w->mean;

    /* Pre-compute the per-point delay from frequency */
    delay_ns         = (long)((1.0 / freq / NUM_POINTS) * 1.0e9);
    interval.tv_sec  = delay_ns / 1000000000L;
    interval.tv_nsec = delay_ns % 1000000000L;

    /* Set system clock resolution for accurate nanosleep timing */
    clk.nsec  = CLK_PERIOD_NS;
    clk.fract = 0;
    ClockPeriod(CLOCK_REALTIME, &clk, NULL, 0);

    printf("[Waveform thread] started  type=%s  freq=%.2f Hz  mean=%.2f\n",
           type, freq, mean);

    while (!stop_signal) {
        /* Snapshot amplitude for this cycle – read once to stay consistent */
        amp = g_amp;

        /*
         * Unipolar DAC: ensure mean >= amp so the sine never dips below 0.
         * Without this, a low mean clips the negative half into a half-wave.
         */
        if (mean < amp) mean = amp;

        /* ── Build sample table ────────────────────────────────────────── */
        for (i = 0; i < NUM_POINTS; i++) {
            if (strcmp(type, WAVE_SINE) == 0) {
                val = sinf(i * (2.0f * M_PI / NUM_POINTS)) * amp + mean;

            } else if (strcmp(type, WAVE_SQUARE) == 0) {
                val = (i < NUM_POINTS / 2) ? (mean + amp) : (mean - amp);

            } else if (strcmp(type, WAVE_TRIANGLE) == 0) {
                if (i < NUM_POINTS / 2)
                    val = mean - amp + (2.0f * amp * (i / (NUM_POINTS / 2.0f)));
                else
                    val = mean + amp - (2.0f * amp *
                          ((i - NUM_POINTS / 2.0f) / (NUM_POINTS / 2.0f)));

            } else {   /* sawtooth */
                val = (mean - amp) + (2.0f * amp * ((float)i / NUM_POINTS));
            }

            if (val > 1.0f) val = 1.0f;
            if (val < 0.0f) val = 0.0f;
            data[i] = (uint16_t)(val * 0xFFFF);
        }

        /* ── Output one full cycle to the DAC ──────────────────────────── */
        for (i = 0; i < NUM_POINTS && !stop_signal; i++) {
            out16(DA_CTLREG(iobase), DA_CTLREG_VAL);
            out16(DA_FIFOCLR(iobase), 0);
            out16(DA_DATA(iobase),    data[i]);
            nanosleep(&interval, NULL);
        }
    }

    printf("[Waveform thread] stopped\n");
    return NULL;
}

/* ══════════════════════════════════════════════════════════════════════════ */
/* Main : DIO Monitor + Thread Supervisor                                    */
/* ══════════════════════════════════════════════════════════════════════════ */

int main(int argc, char *argv[])
{
    uintptr_t   iobase[DA_NUM_BARS];
    void       *pci_hdl;
    pthread_t   adc_tid, wave_tid;
    wave_args_t wave_args;
    uint8_t     dio_initial, dio_current;

    if (argc < 4) {
        printf("Usage: %s <type> <freq> <mean>\n\n", argv[0]);
        printf("  type  : sine | square | triangle | sawtooth\n");
        printf("  freq  : output frequency in Hz\n");
        printf("  mean  : DC offset (0.0 – 1.0)\n\n");
        printf("Amplitude is controlled live by the potentiometer (ADC ch 0).\n");
        printf("Change DIO Port A bit pattern to stop the program.\n");
        return EXIT_FAILURE;
    }

    /* Acquire hardware I/O privileges */
    if (ThreadCtl(_NTO_TCTL_IO, 0) == -1) {
        perror("ThreadCtl");
        return EXIT_FAILURE;
    }

    /* ── PCI setup and ADC initialisation (functions from fileio.c) ──── */
    pci_hdl = pci_setup(iobase);
    adc_init(iobase);

    /* ── DIO : configure Port A as input, read initial pattern ───────── */
    out8(DIO_CTLREG(iobase), 0x90);             /* Port A=in, B/C=out        */
    dio_initial = in8(DIO_PORTA(iobase));
    printf("DIO Port A initial pattern : 0x%02x\n", (unsigned int)dio_initial);
    printf("Program will stop when this pattern changes.\n\n");

    /* ── Spawn ADC amplitude thread ───────────────────────────────────── */
    stop_signal = 0;
    if (pthread_create(&adc_tid, NULL, adc_amp_thread, iobase) != 0) {
        perror("pthread_create (adc)");
        pci_detach_device(pci_hdl);
        return EXIT_FAILURE;
    }

    /* ── Spawn waveform output thread ─────────────────────────────────── */
    wave_args.iobase = iobase;
    wave_args.type   = argv[1];
    wave_args.freq   = atof(argv[2]);
    wave_args.mean   = atof(argv[3]);

    if (pthread_create(&wave_tid, NULL, waveform_output_thread, &wave_args) != 0) {
        perror("pthread_create (waveform)");
        stop_signal = 1;
        pthread_join(adc_tid, NULL);
        pci_detach_device(pci_hdl);
        return EXIT_FAILURE;
    }

    printf("Running. Turn potentiometer to change amplitude.\n");
    printf("Change DIO Port A to exit.\n\n");

    /* ── DIO poll loop : exit trigger ─────────────────────────────────── */
    while (1) {
        dio_current = in8(DIO_PORTA(iobase));
        if (dio_current != dio_initial) {
            printf("\nDIO pattern changed : 0x%02x -> 0x%02x  Stopping...\n",
                   (unsigned int)dio_initial, (unsigned int)dio_current);
            break;
        }
        usleep(50000);   /* poll every 50 ms – negligible CPU cost            */
    }

    /* ── Clean shutdown ───────────────────────────────────────────────── */
    stop_signal = 1;
    pthread_join(wave_tid, NULL);
    pthread_join(adc_tid,  NULL);

    pci_detach_device(pci_hdl);
    printf("Done.\n");
    return EXIT_SUCCESS;
}
