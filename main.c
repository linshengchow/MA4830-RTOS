/*
 * main.c  –  MA4830 CA2 : Full Integration
 *
 * Five-thread real-time program integrating ADC, DAC, DIO, and file I/O:
 *
 *   1. adc_amp_thread         – reads potentiometer on ch 0 every 50 ms and
 *                               updates shared amplitude (0.0 – 0.5).
 *
 *   2. waveform_output_thread – outputs a continuous waveform on the DAC,
 *                               reading g_amp and g_wave_type each cycle.
 *                               Pauses DAC output during RECORDING / PLAYBACK
 *                               so the file thread has exclusive DAC access.
 *
 *   3. file_write_thread      – idle in MODE_NORMAL.
 *                               MODE_RECORDING: reads shared_pot_value, writes
 *                               to pot_data.txt AND outputs value to DAC so
 *                               the oscilloscope shows the live pot signal.
 *                               MODE_PLAYBACK:  reads pot_data.txt and replays
 *                               each sample to the DAC, then returns to NORMAL.
 *
 *   4. user_cmd_thread        – prints board status every 500 ms and accepts
 *                               runtime commands from the terminal.
 *
 *   5. dio_monitor_thread     – polls DIO Port A every 50 ms; sets stop_signal
 *                               when the bit pattern changes (hardware exit).
 *
 * Reuses:
 *   pci_setup(), adc_init(), adc_read(), dac_write()  –  fileio.c
 *   DA_CTLREG, DA_DATA, DA_FIFOCLR, NUM_POINTS,
 *   CLK_PERIOD_NS, WAVE_* strings                     –  waveform.h
 *   DIO_PORTA, DIO_CTLREG macros                      –  waveform.h
 *   data_mutex, shared_pot_value, stop_signal,
 *   DEFAULT_FILENAME, SAMPLE_DELAY_US                 –  fileio.c / fileio.h
 *
 * Compilation:
 *   qcc -o main main.c fileio.c waveform.c -lm -lpthread
 *
 * Usage:
 *   ./main <type> <freq> <amp> <mean>
 *   e.g.  ./main sine     5.0  0.25  0.5
 *         ./main triangle 2.0  0.4   0.5
 *
 *   type : sine | square | triangle | sawtooth
 *   freq : output frequency in Hz  (> 0)
 *   amp  : initial amplitude 0.0 – 0.5  (pot overrides this live)
 *   mean : DC offset          0.0 – 1.0
 *
 * Runtime commands (type command + Enter):
 *   change   – interactive menu to hot-swap waveform (oscilloscope keeps running)
 *   record   – stop waveform; log ADC pot to file; show pot signal on oscilloscope
 *   playback – replay recorded file to DAC on oscilloscope; then resume waveform
 *   stop     – stop recording or playback; resume waveform
 *   q        – quit program
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <time.h>
#include <pthread.h>
#include <sys/select.h>
#include <sys/neutrino.h>
#include <hw/inout.h>

#include "waveform.h"   /* DA_*, DIO_*, NUM_POINTS, CLK_PERIOD_NS, WAVE_*   */
#include "fileio.h"     /* pci_setup, adc_init, adc_read, dac_write, etc.   */

/* ── Operating modes ─────────────────────────────────────────────────────── */
#define MODE_NORMAL    0   /* waveform output active                          */
#define MODE_RECORDING 1   /* ADC → file + ADC → DAC; waveform paused         */
#define MODE_PLAYBACK  2   /* file → DAC; waveform paused                     */

/* ── Runtime-mutable shared state ────────────────────────────────────────── */
static volatile float g_amp           = 0.25f;
static char           g_wave_type[16] = "";    /* protected by data_mutex   */
static volatile int   op_mode         = MODE_NORMAL;

/* ── Helpers ──────────────────────────────────────────────────────────────── */

static int valid_wave_type(const char *s)
{
    return strcmp(s, WAVE_SINE)     == 0 ||
           strcmp(s, WAVE_SQUARE)   == 0 ||
           strcmp(s, WAVE_TRIANGLE) == 0 ||
           strcmp(s, WAVE_SAWTOOTH) == 0;
}

static const char *mode_str(void)
{
    switch (op_mode) {
        case MODE_RECORDING: return "REC";
        case MODE_PLAYBACK:  return "PB ";
        default:             return "---";
    }
}

/* ══════════════════════════════════════════════════════════════════════════ */
/* Thread 1 : ADC → Amplitude                                                */
/* ══════════════════════════════════════════════════════════════════════════ */

static void *adc_amp_thread(void *arg)
{
    uintptr_t     *iobase = (uintptr_t *)arg;
    unsigned short raw;

    printf("[ADC] started\n");

    while (!stop_signal) {
        raw = adc_read(iobase);

        pthread_mutex_lock(&data_mutex);
        shared_pot_value = raw;
        g_amp = (raw / 65535.0f) * 0.5f;   /* map 0–0xFFFF → 0.0–0.5 */
        pthread_mutex_unlock(&data_mutex);

        usleep(50000);   /* 50 ms */
    }

    printf("[ADC] stopped\n");
    return NULL;
}

/* ══════════════════════════════════════════════════════════════════════════ */
/* Thread 2 : Waveform Output                                                */
/* ══════════════════════════════════════════════════════════════════════════ */

typedef struct {
    uintptr_t *iobase;
    float      freq;
    float      mean;
} wave_args_t;

static void *waveform_output_thread(void *arg)
{
    /* C89: all declarations before statements */
    wave_args_t         *w = (wave_args_t *)arg;
    uintptr_t           *iobase;
    float                freq;
    float                mean;
    char                 local_type[16];
    uint16_t             data[NUM_POINTS];
    int                  i;
    float                val;
    float                amp;
    long                 delay_ns;
    struct timespec      interval;
    struct _clockperiod  clk;

    iobase = w->iobase;
    freq   = w->freq;
    mean   = w->mean;

    delay_ns         = (long)((1.0 / freq / NUM_POINTS) * 1.0e9);
    interval.tv_sec  = delay_ns / 1000000000L;
    interval.tv_nsec = delay_ns % 1000000000L;

    clk.nsec  = CLK_PERIOD_NS;
    clk.fract = 0;
    ClockPeriod(CLOCK_REALTIME, &clk, NULL, 0);

    printf("[Waveform] started  freq=%.2f Hz  mean=%.2f\n", freq, mean);

    while (!stop_signal) {
        /*
         * Yield DAC to file_write_thread during recording / playback.
         * The thread keeps looping so it can resume instantly when op_mode
         * returns to MODE_NORMAL without needing a restart.
         */
        if (op_mode != MODE_NORMAL) {
            usleep(10000);   /* 10 ms idle – low CPU, fast resume */
            continue;
        }

        /* Snapshot both amplitude and waveform type for this cycle */
        pthread_mutex_lock(&data_mutex);
        amp = g_amp;
        strncpy(local_type, g_wave_type, sizeof(local_type) - 1);
        local_type[sizeof(local_type) - 1] = '\0';
        pthread_mutex_unlock(&data_mutex);

        /* Unipolar DAC: ensure waveform never dips below 0 */
        if (mean < amp) mean = amp;

        /* ── Build sample table ────────────────────────────────────────── */
        for (i = 0; i < NUM_POINTS; i++) {
            if (strcmp(local_type, WAVE_SINE) == 0) {
                val = sinf(i * (2.0f * M_PI / NUM_POINTS)) * amp + mean;

            } else if (strcmp(local_type, WAVE_SQUARE) == 0) {
                val = (i < NUM_POINTS / 2) ? (mean + amp) : (mean - amp);

            } else if (strcmp(local_type, WAVE_TRIANGLE) == 0) {
                if (i < NUM_POINTS / 2)
                    val = mean - amp + (2.0f * amp * (i / (NUM_POINTS / 2.0f)));
                else
                    val = mean + amp - (2.0f * amp *
                          ((i - NUM_POINTS / 2.0f) / (NUM_POINTS / 2.0f)));

            } else {   /* sawtooth (default) */
                val = (mean - amp) + (2.0f * amp * ((float)i / NUM_POINTS));
            }

            if (val > 1.0f) val = 1.0f;
            if (val < 0.0f) val = 0.0f;
            data[i] = (uint16_t)(val * 0xFFFF);
        }

        /* ── Output one full cycle to the DAC ──────────────────────────── */
        for (i = 0; i < NUM_POINTS && !stop_signal && op_mode == MODE_NORMAL; i++) {
            out16(DA_CTLREG(iobase), DA_CTLREG_VAL);
            out16(DA_FIFOCLR(iobase), 0);
            out16(DA_DATA(iobase),    data[i]);
            nanosleep(&interval, NULL);
        }
    }

    printf("[Waveform] stopped\n");
    return NULL;
}

/* ══════════════════════════════════════════════════════════════════════════ */
/* Thread 3 : File I/O – on-demand recording and playback                    */
/* ══════════════════════════════════════════════════════════════════════════ */

/*
 * MODE_RECORDING : reads shared_pot_value (populated by adc_amp_thread, so
 *   no extra hardware call needed), writes to pot_data.txt, and also calls
 *   dac_write() so the oscilloscope shows the live potentiometer signal.
 *
 * MODE_PLAYBACK  : parses pot_data.txt line-by-line and replays each value
 *   to the DAC.  Checks stop_signal and op_mode each iteration so the user
 *   can abort with 'stop' before the file is exhausted.  Auto-returns to
 *   MODE_NORMAL when done.
 *
 * MODE_NORMAL    : idles at 50 ms.
 */
static void *file_write_thread(void *arg)
{
    uintptr_t     *iobase     = (uintptr_t *)arg;
    FILE          *fp         = NULL;
    int            count      = 0;
    unsigned short val;

    printf("[File] started (idle)\n");

    while (!stop_signal) {

        /* ── RECORDING ────────────────────────────────────────────────── */
        if (op_mode == MODE_RECORDING) {

            /* Open file on first entry into recording mode */
            if (fp == NULL) {
                fp = fopen(DEFAULT_FILENAME, "w");
                if (!fp) {
                    perror("[File] fopen");
                    op_mode = MODE_NORMAL;
                    continue;
                }
                count = 0;
                printf("[File] recording started -> %s\n", DEFAULT_FILENAME);
                printf("[File] pot signal now visible on oscilloscope\n");
            }

            pthread_mutex_lock(&data_mutex);
            val = shared_pot_value;
            pthread_mutex_unlock(&data_mutex);

            /* Write to file */
            fprintf(fp, "ADC Chan: %02x Data [%3d]: %4x\n",
                    0, count++, (unsigned int)val);

            /* Mirror to DAC so oscilloscope shows the pot signal */
            dac_write(iobase, val);

            usleep(SAMPLE_DELAY_US);   /* 100 ms between samples */

        /* ── End of RECORDING (mode changed externally by 'stop') ─────── */
        } else if (fp != NULL) {
            fclose(fp);
            printf("[File] recording stopped  (%d samples saved to %s)\n",
                   count, DEFAULT_FILENAME);
            fp = NULL;

        /* ── PLAYBACK ─────────────────────────────────────────────────── */
        } else if (op_mode == MODE_PLAYBACK) {
            FILE        *pf = fopen(DEFAULT_FILENAME, "r");
            unsigned int ch, idx, raw;

            if (!pf) {
                perror("[File] playback fopen");
                printf("[File] run 'record' first to create %s\n", DEFAULT_FILENAME);
                op_mode = MODE_NORMAL;
            } else {
                printf("[File] playback started from %s\n", DEFAULT_FILENAME);

                while (!stop_signal && op_mode == MODE_PLAYBACK &&
                       fscanf(pf, " ADC Chan: %x Data [%d]: %x",
                              &ch, &idx, &raw) == 3)
                {
                    dac_write(iobase, (unsigned short)(raw & 0xFFFF));
                    usleep(SAMPLE_DELAY_US);
                }

                fclose(pf);

                if (op_mode == MODE_PLAYBACK) {
                    /* File exhausted naturally – return to normal */
                    op_mode = MODE_NORMAL;
                    printf("[File] playback complete – waveform resumed\n");
                } else {
                    printf("[File] playback stopped by user\n");
                }
            }

        /* ── IDLE ─────────────────────────────────────────────────────── */
        } else {
            usleep(50000);
        }
    }

    /* Flush if still recording when program exits */
    if (fp) {
        fclose(fp);
        printf("[File] closed on exit (%d samples saved)\n", count);
    }
    printf("[File] stopped\n");
    return NULL;
}

/* ══════════════════════════════════════════════════════════════════════════ */
/* Thread 4 : User Command Interface                                          */
/* ══════════════════════════════════════════════════════════════════════════ */

/*
 * Prints a status line every 500 ms using select() with a timeout, so the
 * terminal stays live without blocking command input.
 *
 * Commands:
 *   change   – pause status; show numbered waveform menu; oscilloscope keeps
 *              running; resume status after selection
 *   record   – stop waveform DAC output; log ADC to file; pot → oscilloscope
 *   playback – replay recorded file to oscilloscope; auto-resume waveform
 *   stop     – stop recording or playback; resume waveform
 *   q / quit – terminate all threads and exit
 */
static void *user_cmd_thread(void *arg)
{
    wave_args_t   *w = (wave_args_t *)arg;
    fd_set         rfds;
    struct timeval tv;
    char           line[64];
    char           local_type[16];
    unsigned short raw;
    float          amp;
    int            ret;

    printf("Commands: change | record | playback | stop | q\n\n");

    while (!stop_signal) {
        /* ── Status line ───────────────────────────────────────────────── */
        pthread_mutex_lock(&data_mutex);
        raw = shared_pot_value;
        amp = g_amp;
        strncpy(local_type, g_wave_type, sizeof(local_type) - 1);
        local_type[sizeof(local_type) - 1] = '\0';
        pthread_mutex_unlock(&data_mutex);

        printf("[STATUS] type=%-8s  freq=%5.1f Hz  amp=%.4f  ADC=0x%04x  mode=%s\n",
               local_type, w->freq, amp, (unsigned int)raw, mode_str());

        /* ── Wait up to 500 ms for keyboard input ──────────────────────── */
        FD_ZERO(&rfds);
        FD_SET(STDIN_FILENO, &rfds);
        tv.tv_sec  = 0;
        tv.tv_usec = 500000;

        ret = select(STDIN_FILENO + 1, &rfds, NULL, NULL, &tv);
        if (ret <= 0) continue;   /* timeout – print status again */

        if (!fgets(line, sizeof(line), stdin)) break;
        line[strcspn(line, "\n\r")] = '\0';

        /* ── Parse command ─────────────────────────────────────────────── */

        /* CHANGE : interactive waveform menu ──────────────────────────── */
        if (strcmp(line, "change") == 0) {
            const char *new_type = NULL;

            printf("\n--- Select waveform (oscilloscope continues) ---\n");
            printf("  1. sine\n");
            printf("  2. square\n");
            printf("  3. triangle\n");
            printf("  4. sawtooth\n");
            printf("> ");
            fflush(stdout);

            if (fgets(line, sizeof(line), stdin)) {
                line[strcspn(line, "\n\r")] = '\0';

                if (strcmp(line, "1") == 0 || strcmp(line, WAVE_SINE) == 0)
                    new_type = WAVE_SINE;
                else if (strcmp(line, "2") == 0 || strcmp(line, WAVE_SQUARE) == 0)
                    new_type = WAVE_SQUARE;
                else if (strcmp(line, "3") == 0 || strcmp(line, WAVE_TRIANGLE) == 0)
                    new_type = WAVE_TRIANGLE;
                else if (strcmp(line, "4") == 0 || strcmp(line, WAVE_SAWTOOTH) == 0)
                    new_type = WAVE_SAWTOOTH;

                if (new_type) {
                    pthread_mutex_lock(&data_mutex);
                    strncpy(g_wave_type, new_type, sizeof(g_wave_type) - 1);
                    g_wave_type[sizeof(g_wave_type) - 1] = '\0';
                    pthread_mutex_unlock(&data_mutex);
                    printf("[CMD] waveform -> %s\n\n", new_type);
                } else {
                    printf("[CMD] invalid selection '%s' – waveform unchanged\n\n", line);
                }
            }

        /* RECORD ──────────────────────────────────────────────────────── */
        } else if (strcmp(line, "record") == 0) {
            if (op_mode != MODE_NORMAL) {
                printf("[CMD] type 'stop' first before starting a new operation\n");
            } else {
                op_mode = MODE_RECORDING;
                printf("[CMD] recording started – waveform paused\n");
            }

        /* PLAYBACK ────────────────────────────────────────────────────── */
        } else if (strcmp(line, "playback") == 0) {
            if (op_mode != MODE_NORMAL) {
                printf("[CMD] type 'stop' first before starting playback\n");
            } else {
                op_mode = MODE_PLAYBACK;
                printf("[CMD] playback started – waveform paused\n");
            }

        /* STOP ────────────────────────────────────────────────────────── */
        } else if (strcmp(line, "stop") == 0) {
            if (op_mode == MODE_NORMAL) {
                printf("[CMD] nothing is running\n");
            } else {
                op_mode = MODE_NORMAL;
                printf("[CMD] stopped – waveform resumed\n");
            }

        /* QUIT ────────────────────────────────────────────────────────── */
        } else if (strcmp(line, "q") == 0 || strcmp(line, "quit") == 0) {
            printf("[CMD] quit\n");
            stop_signal = 1;

        } else if (line[0] != '\0') {
            printf("[CMD] unknown: '%s'\n", line);
            printf("      Commands: change | record | playback | stop | q\n");
        }
    }

    printf("[User cmd] stopped\n");
    return NULL;
}

/* ══════════════════════════════════════════════════════════════════════════ */
/* Thread 5 : DIO Monitor – Hardware exit trigger                            */
/* ══════════════════════════════════════════════════════════════════════════ */

typedef struct {
    uintptr_t *iobase;
    uint8_t    initial_pattern;
} dio_args_t;

static void *dio_monitor_thread(void *arg)
{
    dio_args_t *d = (dio_args_t *)arg;
    uint8_t     current;

    printf("[DIO] watching Port A for pattern change (initial=0x%02x)\n",
           (unsigned int)d->initial_pattern);

    while (!stop_signal) {
        current = in8(DIO_PORTA(d->iobase));
        if (current != d->initial_pattern) {
            printf("\n[DIO] pattern changed: 0x%02x -> 0x%02x  Stopping...\n",
                   (unsigned int)d->initial_pattern, (unsigned int)current);
            stop_signal = 1;
            break;
        }
        usleep(50000);   /* 50 ms */
    }

    printf("[DIO] stopped\n");
    return NULL;
}

/* ══════════════════════════════════════════════════════════════════════════ */
/* Main : Argument validation, hardware init, thread supervisor              */
/* ══════════════════════════════════════════════════════════════════════════ */

int main(int argc, char *argv[])
{
    uintptr_t   iobase[DA_NUM_BARS];
    void       *pci_hdl;
    pthread_t   adc_tid, wave_tid, file_tid, user_tid, dio_tid;
    wave_args_t wave_args;
    dio_args_t  dio_args;
    float       init_amp;
    float       freq;
    float       mean;

    /* ── Argument count ───────────────────────────────────────────────── */
    if (argc != 5) {
        printf("Usage: %s <type> <freq> <amp> <mean>\n\n", argv[0]);
        printf("  type : sine | square | triangle | sawtooth\n");
        printf("  freq : output frequency in Hz       (e.g. 5.0)\n");
        printf("  amp  : initial amplitude  0.0 – 0.5 (e.g. 0.25)  [pot overrides live]\n");
        printf("  mean : DC offset          0.0 – 1.0 (e.g. 0.5)\n\n");
        printf("Runtime: change | record | playback | stop | q\n");
        return EXIT_FAILURE;
    }

    /* ── Validate type ────────────────────────────────────────────────── */
    if (!valid_wave_type(argv[1])) {
        fprintf(stderr, "Error: type '%s' is invalid. "
                "Use: sine | square | triangle | sawtooth\n", argv[1]);
        return EXIT_FAILURE;
    }

    /* ── Validate freq ────────────────────────────────────────────────── */
    freq = atof(argv[2]);
    if (freq <= 0.0f) {
        fprintf(stderr, "Error: freq must be > 0 (got '%s')\n", argv[2]);
        return EXIT_FAILURE;
    }

    /* ── Validate amp ─────────────────────────────────────────────────── */
    init_amp = atof(argv[3]);
    if (init_amp < 0.0f || init_amp > 0.5f) {
        fprintf(stderr, "Error: amp must be 0.0 – 0.5 (got '%s')\n", argv[3]);
        return EXIT_FAILURE;
    }

    /* ── Validate mean ────────────────────────────────────────────────── */
    mean = atof(argv[4]);
    if (mean < 0.0f || mean > 1.0f) {
        fprintf(stderr, "Error: mean must be 0.0 – 1.0 (got '%s')\n", argv[4]);
        return EXIT_FAILURE;
    }

    /* ── Acquire hardware I/O privileges ─────────────────────────────── */
    if (ThreadCtl(_NTO_TCTL_IO, 0) == -1) {
        perror("ThreadCtl");
        return EXIT_FAILURE;
    }

    /* ── PCI setup and ADC initialisation ────────────────────────────── */
    pci_hdl = pci_setup(iobase);
    adc_init(iobase);

    /* ── Initialise shared globals ────────────────────────────────────── */
    strncpy(g_wave_type, argv[1], sizeof(g_wave_type) - 1);
    g_wave_type[sizeof(g_wave_type) - 1] = '\0';
    g_amp       = init_amp;
    op_mode     = MODE_NORMAL;
    stop_signal = 0;

    /* ── DIO : configure Port A as input, snapshot initial pattern ───── */
    out8(DIO_CTLREG(iobase), 0x90);
    dio_args.iobase          = iobase;
    dio_args.initial_pattern = in8(DIO_PORTA(iobase));

    /* ── Prepare waveform thread arguments ───────────────────────────── */
    wave_args.iobase = iobase;
    wave_args.freq   = freq;
    wave_args.mean   = mean;

    printf("\nMA4830 CA2 – type=%s  freq=%.2f Hz  amp=%.2f  mean=%.2f\n",
           argv[1], freq, init_amp, mean);
    printf("DIO Port A initial pattern: 0x%02x\n",
           (unsigned int)dio_args.initial_pattern);
    printf("Exit: type 'q'  OR  change DIO Port A pattern\n\n");

    /* ── Spawn all five threads ───────────────────────────────────────── */
    if (pthread_create(&adc_tid,  NULL, adc_amp_thread,        iobase)      != 0 ||
        pthread_create(&wave_tid, NULL, waveform_output_thread, &wave_args) != 0 ||
        pthread_create(&file_tid, NULL, file_write_thread,      iobase)     != 0 ||
        pthread_create(&user_tid, NULL, user_cmd_thread,        &wave_args) != 0 ||
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

    /* ── Wait for stop_signal (set by 'q' or DIO change) ─────────────── */
    while (!stop_signal) {
        usleep(100000);
    }

    /* ── Clean shutdown ───────────────────────────────────────────────── */
    pthread_join(wave_tid, NULL);
    pthread_join(adc_tid,  NULL);
    pthread_join(file_tid, NULL);
    pthread_join(user_tid, NULL);
    pthread_join(dio_tid,  NULL);

    pci_detach_device(pci_hdl);
    printf("Done.\n");
    return EXIT_SUCCESS;
}
