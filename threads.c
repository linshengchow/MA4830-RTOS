/*
 *  threads.c  —  MA4830 Major CA
 *
 *  All five worker threads live here, together with the shared state they
 *  operate on and the small accessor functions that enforce mutex discipline.
 *  Keeping this module self-contained means main.c can remain a thin
 *  orchestration layer (argument parsing, hardware init, thread spawn/join).
 *
 *  Coordination model
 *  ──────────────────
 *  • data_mutex     protects g_config, g_mode and shared_pot_value.
 *  • stop_signal    is a lock-free volatile flag read in every thread's
 *                   outer loop.  Any thread may set it to 1 to request
 *                   shutdown; all threads exit within ~50 ms.
 *  • g_mode         drives the cooperative DAC handoff between the
 *                   waveform thread and the file thread.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <time.h>
#include <pthread.h>
#include <termios.h>
#include <sys/select.h>
#include <sys/neutrino.h>
#include <hw/inout.h>

#include "waveform.h"
#include "fileio.h"
#include "threads.h"

/* ══════════════════════════════════════════════════════════════════════════
 *  Shared-State Definitions
 * ══════════════════════════════════════════════════════════════════════════ */

pthread_mutex_t data_mutex       = PTHREAD_MUTEX_INITIALIZER;
wave_config_t   g_config         = { "sine", 1.0f, 0.25f, 0.5f };
op_mode_t       g_mode           = MODE_NORMAL;
unsigned short  shared_pot_value = 0;   /* latest A0 ADC sample (amplitude) */
unsigned short  shared_dac_value = 0;   /* latest value written to DAC      */
volatile int    stop_signal      = 0;

/* ══════════════════════════════════════════════════════════════════════════
 *  Shared-State Helpers
 * ══════════════════════════════════════════════════════════════════════════ */

void cfg_snapshot(wave_config_t *out)
{
    pthread_mutex_lock(&data_mutex);
    *out = g_config;
    pthread_mutex_unlock(&data_mutex);
}

void cfg_set_type(const char *type)
{
    pthread_mutex_lock(&data_mutex);
    strncpy(g_config.type, type, sizeof(g_config.type) - 1);
    g_config.type[sizeof(g_config.type) - 1] = '\0';
    pthread_mutex_unlock(&data_mutex);
}

void cfg_set_freq(float freq)
{
    if (freq <= 0.0f) return;
    pthread_mutex_lock(&data_mutex);
    g_config.freq = freq;
    pthread_mutex_unlock(&data_mutex);
}

void cfg_set_amp(float amp)
{
    if (amp < 0.0f) amp = 0.0f;
    if (amp > 0.5f) amp = 0.5f;
    pthread_mutex_lock(&data_mutex);
    g_config.amp = amp;
    pthread_mutex_unlock(&data_mutex);
}

void cfg_set_mean(float mean)
{
    if (mean < 0.0f) mean = 0.0f;
    if (mean > 1.0f) mean = 1.0f;
    pthread_mutex_lock(&data_mutex);
    g_config.mean = mean;
    pthread_mutex_unlock(&data_mutex);
}

op_mode_t get_mode(void)
{
    op_mode_t m;
    pthread_mutex_lock(&data_mutex);
    m = g_mode;
    pthread_mutex_unlock(&data_mutex);
    return m;
}

void set_mode(op_mode_t m)
{
    pthread_mutex_lock(&data_mutex);
    g_mode = m;
    pthread_mutex_unlock(&data_mutex);
}

/* ══════════════════════════════════════════════════════════════════════════
 *  Internal Utilities
 * ══════════════════════════════════════════════════════════════════════════ */

/* EINTR-safe nanosleep wrapper: if a signal interrupts the sleep, resume
 * with the remaining time so the caller waits for its full intended
 * interval.  Called from the waveform inner loop where timing accuracy
 * matters most. */
static void sleep_ns(long ns)
{
    struct timespec req, rem;
    req.tv_sec  = ns / 1000000000L;
    req.tv_nsec = ns % 1000000000L;
    while (nanosleep(&req, &rem) == -1 && errno == EINTR) {
        req = rem;
    }
}

static const char *mode_str(op_mode_t m)
{
    switch (m) {
        case MODE_RECORDING: return "REC";
        case MODE_PLAYBACK:  return "PB ";
        case MODE_NORMAL:    /* fallthrough */
        default:             return "---";
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 *  Thread 1 — ADC (Amplitude + Frequency)
 * ══════════════════════════════════════════════════════════════════════════
 *
 *  Reads two potentiometers every 50 ms:
 *    A0  →  amplitude  mapped linearly to  0.0 – 0.5
 *    A1  →  frequency  mapped logarithmically to FREQ_MIN – FREQ_MAX Hz
 *
 *  Logarithmic mapping is used for frequency because it feels natural on a
 *  physical knob — equal angular travel gives equal perceived pitch change
 *  (one octave per division), matching how analogue function generators work.
 *
 *  The arrow keys own 'mean' and 'wave type', so there is no conflict.
 */
#define FREQ_MIN   0.5f     /* Hz — minimum output frequency */
#define FREQ_MAX   500.0f   /* Hz — maximum output frequency */

void *adc_amp_thread(void *arg)
{
    uintptr_t     *iobase = (uintptr_t *)arg;
    unsigned short raw_a0, raw_a1;
    float          amp, freq;

    printf("[ADC] started  (A0=amplitude  A1=frequency)\n");

    while (!stop_signal) {
        /* ── Channel 0 : Amplitude 0.0 – 0.5 ── */
        raw_a0 = adc_read_ch(iobase, 0);
        amp    = (raw_a0 / 65535.0f) * 0.5f;

        /* ── Channel 1 : Frequency FREQ_MIN – FREQ_MAX (log scale) ── */
        raw_a1 = adc_read_ch(iobase, 1);
        freq   = FREQ_MIN * powf(FREQ_MAX / FREQ_MIN,
                                 raw_a1 / 65535.0f);

        pthread_mutex_lock(&data_mutex);
        shared_pot_value = raw_a0;   /* status display shows A0 */
        g_config.amp     = amp;
        g_config.freq    = freq;
        pthread_mutex_unlock(&data_mutex);

        usleep(50000);   /* 50 ms */
    }

    printf("[ADC] stopped\n");
    return NULL;
}

/* ══════════════════════════════════════════════════════════════════════════
 *  Thread 2 — Waveform Output
 * ══════════════════════════════════════════════════════════════════════════
 *
 *  Generates the configured waveform one cycle at a time and pushes it to
 *  the DAC.  All waveform parameters are read once at the top of each
 *  cycle from a mutex-protected snapshot, so an update made by the user
 *  while a cycle is in progress takes effect on the very next cycle.
 *
 *  The thread respects g_mode and yields the DAC during recording and
 *  playback, rather than being killed and recreated.  This means the
 *  transition back to normal output is effectively instantaneous.
 */
void *waveform_output_thread(void *arg)
{
    uintptr_t      *iobase = (uintptr_t *)arg;
    uint16_t        data[NUM_POINTS];
    wave_config_t   cfg;
    int             i;
    long            delay_ns;
    struct _clockperiod clk;

    /* Tighten the system clock resolution so nanosleep() delivers the
     * microsecond-level accuracy we need for audible-range waveforms. */
    clk.nsec  = CLK_PERIOD_NS;
    clk.fract = 0;
    ClockPeriod(CLOCK_REALTIME, &clk, NULL, 0);

    printf("[Waveform] started\n");

    while (!stop_signal) {
        /* Yield the DAC during RECORDING and PLAYBACK.
         * Both modes use the DAC to show the pot signal on the scope —
         * RECORDING: live pot value mirrored to DAC while writing to file
         * PLAYBACK:  recorded pot values replayed to DAC from file       */
        if (get_mode() != MODE_NORMAL) {
            usleep(10000);
            continue;
        }

        /* One atomic snapshot per cycle */
        cfg_snapshot(&cfg);

        /* Rebuild the sample table.  generate_samples() applies the
         * mean>=amp clamp internally, so cfg.mean is never mutated. */
        generate_samples(data, NUM_POINTS, cfg.type, cfg.amp, cfg.mean);

        /* Inter-sample delay derived from the latest frequency */
        delay_ns = (long)((1.0 / cfg.freq / NUM_POINTS) * 1e9);

        /* ── Output one full cycle.  Check escape conditions between
         * samples so a mode change or shutdown takes effect promptly. ── */
        for (i = 0; i < NUM_POINTS; i++) {
            if (stop_signal || get_mode() != MODE_NORMAL) break;
            dac_write(iobase, data[i]);
            pthread_mutex_lock(&data_mutex);
            shared_dac_value = data[i];
            pthread_mutex_unlock(&data_mutex);
            sleep_ns(delay_ns);
        }
    }

    printf("[Waveform] stopped\n");
    return NULL;
}

/* ══════════════════════════════════════════════════════════════════════════
 *  Thread 3 — File I/O
 * ══════════════════════════════════════════════════════════════════════════
 *
 *  Implements the Function 4 file I/O requirement:
 *    "Read a potentiometer port and write out the data to disk.
 *     Output this pattern to D/A.  Whilst the data is being read,
 *     provide an illustration on the Oscilloscope."
 *
 *  MODE_RECORDING
 *    Reads the latest A0 pot value from shared_pot_value (kept fresh by
 *    the ADC thread — no extra hardware round-trip needed), writes each
 *    sample to pot_data.txt, AND mirrors it to the DAC so the oscilloscope
 *    shows the live potentiometer signal in real time.
 *    The waveform thread yields the DAC for the duration.
 *
 *  MODE_PLAYBACK
 *    Reads pot_data.txt back and replays each value to the DAC so the
 *    oscilloscope shows exactly what was recorded.
 *    Abortable mid-file via 'stop'.  Auto-returns to MODE_NORMAL on EOF.
 *
 *  MODE_NORMAL
 *    Idle — waveform thread has the DAC.
 */
void *file_write_thread(void *arg)
{
    uintptr_t     *iobase = (uintptr_t *)arg;
    FILE          *fp     = NULL;
    int            count  = 0;
    unsigned short val;
    op_mode_t      mode;

    printf("[File] started (idle)\n");

    while (!stop_signal) {
        mode = get_mode();

        /* ── RECORDING ──────────────────────────────────────────────── */
        if (mode == MODE_RECORDING) {

            if (fp == NULL) {
                fp = fopen(DEFAULT_FILENAME, "w");
                if (!fp) {
                    perror("[File] fopen");
                    set_mode(MODE_NORMAL);
                    continue;
                }
                count = 0;
                printf("[File] recording pot (A0) -> %s\n", DEFAULT_FILENAME);
                printf("[File] pot signal now visible on oscilloscope\n");
            }

            /* Read the latest pot value from the ADC thread */
            pthread_mutex_lock(&data_mutex);
            val = shared_pot_value;
            pthread_mutex_unlock(&data_mutex);

            /* Write to disk */
            fprintf(fp, "POT [%4d]: %04x\n", count++, (unsigned int)val);
            fflush(fp);

            /* Mirror to DAC — this is the oscilloscope illustration */
            dac_write(iobase, val);

            usleep(SAMPLE_DELAY_US);   /* 100 ms between samples */

        /* ── End of RECORDING (user typed 'stop') ───────────────────── */
        } else if (fp != NULL) {
            fclose(fp);
            printf("[File] recording stopped (%d samples saved to %s)\n",
                   count, DEFAULT_FILENAME);
            fp = NULL;

        /* ── PLAYBACK ───────────────────────────────────────────────── */
        } else if (mode == MODE_PLAYBACK) {
            FILE        *pf = fopen(DEFAULT_FILENAME, "r");
            unsigned int idx, raw;

            if (!pf) {
                perror("[File] playback fopen");
                printf("[File] run 'record' first to create %s\n",
                       DEFAULT_FILENAME);
                set_mode(MODE_NORMAL);
            } else {
                printf("[File] playback started from %s\n", DEFAULT_FILENAME);

                while (!stop_signal && get_mode() == MODE_PLAYBACK &&
                       fscanf(pf, " POT [%d]: %x", &idx, &raw) == 2)
                {
                    dac_write(iobase, (unsigned short)(raw & 0xFFFF));
                    usleep(SAMPLE_DELAY_US);
                }
                fclose(pf);

                if (get_mode() == MODE_PLAYBACK) {
                    set_mode(MODE_NORMAL);
                    printf("[File] playback complete - waveform resumed\n");
                } else {
                    printf("[File] playback stopped by user\n");
                }
            }

        /* ── IDLE ───────────────────────────────────────────────────── */
        } else {
            usleep(50000);
        }
    }

    if (fp) {
        fclose(fp);
        printf("[File] closed on exit (%d samples saved)\n", count);
    }
    printf("[File] stopped\n");
    return NULL;
}

/* ══════════════════════════════════════════════════════════════════════════
 *  Thread 4 — User Command Interface
 * ══════════════════════════════════════════════════════════════════════════
 *
 *  Terminal is switched to raw (non-canonical, no-echo) mode so arrow keys
 *  can be read without waiting for Enter.  The thread still supports typed
 *  commands for operations that need arguments (record, playback, stop, q).
 *
 *  Control layout:
 *    ↑ / ↓      mean   +0.05 / -0.05
 *    ← / →      wave type   round-robin (left / right)
 *    A0 pot     amplitude   (continuous, ADC thread)
 *    A1 pot     frequency   (continuous, ADC thread)
 *    typed cmds record | playback | stop | q
 *
 *  Raw mode is restored on exit so the terminal is not left broken.
 */

/* ── Wave type round-robin ─────────────────────────────────────────────── */

static const char *WAVE_ORDER[] = {
    WAVE_SINE, WAVE_SQUARE, WAVE_TRIANGLE, WAVE_SAWTOOTH
};
#define N_WAVES 4

/*
 * wave_index - Return the index of the current type in WAVE_ORDER, or 0
 * if not found (safe fallback to WAVE_SINE).
 */
static int wave_index(const char *type)
{
    int i;
    for (i = 0; i < N_WAVES; i++)
        if (strcmp(type, WAVE_ORDER[i]) == 0) return i;
    return 0;
}

static void wave_rotate(int delta)
{
    wave_config_t cfg;
    int           idx;
    const char   *new_type;

    cfg_snapshot(&cfg);
    idx      = (wave_index(cfg.type) + delta + N_WAVES) % N_WAVES;
    new_type = WAVE_ORDER[idx];
    cfg_set_type(new_type);
    printf("\r[KEY] wave type -> %-9s\r\n", new_type);
}

/* ── Mean step via arrow keys ──────────────────────────────────────────── */
#define MEAN_STEP 0.05f

static void mean_step(float delta)
{
    wave_config_t cfg;
    float         new_mean;

    cfg_snapshot(&cfg);
    new_mean = cfg.mean + delta;
    if (new_mean < 0.0f) new_mean = 0.0f;
    if (new_mean > 1.0f) new_mean = 1.0f;
    cfg_set_mean(new_mean);
    printf("\r[KEY] mean -> %.2f        \r\n", new_mean);
}

/* ── Raw terminal mode ─────────────────────────────────────────────────── */

static struct termios s_orig_termios;
static int            s_raw_active = 0;

static void enable_raw_mode(void)
{
    struct termios raw;
    tcgetattr(STDIN_FILENO, &s_orig_termios);
    raw = s_orig_termios;

    /* Disable echo and line-buffering flags.
     * ISIG is intentionally kept enabled so Ctrl+C still generates SIGINT
     * and reaches the handle_sigint() handler installed in main.c.        */
    raw.c_lflag &= (tcflag_t)~(ICANON | ECHO | ECHOE | ECHOK);
    raw.c_iflag &= (tcflag_t)~(ICRNL | IXON);
    raw.c_cc[VMIN]  = 0;   /* return immediately even with 0 bytes      */
    raw.c_cc[VTIME] = 1;   /* 100 ms read timeout (units of 1/10 s)     */

    tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);
    s_raw_active = 1;
}

void disable_raw_mode(void)
{
    if (s_raw_active) {
        tcsetattr(STDIN_FILENO, TCSAFLUSH, &s_orig_termios);
        s_raw_active = 0;
    }
}

/* ── Arrow key detection ───────────────────────────────────────────────── */
/*
 * Arrow keys arrive as a 3-byte ESC sequence: ESC '[' <letter>
 *   A = Up    B = Down    C = Right    D = Left
 *
 * We attempt two extra single-byte reads after seeing ESC.  Because the
 * terminal is in raw mode with VTIME=1, these reads return immediately if
 * no further bytes arrive, so a standalone ESC keypress does not block.
 */
static void handle_escape_sequence(void)
{
    char seq[2] = {0, 0};

    if (read(STDIN_FILENO, &seq[0], 1) != 1) return;
    if (read(STDIN_FILENO, &seq[1], 1) != 1) return;
    if (seq[0] != '[') return;

    switch (seq[1]) {
        case 'A': mean_step(+MEAN_STEP);  break;   /* Up    → mean up   */
        case 'B': mean_step(-MEAN_STEP);  break;   /* Down  → mean down */
        case 'C': wave_rotate(+1);        break;   /* Right → next type */
        case 'D': wave_rotate(-1);        break;   /* Left  → prev type */
        default:  break;
    }
}

/* ── Help display ──────────────────────────────────────────────────────── */

static void print_help(void)
{
    printf("\r\n");
    printf("+-----------------------------------------------------+\r\n");
    printf("|         MA4830 Waveform Generator  -  Help         |\r\n");
    printf("+------------------+----------------------------------+\r\n");
    printf("| HARDWARE INPUTS  |                                  |\r\n");
    printf("|  A0 pot          |  Amplitude  (0.0 - 0.5)          |\r\n");
    printf("|  A1 pot          |  Frequency  (0.5 - 500 Hz, log)  |\r\n");
    printf("+------------------+----------------------------------+\r\n");
    printf("| ARROW KEYS       |                                  |\r\n");
    printf("|  Up / Down       |  Mean  +/- 0.05                  |\r\n");
    printf("|  Left / Right    |  Wave type  (round-robin)        |\r\n");
    printf("+------------------+----------------------------------+\r\n");
    printf("| TYPED COMMANDS   |                                  |\r\n");
    printf("|  change <type>   |  sine | square | triangle |      |\r\n");
    printf("|                  |  sawtooth  (or use arrow keys)   |\r\n");
    printf("|  record          |  Record pot to file + scope      |\r\n");
    printf("|  playback        |  Replay recorded file to scope   |\r\n");
    printf("|  stop            |  End record / playback           |\r\n");
    printf("|  help            |  Show this screen                |\r\n");
    printf("|  q               |  Quit program                    |\r\n");
    printf("+------------------+----------------------------------+\r\n");
    printf("| EXIT             |  Type q  OR  change DIO Port A   |\r\n");
    printf("+------------------+----------------------------------+\r\n");
    printf("\r\n");
}

/* ── Typed command dispatcher ──────────────────────────────────────────── */

static void dispatch_command(const char *line)
{
    /* change <type>  — update waveform type from keyboard               */
    if (strncmp(line, "change", 6) == 0) {
        const char *arg = line + 6;
        while (*arg == ' ') arg++;   /* skip whitespace */

        if (*arg == '\0') {
            /* No argument — print available types */
            printf("\r\n[CMD] usage: change <sine|square|triangle|sawtooth>\r\n");
            printf("[CMD] or use left/right arrow keys to cycle types\r\n");
        } else if (valid_wave_type(arg)) {
            cfg_set_type(arg);
            printf("\r\n[CMD] wave type -> %s\r\n", arg);
        } else {
            printf("\r\n[CMD] unknown type '%s'\r\n", arg);
            printf("[CMD] valid types: sine | square | triangle | sawtooth\r\n");
        }

    } else if (strcmp(line, "record") == 0) {
        if (get_mode() != MODE_NORMAL)
            printf("\r\n[CMD] type 'stop' first\r\n");
        else {
            set_mode(MODE_RECORDING);
            printf("\r\n[CMD] recording started - pot signal on oscilloscope\r\n");
        }

    } else if (strcmp(line, "playback") == 0) {
        if (get_mode() != MODE_NORMAL)
            printf("\r\n[CMD] type 'stop' first\r\n");
        else {
            set_mode(MODE_PLAYBACK);
            printf("\r\n[CMD] playback started\r\n");
        }

    } else if (strcmp(line, "stop") == 0) {
        if (get_mode() == MODE_NORMAL)
            printf("\r\n[CMD] nothing is running\r\n");
        else {
            set_mode(MODE_NORMAL);
            printf("\r\n[CMD] stopped - waveform resumed\r\n");
        }

    } else if (strcmp(line, "help") == 0) {
        print_help();

    } else if (strcmp(line, "q") == 0 || strcmp(line, "quit") == 0) {
        printf("\r\n[CMD] quitting...\r\n");
        stop_signal = 1;

    } else if (line[0] != '\0') {
        printf("\r\n[CMD] unknown command '%s' - type 'help' for a list\r\n",
               line);
    }
}

/* ── Status line ───────────────────────────────────────────────────────── */

static void print_status(void)
{
    wave_config_t  cfg;
    unsigned short raw;

    cfg_snapshot(&cfg);
    pthread_mutex_lock(&data_mutex);
    raw = shared_pot_value;
    pthread_mutex_unlock(&data_mutex);

    /* \r moves to column 0 before printing so the line always overwrites
     * cleanly in raw mode regardless of cursor position.               */
    printf("\r[STATUS] %-8s  %6.2f Hz  amp=%.3f  mean=%.2f  "
           "ADC=0x%04x  [%s]\r\n",
           cfg.type, cfg.freq, cfg.amp, cfg.mean,
           (unsigned int)raw, mode_str(get_mode()));
}

/* ── Thread entry point ────────────────────────────────────────────────── */

void *user_cmd_thread(void *arg)
{
    char  c;
    char  linebuf[64];
    int   linelen    = 0;
    int   status_ctr = 0;   /* print status every ~500 ms (5 × 100 ms ticks) */

    (void)arg;

    enable_raw_mode();
    print_help();   /* show full help on startup */

    while (!stop_signal) {
        /* Print status every ~500 ms, but only when not mid-typing.
         * \r\n before status ensures it always starts at column 0.     */
        if (status_ctr++ >= 5) {
            if (linelen == 0)
                print_status();
            status_ctr = 0;
        }

        /* Non-blocking read — VTIME=1 gives 100 ms timeout */
        if (read(STDIN_FILENO, &c, 1) != 1)
            continue;

        if (c == '\x1b') {
            /* Arrow key escape sequence */
            handle_escape_sequence();
            status_ctr = 5;   /* show updated status immediately after */

        } else if (c == '\r' || c == '\n') {
            /* Enter — dispatch buffered command */
            linebuf[linelen] = '\0';
            dispatch_command(linebuf);
            linelen    = 0;
            status_ctr = 5;

        } else if (c == 127 || c == '\b') {
            /* Backspace — erase last character on screen */
            if (linelen > 0) {
                linelen--;
                write(STDOUT_FILENO, "\b \b", 3);
            }

        } else if (c >= 0x20 && c < 0x7F) {
            /* Printable — echo and accumulate */
            if (linelen < (int)sizeof(linebuf) - 1) {
                linebuf[linelen++] = c;
                write(STDOUT_FILENO, &c, 1);
            }
        }
    }

    disable_raw_mode();
    printf("\n[User cmd] stopped\n");
    return NULL;
}

/* ══════════════════════════════════════════════════════════════════════════
 *  Thread 5 — DIO Monitor
 * ══════════════════════════════════════════════════════════════════════════
 *
 *  Provides an out-of-band hardware exit path: change any DIO Port A bit
 *  from its startup pattern and the program shuts down cleanly, going
 *  through exactly the same path as the 'q' command.
 */
void *dio_monitor_thread(void *arg)
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