#ifndef THREADS_H
#define THREADS_H

#include <stdint.h>
#include <pthread.h>

/* ══════════════════════════════════════════════════════════════════════════
 *  Shared State Types
 * ══════════════════════════════════════════════════════════════════════════ */

/*
 * op_mode_t - Cooperative DAC ownership.
 *
 * The waveform thread and the file thread share exactly one DAC channel.
 * Rather than stopping and restarting threads (expensive and error-prone),
 * both threads poll this flag on every cycle and whichever one "owns" the
 * current mode drives the DAC; the other yields into a short sleep.  State
 * transitions are made by user_cmd_thread or by file_write_thread at the
 * end of playback.
 */
typedef enum {
    MODE_NORMAL    = 0,   /* waveform_output_thread owns the DAC      */
    MODE_RECORDING = 1,   /* file_write_thread: ADC -> file + DAC     */
    MODE_PLAYBACK  = 2    /* file_write_thread: file -> DAC           */
} op_mode_t;

/*
 * wave_config_t - All live-mutable waveform parameters in one struct.
 *
 * Grouping these fields means a single mutex lock yields a consistent
 * snapshot for one whole cycle — no risk of the waveform thread reading
 * an updated 'freq' but a stale 'amp' (or vice versa).  Always access
 * through cfg_snapshot() / cfg_set_*() helpers; never touch the fields
 * directly from another translation unit.
 */
typedef struct {
    char  type[16];    /* One of WAVE_SINE, WAVE_SQUARE, ...          */
    float freq;        /* Hz, strictly > 0                            */
    float amp;         /* 0.0 – 0.5 of full scale                     */
    float mean;        /* 0.0 – 1.0 of full scale                     */
} wave_config_t;

/* ══════════════════════════════════════════════════════════════════════════
 *  Global Shared State (defined in threads.c)
 * ══════════════════════════════════════════════════════════════════════════
 *
 *  All of the following are guarded by data_mutex unless noted otherwise.
 *  stop_signal is a volatile flag that any thread may set to request a
 *  global shutdown — it is read without locking because a stale FALSE
 *  simply means one more loop iteration before exit.
 */

extern pthread_mutex_t data_mutex;        /* guards the fields below    */
extern wave_config_t   g_config;          /* live waveform parameters   */
extern op_mode_t       g_mode;            /* DAC ownership state        */
extern unsigned short  shared_pot_value;  /* latest A0 ADC sample       */
extern unsigned short  shared_dac_value;  /* latest value written to DAC*/

extern volatile int    stop_signal;       /* lock-free; set to request
                                              shutdown from any thread   */

/* ══════════════════════════════════════════════════════════════════════════
 *  Shared-State Helpers
 * ══════════════════════════════════════════════════════════════════════════
 *
 *  These wrap the mutex acquire/release sequence so callers cannot forget
 *  to lock.  Keeping the critical section as short as possible (a few
 *  field copies) means contention is never an issue in practice.
 */

/** cfg_snapshot - Copy the current config into *out atomically. */
void cfg_snapshot(wave_config_t *out);

/** cfg_set_type - Update waveform type (must be a WAVE_* string). */
void cfg_set_type(const char *type);

/** cfg_set_freq - Update output frequency in Hz (must be > 0). */
void cfg_set_freq(float freq);

/** cfg_set_amp  - Update amplitude (clamped to 0.0 – 0.5). */
void cfg_set_amp(float amp);

/** cfg_set_mean - Update DC offset (clamped to 0.0 – 1.0). */
void cfg_set_mean(float mean);

/** get_mode / set_mode - Mutex-guarded accessors for the op_mode flag. */
op_mode_t get_mode(void);
void      set_mode(op_mode_t m);

/* ══════════════════════════════════════════════════════════════════════════
 *  Thread Entry Points
 * ══════════════════════════════════════════════════════════════════════════
 *
 *  Each thread takes a void* argument whose concrete type is documented
 *  alongside the function.  All threads exit their main loop when
 *  stop_signal becomes non-zero.
 */

/*
 * adc_amp_thread - Polls ADC channel 0 every 50 ms and updates the live
 * amplitude from the potentiometer.
 *
 * @param arg   uintptr_t *iobase
 */
void *adc_amp_thread(void *arg);

/*
 * waveform_output_thread - Continuously generates and outputs the
 * configured waveform on the DAC.  Yields the DAC during RECORDING and
 * PLAYBACK modes by polling g_mode.  Resumes instantly when g_mode
 * returns to NORMAL — the thread is never stopped or restarted.
 *
 * @param arg   uintptr_t *iobase
 */
void *waveform_output_thread(void *arg);

/*
 * file_write_thread - Handles both on-demand recording and playback,
 * selected via g_mode.  Automatically returns g_mode to NORMAL when a
 * playback file is exhausted.
 *
 * @param arg   uintptr_t *iobase
 */
void *file_write_thread(void *arg);

/*
 * user_cmd_thread - Status display + command parser driven by a 500 ms
 * non-blocking select() on stdin.  Commands: change, freq, mean,
 * record, playback, stop, q/quit.
 *
 * @param arg   uintptr_t *iobase  (unused at present, but reserved so
 *                                  future commands can touch hardware)
 */
void *user_cmd_thread(void *arg);

/** disable_raw_mode - Restore terminal to its original cooked mode.
 *  Called by main() after all threads are joined so the terminal is left
 *  clean regardless of how the program exits. */
void disable_raw_mode(void);

/*
 * dio_args_t - Argument passed to dio_monitor_thread; bundled into a
 * struct because the thread needs both the iobase pointer and the
 * initial pattern snapshot taken in main().
 */
typedef struct {
    uintptr_t *iobase;
    uint8_t    initial_pattern;
} dio_args_t;

/*
 * dio_monitor_thread - Polls DIO Port A every 50 ms.  Asserts
 * stop_signal the moment the bit pattern differs from the snapshot
 * captured at startup, giving the user a hardware exit path.
 *
 * @param arg   dio_args_t *
 */
void *dio_monitor_thread(void *arg);

#endif /* THREADS_H */