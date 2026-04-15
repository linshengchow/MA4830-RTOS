#ifndef FILEIO_DA_H
#define FILEIO_DA_H

#include <stdio.h>
#include <pthread.h>
#include <sys/neutrino.h>
#include <hw/inout.h>

/* ── Hardware Addresses ──────────────────────────────────────────────────── */
/* Replace these with the actual addresses used by your lab hardware.        */
#define ADC_BASE_ADDR       0x280
#define DAC_BASE_ADDR       0x290

/* ── Sampling Parameters ─────────────────────────────────────────────────── */
#define SAMPLE_DELAY_US     100000      /* Record/playback interval: 100 ms  */
#define DEFAULT_FILENAME    "pot_data.bin"

/* ── Shared State (defined in fileio_da.c) ───────────────────────────────── */
extern pthread_mutex_t  data_mutex;
extern unsigned short   shared_pot_value;
extern volatile int     stop_signal;

/* ── Hardware Helpers ────────────────────────────────────────────────────── */

/**
 * read_adc - Reads one 16-bit sample from the ADC (potentiometer input).
 * Returns the raw ADC value.
 */
static inline unsigned short read_adc(void)
{
    return in16(ADC_BASE_ADDR);
}

/**
 * write_dac - Writes one 16-bit value to the DAC output.
 * @param value  Raw 16-bit sample to output.
 */
static inline void write_dac(unsigned short value)
{
    out16(DAC_BASE_ADDR, value);
}

/* ── Function Declarations ───────────────────────────────────────────────── */

/**
 * file_io_record_thread - Thread that reads ADC and saves samples to a binary file.
 *
 * @param arg  Pointer to a const char* filename string.
 *
 * Continuously reads from the ADC into shared_pot_value (mutex-protected)
 * and writes each sample to the binary file. Runs until stop_signal != 0.
 */
void *file_io_record_thread(void *arg);

/**
 * playback_file_to_dac - Reads a recorded binary file and replays it to the DAC.
 *
 * @param filename   Path to the binary file produced by file_io_record_thread.
 * @param delay_us   Inter-sample delay in microseconds.
 *
 * Returns 0 on success, -1 on error.
 */
int playback_file_to_dac(const char *filename, useconds_t delay_us);

#endif /* FILEIO_DA_H */