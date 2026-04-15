#ifndef FILEIO_DA_H
#define FILEIO_DA_H

#include <stdio.h>
#include <stdint.h>
#include <pthread.h>
#include <sys/neutrino.h>
#include <unistd.h>

/* ── Sampling Parameters ─────────────────────────────────────────────────── */
#define SAMPLE_DELAY_US     100000      /* Record/playback interval: 100 ms  */
#define DEFAULT_FILENAME    "pot_data.txt"

/* ── Shared State (defined in fileio_da.c) ───────────────────────────────── */
extern pthread_mutex_t  data_mutex;
extern unsigned short   shared_pot_value;
extern volatile int     stop_signal;

/* ── Hardware Setup & ADC Functions (defined in fileio.c) ───────────────── */

/** pci_setup  - Attach to the PCI-DAS1602 and map all I/O base addresses.   */
void *pci_setup(uintptr_t *iobase);

/** adc_init   - Initialise ADC control registers (call once after pci_setup). */
void adc_init(uintptr_t *iobase);

/** adc_read   - Trigger a conversion on ADC channel 0 and return raw 16-bit. */
unsigned short adc_read(uintptr_t *iobase);

/** dac_write  - Write a 16-bit value to DAC channel 0. */
void dac_write(uintptr_t *iobase, unsigned short value);

/* ── Function Declarations ───────────────────────────────────────────────── */

/**
 * file_io_record_thread - Thread that reads ADC channel 0 (potentiometer) and
 * saves samples to a text file in the same format used by DAQ_test.c:
 *   "ADC Chan: %02x Data [%3d]: %4x\n"
 * Runs until stop_signal != 0.
 */
void *file_io_record_thread(void *arg);

/**
 * playback_file_to_dac - Reads a text file produced by file_io_record_thread
 * and replays each sample to the DAC via PCI-mapped iobase.
 *
 * @param iobase     PCI-mapped I/O base address array from pci_setup().
 * @param filename   Path to the text file to replay.
 * @param delay_us   Inter-sample delay in microseconds.
 *
 * Returns 0 on success, -1 on error.
 */
int playback_file_to_dac(uintptr_t *iobase, const char *filename, useconds_t delay_us);

#endif /* FILEIO_DA_H */