#ifndef FILEIO_DA_H
#define FILEIO_DA_H

#include <stdio.h>
#include <stdint.h>
#include <unistd.h>

/* ── Sampling Parameters ─────────────────────────────────────────────────── */
#define SAMPLE_DELAY_US     100000      /* Record/playback interval: 100 ms  */
#define DEFAULT_FILENAME    "pot_data.txt"

/* ── Hardware Setup & Primitives ─────────────────────────────────────────── */

/** pci_setup  - Attach to the PCI-DAS1602 and map all I/O base addresses.   */
void *pci_setup(uintptr_t *iobase);

/** adc_init   - Initialise ADC control registers (call once after pci_setup). */
void adc_init(uintptr_t *iobase);

/** adc_read_ch - Trigger a conversion on channel 'ch' and return raw 16-bit. */
unsigned short adc_read_ch(uintptr_t *iobase, int ch);

/** adc_read   - Convenience wrapper: trigger conversion on channel 0. */
unsigned short adc_read(uintptr_t *iobase);

/** dac_write  - Write a 16-bit value to DAC channel 0.                      */
void dac_write(uintptr_t *iobase, unsigned short value);

/* ── File Helpers ────────────────────────────────────────────────────────── */

/**
 * playback_file_to_dac - Reads a text file produced by the recording thread
 * and replays each sample to the DAC.  Used by the standalone 'fileio'
 * diagnostic binary.  The integrated program uses file_write_thread() in
 * threads.c, which performs the same parsing inline but honours stop_signal
 * and op_mode for interactive abort.
 *
 * @param iobase     PCI-mapped I/O base address array from pci_setup().
 * @param filename   Path to the text file to replay.
 * @param delay_us   Inter-sample delay in microseconds.
 * @return 0 on success, -1 on error.
 */
int playback_file_to_dac(uintptr_t *iobase, const char *filename,
                         useconds_t delay_us);

#endif /* FILEIO_DA_H */