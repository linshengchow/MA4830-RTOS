#ifndef WAVEFORM_DA_H
#define WAVEFORM_DA_H

#include <stdint.h>
#include <hw/pci.h>
#include <hw/inout.h>
#include <sys/neutrino.h>
#include <sys/mman.h>

/* ── PCI Device Identity ─────────────────────────────────────────────────── */
#define DA_VENDOR_ID    0x1307
#define DA_DEVICE_ID    0x01

/* ── DAC Register Offsets (relative to iobase[]) ────────────────────────── */
#define DA_CTLREG(iobase)   ((iobase)[1] + 8)
#define DA_DATA(iobase)     ((iobase)[4] + 0)
#define DA_FIFOCLR(iobase)  ((iobase)[4] + 2)

/* ── Waveform Parameters ─────────────────────────────────────────────────── */
#define NUM_POINTS          20          /* Samples per waveform cycle        */
#define DA_IO_RANGE         0x0F        /* I/O region size per BAR           */
#define DA_NUM_BARS         5           /* Number of PCI BARs to map         */
#define DA_CTLREG_VAL       0x0a23      /* Control register value, channel 0 */
#define CLK_PERIOD_NS       10000       /* System clock resolution (10 µs)   */

/* ── Digital I/O Register Offsets (relative to iobase[3]) ───────────────── */
/* Matches DAQ_test.c — configure with out8(DIO_CTLREG(b), 0x90) for        */
/* Port A = input, Port B/C = output.                                        */
#define DIO_PORTA(b)    ((b)[3] + 4)    /* Badr3 + 4 : Port A (input)       */
#define DIO_PORTB(b)    ((b)[3] + 5)    /* Badr3 + 5 : Port B (output)      */
#define DIO_PORTC(b)    ((b)[3] + 6)    /* Badr3 + 6 : Port C               */
#define DIO_CTLREG(b)   ((b)[3] + 7)    /* Badr3 + 7 : DIO control register */

/* ── Waveform Type Strings ───────────────────────────────────────────────── */
#define WAVE_SINE       "sine"
#define WAVE_SQUARE     "square"
#define WAVE_TRIANGLE   "triangle"
#define WAVE_SAWTOOTH   "sawtooth"

/* ── Public API ──────────────────────────────────────────────────────────── */

/**
 * generate_samples - Build one full cycle of a waveform into a 16-bit buffer.
 *
 * @param buf   Output buffer of at least 'n' uint16_t values.
 * @param n     Number of samples per cycle (typically NUM_POINTS).
 * @param type  One of WAVE_SINE, WAVE_SQUARE, WAVE_TRIANGLE, WAVE_SAWTOOTH.
 * @param amp   Amplitude as a fraction of full scale (0.0 – 0.5).
 * @param mean  DC offset as a fraction of full scale (0.0 – 1.0).
 *
 * Values are clipped to [0.0, 1.0] and scaled to the 16-bit DAC range.
 * A local clamp ensures that (mean - amp) is never negative for this call,
 * without mutating the caller's 'mean' value.  Safe to call every cycle.
 */
void generate_samples(uint16_t *buf, int n,
                      const char *type, float amp, float mean);

/**
 * valid_wave_type - Return non-zero if 's' matches one of the four
 * WAVE_* string constants.  Used for argument and menu validation.
 */
int valid_wave_type(const char *s);

/**
 * run_waveform - Continuously outputs a waveform on the DAC.
 *
 * @param iobase  Mapped I/O base addresses for the PCI device.
 * @param type    Waveform type string.
 * @param freq    Frequency in Hz.
 * @param amp     Amplitude (0.0 – 0.5).
 * @param mean    DC offset (0.0 – 1.0).
 *
 * This function runs an infinite loop and does not return; it is used
 * by the standalone test binary only.  The integrated program uses
 * waveform_output_thread() in threads.c instead.
 */
void run_waveform(uintptr_t *iobase, const char *type,
                  float freq, float amp, float mean);

#endif /* WAVEFORM_DA_H */