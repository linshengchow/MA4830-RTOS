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

/* ── Waveform Type Strings ───────────────────────────────────────────────── */
#define WAVE_SINE       "sine"
#define WAVE_SQUARE     "square"
#define WAVE_TRIANGLE   "triangle"
#define WAVE_SAWTOOTH   "sawtooth"

/* ── Function Declarations ───────────────────────────────────────────────── */

/**
 * run_waveform - Continuously outputs a waveform on the DAC.
 *
 * @param iobase  Mapped I/O base addresses for the PCI device (array of 6).
 * @param type    Waveform type string: "sine", "square", "triangle", "sawtooth".
 * @param freq    Frequency in Hz.
 * @param amp     Amplitude (0.0 to 0.5, as a fraction of full scale).
 * @param mean    DC offset / mean level (0.0 to 1.0, as a fraction of full scale).
 *
 * Note: This function runs an infinite loop and does not return.
 */
void run_waveform(uintptr_t *iobase, char *type, float freq, float amp, float mean);

#endif /* WAVEFORM_DA_H */