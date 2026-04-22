#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <time.h>

#include "waveform.h"

/* ── Sample Generation (single source of truth) ──────────────────────────── */

/*
 * generate_samples - Build one cycle of a waveform into a 16-bit buffer.
 *
 * The DAC is configured for a unipolar 0–5 V output, so the four formulas
 * below all produce a value in the range [mean - amp, mean + amp].  A
 * local clamp (mean_eff = max(mean, amp)) prevents the negative excursion
 * from going below zero WITHOUT mutating the caller's 'mean' — this was
 * a subtle bug in the previous implementation where 'mean' was clamped in
 * place inside an infinite loop, so a later amplitude decrease could not
 * restore the original centre.
 *
 * A final [0.0, 1.0] clamp protects against any rounding or parameter edge
 * cases before scaling to the 16-bit DAC range.
 */
void generate_samples(uint16_t *buf, int n,
                      const char *type, float amp, float mean)
{
    float mean_eff = (mean < amp) ? amp : mean;
    int   i;
    float val = 0.0f;

    for (i = 0; i < n; i++) {
        if (strcmp(type, WAVE_SINE) == 0) {
            val = sinf(i * (2.0f * (float)M_PI / n)) * amp + mean_eff;

        } else if (strcmp(type, WAVE_SQUARE) == 0) {
            val = (i < n / 2) ? (mean_eff + amp) : (mean_eff - amp);

        } else if (strcmp(type, WAVE_TRIANGLE) == 0) {
            if (i < n / 2)
                val = mean_eff - amp + (2.0f * amp * (i / (n / 2.0f)));
            else
                val = mean_eff + amp - (2.0f * amp *
                      ((i - n / 2.0f) / (n / 2.0f)));

        } else if (strcmp(type, WAVE_SAWTOOTH) == 0) {
            val = (mean_eff - amp) + (2.0f * amp * ((float)i / (float)n));
        }

        /* Final safety clamp before 16-bit scaling */
        if (val > 1.0f) val = 1.0f;
        if (val < 0.0f) val = 0.0f;
        buf[i] = (uint16_t)(val * 0xFFFF);
    }
}

/* ── Type Validation ─────────────────────────────────────────────────────── */

int valid_wave_type(const char *s)
{
    if (s == NULL) return 0;
    return strcmp(s, WAVE_SINE)     == 0 ||
           strcmp(s, WAVE_SQUARE)   == 0 ||
           strcmp(s, WAVE_TRIANGLE) == 0 ||
           strcmp(s, WAVE_SAWTOOTH) == 0;
}

/* ── Standalone Waveform Output ──────────────────────────────────────────── */

/*
 * run_waveform - Thin wrapper used by the standalone 'waveform' binary
 * built with -DWAVEFORM_MAIN.  It now delegates sample generation to
 * generate_samples() so the four formulas live in exactly one place.
 */
void run_waveform(uintptr_t *iobase, const char *type,
                  float freq, float amp, float mean)
{
    uint16_t        data[NUM_POINTS];
    int             i;
    long            delay_ns;
    struct timespec interval, rem;
    struct _clockperiod clk;

    delay_ns         = (long)((1.0 / freq / NUM_POINTS) * 1e9);
    interval.tv_sec  = delay_ns / 1000000000L;
    interval.tv_nsec = delay_ns % 1000000000L;

    clk.nsec  = CLK_PERIOD_NS;
    clk.fract = 0;
    ClockPeriod(CLOCK_REALTIME, &clk, NULL, 0);

    generate_samples(data, NUM_POINTS, type, amp, mean);

    printf("Outputting %s wave at %.2f Hz (amp=%.2f, mean=%.2f)...\n",
           type, freq, amp, mean);

    while (1) {
        for (i = 0; i < NUM_POINTS; i++) {
            out16(DA_CTLREG(iobase), DA_CTLREG_VAL);
            out16(DA_FIFOCLR(iobase), 0);
            out16(DA_DATA(iobase),    data[i]);

            /* EINTR-safe sleep: restart with remaining time if interrupted */
            rem = interval;
            while (nanosleep(&rem, &rem) == -1 && errno == EINTR) { }
        }
    }
}

/* ── Entry Point for Standalone Binary ───────────────────────────────────── */
#ifdef WAVEFORM_MAIN

#include <hw/pci.h>
#include <sys/mman.h>
#include <sys/neutrino.h>

int main(int argc, char *argv[])
{
    struct pci_dev_info info;
    void       *hdl;
    uintptr_t   iobase[DA_NUM_BARS];
    int         i, badr[DA_NUM_BARS];

    if (argc < 5) {
        printf("Usage: %s <type> <freq> <amp> <mean>\n\n", argv[0]);
        printf("  type  : sine | square | triangle | sawtooth\n");
        printf("  freq  : frequency in Hz\n");
        printf("  amp   : amplitude 0.0 – 0.5\n");
        printf("  mean  : DC offset  0.0 – 1.0  (use 0.5 for full sine)\n\n");
        printf("Example:  %s sine 5 0.5 0.5\n", argv[0]);
        return 1;
    }

    if (!valid_wave_type(argv[1])) {
        fprintf(stderr, "Error: type '%s' invalid. "
                "Use: sine | square | triangle | sawtooth\n", argv[1]);
        return 1;
    }

    memset(&info, 0, sizeof(info));
    if (pci_attach(0) < 0) { perror("pci_attach"); exit(1); }

    info.VendorId = DA_VENDOR_ID;
    info.DeviceId = DA_DEVICE_ID;

    if ((hdl = pci_attach_device(0, PCI_SHARE | PCI_INIT_ALL, 0, &info)) == 0) {
        perror("pci_attach_device"); exit(1);
    }

    for (i = 0; i < DA_NUM_BARS; i++) {
        badr[i]   = PCI_IO_ADDR(info.CpuBaseAddress[i]);
        iobase[i] = mmap_device_io(DA_IO_RANGE, badr[i]);
    }

    if (ThreadCtl(_NTO_TCTL_IO, 0) == -1) {
        perror("Thread Control"); exit(1);
    }

    run_waveform(iobase, argv[1],
                 atof(argv[2]), atof(argv[3]), atof(argv[4]));

    pci_detach_device(hdl);
    return 0;
}

#endif /* WAVEFORM_MAIN */