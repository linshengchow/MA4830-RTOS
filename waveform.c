#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <time.h>

#include "waveform.h"

/* ── Waveform Generation & Output ────────────────────────────────────────── */

void run_waveform(uintptr_t *iobase, char *type, float freq, float amp, float mean) {
    uint16_t data[NUM_POINTS];
    int i;
    float val = 0;

    double delay_ns = (1.0 / freq / NUM_POINTS) * 1e9;
    struct timespec interval = {0, (long)delay_ns};

    struct _clockperiod clk;
    clk.nsec = CLK_PERIOD_NS;
    clk.fract = 0;

    /*
     * Unipolar DAC: output range is 0.0 – 1.0 (0 V – 5 V).
     * Ensure mean >= amp so the negative excursion never goes below 0,
     * which would clip the lower half of the wave and produce a half-wave.
     * For a full sine centred at mid-rail, mean = 0.5 and amp <= 0.5.
     */
    if (mean < amp) mean = amp;

    // 1. Prepare Waveform Data
    for (i = 0; i < NUM_POINTS; i++) {
        if (strcmp(type, WAVE_SINE) == 0) {
            val = (sinf(i * (2.0f * M_PI / NUM_POINTS)) * amp) + mean;

        } else if (strcmp(type, WAVE_SQUARE) == 0) {
            val = (i < NUM_POINTS / 2) ? (mean + amp) : (mean - amp);

        } else if (strcmp(type, WAVE_TRIANGLE) == 0) {
            if (i < NUM_POINTS / 2)
                val = mean - amp + (2.0f * amp * (i / (NUM_POINTS / 2.0f)));
            else
                val = mean + amp - (2.0f * amp * ((i - NUM_POINTS / 2.0f) / (NUM_POINTS / 2.0f)));

        } else if (strcmp(type, WAVE_SAWTOOTH) == 0) {
            val = (mean - amp) + (2.0f * amp * ((float)i / (float)NUM_POINTS));
        }

        // Clip to [0.0, 1.0] and scale to 16-bit unsigned range
        if (val > 1.0f) val = 1.0f;
        if (val < 0.0f) val = 0.0f;
        data[i] = (uint16_t)(val * 0xFFFF);
    }

    // 2. Set system clock resolution to CLK_PERIOD_NS for timing accuracy
    ClockPeriod(CLOCK_REALTIME, &clk, NULL, 0);

    // 3. Log output info
    printf("Outputting %s wave at %.2f Hz (amp=%.2f, mean=%.2f)...\n",
           type, freq, amp, mean);

    // 4. Real-time output loop 
    while (1) {
        for (i = 0; i < NUM_POINTS; i++) {
            out16(DA_CTLREG(iobase), DA_CTLREG_VAL);
            out16(DA_FIFOCLR(iobase), 0);
            out16(DA_DATA(iobase),    data[i]);
            nanosleep(&interval, NULL);
        }
    }
}

/* ── Entry Point (only compiled when building waveform as a standalone program) */
#ifdef WAVEFORM_MAIN

int main(int argc, char *argv[]) {
    struct pci_dev_info info;
    void       *hdl;
    uintptr_t   iobase[6];
    int         i, badr[6];

    if (argc < 5) {
        printf("Usage: %s <type> <freq> <amp> <mean>\n\n", argv[0]);
        printf("  type  : sine | square | triangle | sawtooth\n");
        printf("  freq  : frequency in Hz\n");
        printf("  amp   : amplitude 0.0 – 0.5  (e.g. 0.5 for full scale)\n");
        printf("  mean  : DC offset  0.0 – 1.0  (use 0.5 for full sine wave)\n\n");
        printf("Example (full sine at 5 Hz):\n");
        printf("  %s sine 5 0.5 0.5\n", argv[0]);
        return 1;
    }

    // PCI Initialization
    memset(&info, 0, sizeof(info));

    if (pci_attach(0) < 0) {
        perror("pci_attach");
        exit(1);
    }

    info.VendorId = DA_VENDOR_ID;
    info.DeviceId = DA_DEVICE_ID;

    if ((hdl = pci_attach_device(0, PCI_SHARE | PCI_INIT_ALL, 0, &info)) == 0) {
        perror("pci_attach_device");
        exit(1);
    }

    for (i = 0; i < DA_NUM_BARS; i++) {
        badr[i]   = PCI_IO_ADDR(info.CpuBaseAddress[i]);
        iobase[i] = mmap_device_io(DA_IO_RANGE, badr[i]);
    }

    // Acquire I/O privileges
    if (ThreadCtl(_NTO_TCTL_IO, 0) == -1) {
        perror("Thread Control");
        exit(1);
    }

    run_waveform(iobase, argv[1], atof(argv[2]), atof(argv[3]), atof(argv[4]));

    pci_detach_device(hdl);
    return 0;
}

#endif /* WAVEFORM_MAIN */