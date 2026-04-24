#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/neutrino.h>
#include <sys/mman.h>
#include <hw/pci.h>
#include <hw/inout.h>

#include "waveform.h"
#include "fileio.h"

/* ── ADC Register Map (PCI-DAS1602, relative to iobase[]) ───────────────── */
#define ADC_INTERRUPT(b)    ((b)[1] + 0)    /* Badr1 + 0 : interrupt / ADC  */
#define ADC_MUXCHAN(b)      ((b)[1] + 2)    /* Badr1 + 2 : MUX / channel    */
#define ADC_TRIGGER(b)      ((b)[1] + 4)    /* Badr1 + 4 : trigger control  */
#define ADC_AUTOCAL(b)      ((b)[1] + 6)    /* Badr1 + 6 : auto-calibration */
#define ADC_DATA(b)         ((b)[2] + 0)    /* Badr2 + 0 : ADC data         */
#define ADC_FIFOCLR(b)      ((b)[2] + 2)    /* Badr2 + 2 : ADC FIFO clear   */

/* ── PCI Setup ───────────────────────────────────────────────────────────── */

void *pci_setup(uintptr_t *iobase)
{
    struct pci_dev_info info;
    void *hdl;
    int   i, badr[DA_NUM_BARS];

    memset(&info, 0, sizeof(info));

    if (pci_attach(0) < 0) {
        perror("pci_attach");
        exit(EXIT_FAILURE);
    }

    info.VendorId = DA_VENDOR_ID;
    info.DeviceId = DA_DEVICE_ID;

    if ((hdl = pci_attach_device(0, PCI_SHARE | PCI_INIT_ALL, 0, &info)) == 0) {
        perror("pci_attach_device");
        exit(EXIT_FAILURE);
    }

    for (i = 0; i < DA_NUM_BARS; i++) {
        badr[i]   = PCI_IO_ADDR(info.CpuBaseAddress[i]);
        iobase[i] = mmap_device_io(DA_IO_RANGE, badr[i]);
    }

    return hdl;
}

/* ── ADC Primitives ──────────────────────────────────────────────────────── */

void adc_init(uintptr_t *iobase)
{
    out16(ADC_INTERRUPT(iobase), 0x60c0);   /* Clear interrupts              */
    out16(ADC_TRIGGER(iobase),   0x2081);   /* 10 MHz, Burst off, SW trig   */
    out16(ADC_AUTOCAL(iobase),   0x007f);   /* Auto-calibration default      */
    out16(ADC_FIFOCLR(iobase),   0);        /* Clear ADC FIFO                */
    out16(ADC_MUXCHAN(iobase),   0x0D00);   /* SW trig, UP, SE, 5 V, ch 0   */
}

/*
 * adc_read_ch - Trigger a single conversion on the given channel (0 or 1)
 * and return the raw 16-bit result.
 *
 * MUX register encoding for the PCI-DAS1602 in single-ended mode:
 *   Bits [15:8] = 0x0D   (SW trigger, unipolar, single-ended, 5 V range)
 *   Bits  [7:4] = high channel (scan stop)
 *   Bits  [3:0] = low  channel (scan start)
 * For a single channel N, set both nibbles to N: 0x0D00 | (N << 4) | N
 */
unsigned short adc_read_ch(uintptr_t *iobase, int ch)
{
    uint16_t mux = (uint16_t)(0x0D00 | ((ch & 0xF) << 4) | (ch & 0xF));
    out16(ADC_MUXCHAN(iobase), mux);                 /* Select channel       */
    delay(1);                                         /* MUX settle          */
    out16(ADC_DATA(iobase), 0);                       /* Start conversion    */
    while (!(in16(ADC_MUXCHAN(iobase)) & 0x4000));   /* Wait for EOC        */
    return in16(ADC_DATA(iobase));
}

/*
 * adc_read - Convenience wrapper for channel 0 (backwards compatible).
 */
unsigned short adc_read(uintptr_t *iobase)
{
    return adc_read_ch(iobase, 0);
}

/* ── DAC Primitive ───────────────────────────────────────────────────────── */

void dac_write(uintptr_t *iobase, unsigned short value)
{
    out16(DA_CTLREG(iobase),  DA_CTLREG_VAL);
    out16(DA_FIFOCLR(iobase), 0);
    out16(DA_DATA(iobase),    value);
}

/* ── Playback Helper (used by standalone binary) ─────────────────────────── */

int playback_file_to_dac(uintptr_t *iobase, const char *filename,
                         useconds_t delay_us)
{
    FILE          *fp;
    unsigned int   chan_dummy, count_dummy, raw;
    unsigned short value;

    if (filename == NULL || iobase == NULL) {
        fprintf(stderr, "Error: NULL argument passed to playback\n");
        return -1;
    }

    fp = fopen(filename, "r");
    if (fp == NULL) {
        perror("Failed to open file for reading");
        return -1;
    }

    printf("Playing back %s to DAC0...\n", filename);

    while (fscanf(fp, " ADC Chan: %x Data [%d]: %x",
                  &chan_dummy, &count_dummy, &raw) == 3) {
        value = (unsigned short)(raw & 0xFFFF);
        printf("Playback -> DAC: %04x\n", (unsigned int)value);
        dac_write(iobase, value);
        usleep(delay_us);
    }

    fclose(fp);
    printf("Playback finished.\n");
    return 0;
}

/* ══════════════════════════════════════════════════════════════════════════
 *  Standalone Diagnostic Entry Point
 * ──────────────────────────────────────────────────────────────────────────
 *  Built with -DFILEIO_MAIN.  Runs a minimal record-or-playback binary that
 *  exercises the ADC → file and file → DAC paths in isolation, useful for
 *  verifying hardware connectivity before running the full integrated
 *  program.  The integrated version lives in threads.c / main.c.
 * ══════════════════════════════════════════════════════════════════════════ */
#ifdef FILEIO_MAIN

#include <pthread.h>
#include <signal.h>

/* Locally-owned state for the standalone binary only — the integrated
 * program has its own shared state managed in threads.c. */
static volatile int    diag_stop = 0;
static pthread_mutex_t diag_mtx  = PTHREAD_MUTEX_INITIALIZER;
static unsigned short  diag_pot  = 0;

static void diag_sigint(int sig) { (void)sig; diag_stop = 1; }

typedef struct {
    const char *filename;
    uintptr_t  *iobase;
} diag_args_t;

static void *diag_record(void *arg)
{
    diag_args_t    *a = (diag_args_t *)arg;
    FILE           *fp;
    unsigned short  s;
    int             count = 0;

    fp = fopen(a->filename, "w");
    if (!fp) { perror("fopen"); return NULL; }

    printf("Record thread started. Writing ADC data to %s\n", a->filename);

    while (!diag_stop) {
        s = adc_read(a->iobase);

        pthread_mutex_lock(&diag_mtx);
        diag_pot = s;
        pthread_mutex_unlock(&diag_mtx);

        fprintf(fp, "ADC Chan: %02x Data [%3d]: %4x\n",
                0, count, (unsigned int)s);
        printf("ADC Chan: %02x Data [%3d]: %4x\n",
                0, count, (unsigned int)s);
        fflush(fp);
        count++;
        usleep(SAMPLE_DELAY_US);
    }

    fclose(fp);
    printf("Record thread stopped. %d samples saved.\n", count);
    return NULL;
}

int main(int argc, char *argv[])
{
    uintptr_t   iobase[DA_NUM_BARS];
    void       *pci_hdl;
    pthread_t   rec_tid;
    diag_args_t args;
    const char *filename = DEFAULT_FILENAME;

    if (ThreadCtl(_NTO_TCTL_IO, 0) == -1) {
        perror("Failed to get I/O privileges");
        return EXIT_FAILURE;
    }

    if (argc < 2) {
        printf("Usage:\n");
        printf("  %s record\n",   argv[0]);
        printf("  %s playback\n", argv[0]);
        return EXIT_FAILURE;
    }

    pci_hdl = pci_setup(iobase);
    adc_init(iobase);
    signal(SIGINT, diag_sigint);

    if (strcmp(argv[1], "record") == 0) {
        args.filename = filename;
        args.iobase   = iobase;

        if (pthread_create(&rec_tid, NULL, diag_record, &args) != 0) {
            perror("pthread_create");
            pci_detach_device(pci_hdl);
            return EXIT_FAILURE;
        }

        printf("Recording. Press Enter (or Ctrl+C) to stop.\n");
        getchar();
        diag_stop = 1;
        pthread_join(rec_tid, NULL);

        printf("Recording complete. Data saved in %s\n", filename);

    } else if (strcmp(argv[1], "playback") == 0) {
        if (playback_file_to_dac(iobase, filename, SAMPLE_DELAY_US) != 0) {
            fprintf(stderr, "Playback failed.\n");
            pci_detach_device(pci_hdl);
            return EXIT_FAILURE;
        }

    } else {
        fprintf(stderr, "Invalid mode. Use 'record' or 'playback'.\n");
        pci_detach_device(pci_hdl);
        return EXIT_FAILURE;
    }

    pci_detach_device(pci_hdl);
    return EXIT_SUCCESS;
}

#endif /* FILEIO_MAIN */