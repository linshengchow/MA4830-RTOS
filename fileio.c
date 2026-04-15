#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <pthread.h>
#include <sys/neutrino.h>
#include <sys/mman.h>
#include <hw/pci.h>
#include <hw/inout.h>

#include "waveform.h"
#include "fileio.h"

/* ── Record Thread Argument Type ─────────────────────────────────────────── */

typedef struct {
    const char *filename;
    uintptr_t  *iobase;
} record_args_t;

/* ── ADC Register Map (PCI-DAS1602, relative to iobase[]) ───────────────── */
/* Matches DAQ_test.c register definitions exactly.                          */
#define ADC_INTERRUPT(b)    ((b)[1] + 0)    /* Badr1 + 0 : interrupt / ADC  */
#define ADC_MUXCHAN(b)      ((b)[1] + 2)    /* Badr1 + 2 : MUX / channel    */
#define ADC_TRIGGER(b)      ((b)[1] + 4)    /* Badr1 + 4 : trigger control  */
#define ADC_AUTOCAL(b)      ((b)[1] + 6)    /* Badr1 + 6 : auto-calibration */
#define ADC_DATA(b)         ((b)[2] + 0)    /* Badr2 + 0 : ADC data         */
#define ADC_FIFOCLR(b)      ((b)[2] + 2)    /* Badr2 + 2 : ADC FIFO clear   */

/* ── Static Hardware Helpers ─────────────────────────────────────────────── */

/*
 * adc_init - Initialise ADC control registers on the PCI-DAS1602.
 * Mirrors the initialisation block from DAQ_test.c.
 */
void adc_init(uintptr_t *iobase)
{
    out16(ADC_INTERRUPT(iobase), 0x60c0);   /* Clear interrupts              */
    out16(ADC_TRIGGER(iobase),   0x2081);   /* 10 MHz, Burst off, SW trig   */
    out16(ADC_AUTOCAL(iobase),   0x007f);   /* Auto-calibration default      */
    out16(ADC_FIFOCLR(iobase),   0);        /* Clear ADC FIFO                */
    out16(ADC_MUXCHAN(iobase),   0x0D00);   /* SW trig, UP, SE, 5 V, ch 0   */
}

/*
 * adc_read - Trigger a single ADC conversion on channel 0 and return the
 * raw 16-bit result.  Sequence matches DAQ_test.c: set MUX, settle, trigger,
 * poll EOC flag (bit 14 of MUXCHAN), then read.
 */
unsigned short adc_read(uintptr_t *iobase)
{
    out16(ADC_MUXCHAN(iobase), 0x0D00);             /* Ch 0, burst off       */
    delay(1);                                        /* Allow mux to settle   */
    out16(ADC_DATA(iobase), 0);                      /* Start conversion      */
    while (!(in16(ADC_MUXCHAN(iobase)) & 0x4000));  /* Wait for EOC          */
    return in16(ADC_DATA(iobase));
}

/*
 * dac_write - Write one 16-bit value to DAC channel 0 via PCI-mapped iobase.
 * Consistent with waveform.c output sequence.
 */
void dac_write(uintptr_t *iobase, unsigned short value)
{
    out16(DA_CTLREG(iobase), DA_CTLREG_VAL);
    out16(DA_FIFOCLR(iobase), 0);
    out16(DA_DATA(iobase),    value);
}

/* ── Shared Global Definitions ───────────────────────────────────────────── */
pthread_mutex_t data_mutex       = PTHREAD_MUTEX_INITIALIZER;
unsigned short  shared_pot_value = 0;
volatile int    stop_signal      = 0;

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

/* ── Recording Thread ────────────────────────────────────────────────────── */

void *file_io_record_thread(void *arg)
{
    record_args_t *args     = (record_args_t *)arg;
    const char    *filename = args->filename;
    uintptr_t     *iobase   = args->iobase;

    FILE          *fp;
    unsigned short sample;
    int            count = 0;

    if (filename == NULL || iobase == NULL) {
        fprintf(stderr, "Error: NULL argument passed to record thread\n");
        return NULL;
    }

    fp = fopen(filename, "w");
    if (fp == NULL) {
        perror("Failed to open file for writing");
        return NULL;
    }

    printf("Record thread started. Writing ADC data to %s\n", filename);

    while (!stop_signal) {
        /* Trigger ADC conversion on channel 0 and wait for EOC */
        sample = adc_read(iobase);

        /* Update shared value so other threads can read it */
        pthread_mutex_lock(&data_mutex);
        shared_pot_value = sample;
        pthread_mutex_unlock(&data_mutex);

        /* Write in the same human-readable format as DAQ_test.c */
        fprintf(fp, "ADC Chan: %02x Data [%3d]: %4x\n", 0, count, (unsigned int)sample);
        printf(     "ADC Chan: %02x Data [%3d]: %4x\n", 0, count, (unsigned int)sample);

        fflush(fp);
        count++;
        usleep(SAMPLE_DELAY_US);
    }

    fclose(fp);
    printf("Record thread stopped. %d samples saved.\n", count);
    return NULL;
}

/* ── Playback Function ───────────────────────────────────────────────────── */

int playback_file_to_dac(uintptr_t *iobase, const char *filename, useconds_t delay_us)
{
    FILE          *fp;
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

    /*
     * Parse each line written by the record thread:
     *   "ADC Chan: %02x Data [%3d]: %4x\n"
     * The chan and count fields are discarded; only the hex ADC value is used.
     * The DAC on the PCI-DAS1602 is 16-bit, so the raw ADC value is sent
     * directly without any scaling.
     */
    {
        unsigned int chan_dummy, count_dummy, raw;
        while (fscanf(fp, " ADC Chan: %x Data [%d]: %x",
                      &chan_dummy, &count_dummy, &raw) == 3) {
            value = (unsigned short)(raw & 0xFFFF);
            printf("Playback -> DAC: %04x\n", (unsigned int)value);
            dac_write(iobase, value);
            usleep(delay_us);
        }
    }

    fclose(fp);
    printf("Playback finished.\n");
    return 0;
}

/* ── Entry Point (only compiled when building fileio as a standalone program) */
#ifdef FILEIO_MAIN

int main(int argc, char *argv[])
{
    uintptr_t     iobase[DA_NUM_BARS];
    void         *pci_hdl;
    pthread_t     record_tid;
    record_args_t record_args;
    const char   *filename = DEFAULT_FILENAME;

    /* Acquire I/O privileges required for QNX hardware access */
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

    /* Attach to the PCIe-DAS1602 and map I/O base addresses */
    pci_hdl = pci_setup(iobase);

    /* Initialise ADC control registers before any sampling */
    adc_init(iobase);

    /* ── RECORD MODE ── */
    if (strcmp(argv[1], "record") == 0) {
        stop_signal = 0;

        record_args.filename = filename;
        record_args.iobase   = iobase;

        if (pthread_create(&record_tid, NULL, file_io_record_thread, &record_args) != 0) {
            perror("Failed to create record thread");
            pci_detach_device(pci_hdl);
            return EXIT_FAILURE;
        }

        printf("Recording potentiometer data...\n");
        printf("Press Enter to stop recording.\n");
        getchar();

        stop_signal = 1;
        pthread_join(record_tid, NULL);

        printf("Recording complete. Data saved in %s\n", filename);
    }

    /* ── PLAYBACK MODE ── */
    else if (strcmp(argv[1], "playback") == 0) {
        if (playback_file_to_dac(iobase, filename, SAMPLE_DELAY_US) != 0) {
            fprintf(stderr, "Playback failed.\n");
            pci_detach_device(pci_hdl);
            return EXIT_FAILURE;
        }
    }

    else {
        fprintf(stderr, "Invalid mode. Use 'record' or 'playback'.\n");
        pci_detach_device(pci_hdl);
        return EXIT_FAILURE;
    }

    pci_detach_device(pci_hdl);
    return EXIT_SUCCESS;
}

#endif /* FILEIO_MAIN */