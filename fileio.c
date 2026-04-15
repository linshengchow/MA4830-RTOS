#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "fileio_da.h"

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

    if (filename == NULL || iobase == NULL) {
        fprintf(stderr, "Error: NULL argument passed to record thread\n");
        return NULL;
    }

    fp = fopen(filename, "wb");
    if (fp == NULL) {
        perror("Failed to open file for writing");
        return NULL;
    }

    printf("Record thread started. Writing ADC data to %s\n", filename);

    while (!stop_signal) {
        /* Trigger ADC conversion and wait for EOC */
        sample = adc_read(iobase);

        /* Update shared value so other threads can read it */
        pthread_mutex_lock(&data_mutex);
        shared_pot_value = sample;
        pthread_mutex_unlock(&data_mutex);

        /* Persist raw 16-bit sample to binary file */
        if (fwrite(&sample, sizeof(unsigned short), 1, fp) != 1) {
            perror("Failed to write sample to file");
            break;
        }

        /* Flush to disk each sample for safety during demos */
        fflush(fp);

        usleep(SAMPLE_DELAY_US);
    }

    fclose(fp);
    printf("Record thread stopped.\n");
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

    fp = fopen(filename, "rb");
    if (fp == NULL) {
        perror("Failed to open file for reading");
        return -1;
    }

    printf("Playing back %s to DAC0...\n", filename);

    while (fread(&value, sizeof(unsigned short), 1, fp) == 1) {
        /*
         * The recorded ADC samples are 16-bit (0x0000–0xFFFF).
         * The DAC is 12-bit (0x000–0xFFF), so shift down by 4
         * to map the full ADC range to the full DAC range.
         */
        dac_write(iobase, value >> 4);
        usleep(delay_us);
    }

    fclose(fp);
    printf("Playback finished.\n");
    return 0;
}

/* ── Entry Point ─────────────────────────────────────────────────────────── */

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