#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>
#include <sys/neutrino.h>
#include <hw/inout.h>

/*
 * MA4830 File I/O Sample
 *
 * Purpose:
 * 1. Record potentiometer (ADC) values into a file
 * 2. Replay recorded values to DAC
 * 3. Use one thread + shared globals + mutex
 *
 * IMPORTANT:
 * Replace ADC_BASE_ADDR and DAC_BASE_ADDR with the actual
 * addresses used by your lab hardware.
 */

/* =========================
   Shared globals for threads
   ========================= */
pthread_mutex_t data_mutex = PTHREAD_MUTEX_INITIALIZER;
unsigned short shared_pot_value = 0;
volatile int stop_signal = 0;

/* =========================
   Hardware addresses
   =========================
   These are PLACEHOLDERS only.
*/
#define ADC_BASE_ADDR  0x280
#define DAC_BASE_ADDR  0x290

/* =========================
   Hardware helper functions
   ========================= */

/* Read one 16-bit value from ADC (potentiometer input) */
static unsigned short read_adc(void)
{
    return in16(ADC_BASE_ADDR);
}

/* Write one 16-bit value to DAC */
static void write_dac(unsigned short value)
{
    out16(DAC_BASE_ADDR, value);
}

/* =========================
   Recording thread function
   =========================
   This thread:
   - reads ADC
   - updates shared_pot_value
   - writes samples into a binary file
*/
void *file_io_record_thread(void *arg)
{
    const char *filename = (const char *)arg;
    FILE *fp;
    unsigned short current_val;

    if (filename == NULL) {
        fprintf(stderr, "Error: filename is NULL\n");
        return NULL;
    }

    fp = fopen(filename, "wb");
    if (fp == NULL) {
        perror("Failed to open file for writing");
        return NULL;
    }

    printf("Record thread started. Writing ADC data to %s\n", filename);

    while (!stop_signal) {
        /* Read current ADC sample */
        current_val = read_adc();

        /* Update shared global so other threads can use latest pot value */
        pthread_mutex_lock(&data_mutex);
        shared_pot_value = current_val;
        pthread_mutex_unlock(&data_mutex);

        /* Save sample to file */
        if (fwrite(&current_val, sizeof(unsigned short), 1, fp) != 1) {
            perror("Failed to write sample to file");
            break;
        }

        /* Flush so data is pushed to disk during demo/testing */
        fflush(fp);

        /* Delay between samples: 100 ms */
        usleep(100000);
    }

    fclose(fp);
    printf("Record thread stopped.\n");
    return NULL;
}

/* =========================
   Playback function
   =========================
   This function:
   - opens recorded binary file
   - reads one sample at a time
   - outputs sample to DAC
*/
int playback_file_to_dac(const char *filename, useconds_t delay_us)
{
    FILE *fp;
    unsigned short value;

    if (filename == NULL) {
        fprintf(stderr, "Error: filename is NULL\n");
        return -1;
    }

    fp = fopen(filename, "rb");
    if (fp == NULL) {
        perror("Failed to open file for reading");
        return -1;
    }

    printf("Playing back %s to DAC...\n", filename);

    while (fread(&value, sizeof(unsigned short), 1, fp) == 1) {
        write_dac(value);
        usleep(delay_us);
    }

    fclose(fp);
    printf("Playback finished.\n");
    return 0;
}

/* =========================
   Main program
   =========================
   Usage:
   ./fileio_single record
   ./fileio_single playback
*/
int main(int argc, char *argv[])
{
    pthread_t record_tid;
    const char *filename = "pot_data.bin";

    /* Need I/O privileges for QNX hardware access */
    if (ThreadCtl(_NTO_TCTL_IO, 0) == -1) {
        perror("Failed to get I/O privileges");
        return EXIT_FAILURE;
    }

    if (argc < 2) {
        printf("Usage:\n");
        printf("  %s record\n", argv[0]);
        printf("  %s playback\n", argv[0]);
        return EXIT_FAILURE;
    }

    /* RECORD MODE */
    if (strcmp(argv[1], "record") == 0) {
        stop_signal = 0;

        if (pthread_create(&record_tid, NULL, file_io_record_thread, (void *)filename) != 0) {
            perror("Failed to create record thread");
            return EXIT_FAILURE;
        }

        printf("Recording potentiometer data...\n");
        printf("Press Enter to stop recording.\n");
        getchar();

        stop_signal = 1;
        pthread_join(record_tid, NULL);

        printf("Recording complete. Data saved in %s\n", filename);
    }

    /* PLAYBACK MODE */
    else if (strcmp(argv[1], "playback") == 0) {
        if (playback_file_to_dac(filename, 100000) != 0) {
            fprintf(stderr, "Playback failed.\n");
            return EXIT_FAILURE;
        }
    }

    else {
        fprintf(stderr, "Invalid mode. Use 'record' or 'playback'.\n");
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}