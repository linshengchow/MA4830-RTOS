#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <hw/pci.h>
#include <hw/inout.h>
#include <sys/neutrino.h>
#include <sys/mman.h>
#include <math.h>
#include <time.h>

#define DA_CTLREG       iobase[1] + 8
#define DA_Data         iobase[4] + 0
#define DA_FIFOCLR      iobase[4] + 2
#define NUM_POINTS      20

void run_waveform(uintptr_t *iobase, char *type, float freq, float amp, float mean) {
    uint16_t data[NUM_POINTS];
    int i;
    float val = 0;
        double delay_ns = (1.0 / freq / NUM_POINTS) * 1e9;
    struct timespec interval = {0, (long)delay_ns};
    struct _clockperiod clk;
    clk.nsec = 10000; clk.fract = 0;
    
    
    // 1. Prepare Waveform Data
    for(i = 0; i < NUM_POINTS; i++) {
        

        if (strcmp(type, "sine") == 0) {
            val = (sinf(i * (2.0 * M_PI / NUM_POINTS)) * amp) + mean;
        } 
        else if (strcmp(type, "square") == 0) {
            val = (i < NUM_POINTS / 2) ? (mean + amp) : (mean - amp);
        } 
        else if (strcmp(type, "triangle") == 0) {
            if (i < NUM_POINTS / 2)
                val = mean - amp + (2.0 * amp * (i / (NUM_POINTS / 2.0)));
            else
                val = mean + amp - (2.0 * amp * ((i - NUM_POINTS / 2.0) / (NUM_POINTS / 2.0)));
        }
        else if (strcmp(type, "sawtooth") == 0){
        	val = (mean -amp) + (2.0*amp*((float)i / (float)NUM_POINTS));
        }

        // Clip and Scale to 16-bit (0x0000 - 0xFFFF)
        if (val > 1.0) val = 1.0; if (val < 0.0) val = 0.0;
        data[i] = (uint16_t)(val * 0xFFFF);
    }

    // 2. Set System Clock Resolution to 10 microseconds for accuracy

    ClockPeriod(CLOCK_REALTIME, &clk, NULL, 0);

    // 3. Calculate Delay

    printf("Outputting %s wave at %f Hz...\n", type, freq);

    // 4. Real-time Output Loop
    while(1) {
        for(i = 0; i < NUM_POINTS; i++) {
            out16(DA_CTLREG, 0x0a23); // Channel 0
            out16(DA_FIFOCLR, 0);
            out16(DA_Data, data[i]);
            nanosleep(&interval, NULL);
        }
    }
}

int main(int argc, char *argv[]) {
    struct pci_dev_info info;
    void *hdl;
    uintptr_t iobase[6];
    int i, badr[6];
    if (argc < 5) {
        printf("Usage: %s <type: sine|square|triangle> <freq> <amp 0-0.5> <mean 0-1>\n", argv[0]);
        return 1;
    }



    // PCI Initialization
    memset(&info, 0, sizeof(info));
    if(pci_attach(0) < 0) { perror("pci_attach"); exit(1); }
    info.VendorId = 0x1307;
    info.DeviceId = 0x01;

    if ((hdl = pci_attach_device(0, PCI_SHARE|PCI_INIT_ALL, 0, &info)) == 0) {
        perror("pci_attach_device"); exit(1);
    }

    for(i = 0; i < 5; i++) {
        badr[i] = PCI_IO_ADDR(info.CpuBaseAddress[i]);
        iobase[i] = mmap_device_io(0x0f, badr[i]);
    }

    // Acquire I/O privileges
    if(ThreadCtl(_NTO_TCTL_IO, 0) == -1) { perror("Thread Control"); exit(1); }

    run_waveform(iobase, argv[1], atof(argv[2]), atof(argv[3]), atof(argv[4]));

    pci_detach_device(hdl);
    return 0;
}