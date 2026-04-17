CC     = cc
CFLAGS = -Wall
LIBS   = -lm

# Build all programs
all: main fileio waveform

# ── Main program : full integration ──────────────────────────────────────
# Links main.c with fileio.c (adc_read, pci_setup, shared state)
# and waveform.c (run_waveform), plus math and pthread libraries.
main: main.c fileio.c waveform.c fileio.h waveform.h
	$(CC) $(CFLAGS) -o main main.c fileio.c waveform.c $(LIBS)

# ── File I/O program (standalone) ────────────────────────────────────────
# FILEIO_MAIN enables the main() in fileio.c
fileio: fileio.c waveform.c fileio.h waveform.h
	$(CC) $(CFLAGS) -DFILEIO_MAIN -o fileio fileio.c waveform.c $(LIBS)

# ── Waveform generator (standalone) ──────────────────────────────────────
# WAVEFORM_MAIN enables the main() in waveform.c
waveform: waveform.c waveform.h
	$(CC) $(CFLAGS) -DWAVEFORM_MAIN -o waveform waveform.c -lm

clean:
	rm -f main fileio waveform
