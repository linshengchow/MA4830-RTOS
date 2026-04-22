CC     = cc
CFLAGS = -Wall
# LIBS   = -lm -lsocket
LIBS   = -lm 

# Build all programs
all: main fileio waveform

# ── Main program : full integration ──────────────────────────────────────
# Links main.c with threads.c, fileio.c and waveform.c.
# pthread is required for the five worker threads.
main: main.c threads.c fileio.c waveform.c waveform.h fileio.h threads.h
	$(CC) $(CFLAGS) -o main main.c threads.c fileio.c waveform.c $(LIBS)

# ── File I/O program (standalone) ────────────────────────────────────────
# FILEIO_MAIN enables the diagnostic main() in fileio.c.
# Links fileio.c with waveform.c only (no threads.c — it has its own
# minimal recording thread for isolated hardware testing).
fileio: fileio.c waveform.c fileio.h waveform.h
	$(CC) $(CFLAGS) -DFILEIO_MAIN -o fileio fileio.c waveform.c $(LIBS)

# ── Waveform generator (standalone) ──────────────────────────────────────
# WAVEFORM_MAIN enables the main() in waveform.c.
waveform: waveform.c waveform.h
	$(CC) $(CFLAGS) -DWAVEFORM_MAIN -o waveform waveform.c -lm

clean:
	rm -f main fileio waveform