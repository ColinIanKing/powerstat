CFLAGS += -O2 -Wall

powerstat: powerstat.o
	$(CC) $< -lm -o $@

clean:
	rm powerstat powerstat.o
