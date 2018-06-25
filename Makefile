all: tsmini2 raw-to-csv

tsmini2: tsmini2.c
	gcc tsmini2.c -o tsmini2 -lpthread

raw-to-csv: raw-to-csv.c

clean:
	-rm tsmini2 raw-to-csv
