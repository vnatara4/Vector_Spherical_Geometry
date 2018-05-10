CC = gcc
CFLAGS = -c -Wall -march=armv7-a -mfpu=neon -mfloat-abi=hard -mcpu=cortex-a8 -mtune=cortex-a8 -ftree-vectorize -mvectorize-with-neon-quad -O3 -g  -pg -ffast-math 

sg: main.o geometry.o CMAN_coords.o
	$(CC) main.o geometry.o CMAN_coords.o -lrt -lm -g -static -o $@  -pg

geometry_list.s: geometry.c
	$(CC) $(CFLAGS) -Wa,-adhln -g geometry.c -c > geometry_list.s

clean:
	rm -f sg *.o sg *.s
