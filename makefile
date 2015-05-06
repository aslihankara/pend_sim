OPTIONS=-Wall -g #-O3
CC=gcc

all: pendulum.exe average.exe show_states.exe

show_states.exe: show_states.c
	$(CC) $(OPTIONS) show_states.c -o show_states.exe
average.exe: process_scores.c
	$(CC) $(OPTIONS) -O3 process_scores.c -o average.exe

pendulum.exe: main.o controller.o
	$(CC) $(OPTIONS) main.o controller.o -lm -o pendulum.exe

controller.o: controller.c controller.h 
	$(CC) $(OPTIONS) -c controller.c

main.o: main.c
	$(CC) $(OPTIONS) -c main.c 

clean:
	rm -f *.o *.exe

