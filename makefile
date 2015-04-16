OPTIONS=-Wall -g
CC=gcc

all: pendulum.exe 


pendulum.exe: main.o controller.o
	$(CC) $(OPTIONS) main.o controller.o -lm -o pendulum.exe

controller.o: controller.c controller.h 
	$(CC) $(OPTIONS) -c controller.c

main.o: main.c
	$(CC) $(OPTIONS) -c main.c 

clean:
	rm -f *.o *.exe

