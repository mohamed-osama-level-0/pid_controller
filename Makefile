CC = gcc
CFLAGS = -Wall -std=c11

TARGET = pid_sim

all: $(TARGET)

$(TARGET): simulate.o motor.o pid.o
	$(CC) $(CFLAGS) -o $(TARGET) simulate.o motor.o pid.o

simulate.o: simulate.c motor.h pid.h
	$(CC) $(CFLAGS) -c simulate.c

motor.o: motor.c motor.h
	$(CC) $(CFLAGS) -c motor.c

pid.o: pid.c pid.h
	$(CC) $(CFLAGS) -c pid.c

run: $(TARGET)
	./$(TARGET)

clean:
	rm -f *.o $(TARGET) output.csv

.PHONY: all run clean