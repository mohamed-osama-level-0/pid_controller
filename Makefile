# تحديد المترجم وإعدادات التجميع
CC = gcc
CFLAGS = -Wall -Wextra -std=c11


TARGET = simulate


all: $(TARGET)


$(TARGET): simulate.o pid.o motor.o
	$(CC) $(CFLAGS) -o $(TARGET) simulate.o pid.o motor.o


simulate.o: simulate.c pid.h motor.h types.h
	$(CC) $(CFLAGS) -c simulate.c

pid.o: pid.c pid.h types.h
	$(CC) $(CFLAGS) -c pid.c

motor.o: motor.c motor.h types.h
	$(CC) $(CFLAGS) -c motor.c


clean:
	rm -f *.o $(TARGET) output.csv