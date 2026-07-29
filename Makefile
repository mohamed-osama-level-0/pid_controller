# تحديد المترجم وإعدادات التجميع
CC = gcc
CFLAGS = -Wall -Wextra -std=c11

# اسم الملف التنفيذي النهائي
TARGET = simulate

# الهدف الافتراضي عند كتابة make فقط
all: $(TARGET)

# قاعدة ربط الملفات الكائنية لإنشاء البرنامج النهائي
$(TARGET): simulate.o pid.o motor.o
	$(CC) $(CFLAGS) -o $(TARGET) simulate.o pid.o motor.o

# قواعد تجميع كل ملف .c إلى ملف .o (Object File)
simulate.o: simulate.c pid.h motor.h types.h
	$(CC) $(CFLAGS) -c simulate.c

pid.o: pid.c pid.h types.h
	$(CC) $(CFLAGS) -c pid.c

motor.o: motor.c motor.h types.h
	$(CC) $(CFLAGS) -c motor.c

# أمر لتنظيف بيئة العمل
clean:
	rm -f *.o $(TARGET) output.csv