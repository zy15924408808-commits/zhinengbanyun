CC      = gcc
CFLAGS  = -std=c99 -Wall -Wextra -pedantic -Werror
INCS    = -I. -I../Protocol/Inc -I../Algorithm/Inc

SRCS    = test_main.c test_x42s_proto.c test_odometry.c \
          ../Protocol/Src/x42s_proto.c \
          ../Algorithm/Src/odometry.c

TARGET  = run_tests.exe

.PHONY: test clean

test: $(TARGET)
	./$(TARGET)

$(TARGET): $(SRCS) *.h ../Protocol/Inc/*.h ../Algorithm/Inc/*.h
	$(CC) $(CFLAGS) $(INCS) $(SRCS) -lm -o $(TARGET)

clean:
	rm -f $(TARGET)