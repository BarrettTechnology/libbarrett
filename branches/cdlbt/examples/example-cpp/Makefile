### Makefile for the "example-cpp" WAM example program

BIN = example-cpp
OBJ = example-cpp.o
CC = g++

CFLAGS += -pedantic-errors -Wall -Werror
LDFLAGS += -lbarrett -lncurses

${BIN} : ${OBJ}

.PHONY : clean
clean:
	rm -f ${OBJ} ${BIN}
