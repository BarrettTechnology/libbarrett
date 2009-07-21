### Makefile for the "ex2-gcomp" WAM example program

BIN = ex2-gcomp
OBJ = ex2-gcomp.o
CC = gcc

CFLAGS += -std=c89 -pedantic-errors -Wall -Werror
LDFLAGS += -lbarrett -lgsl -lgslcblas -lconfig -lncurses

${BIN} : ${OBJ}

.PHONY : clean
clean:
	rm -f ${OBJ} ${BIN}
