### Makefile for the "bt-bus-util" program

CC = gcc

CFLAGS += -std=c89 -pedantic-errors -Wall -Werror -Wno-long-long

default: bt-bus-util

# Link against btos for realtime stuff:
CFLAGS += -I/usr/local/include
LDFLAGS += -lbt -lgsl -lgslcblas -lconfig -lcurses -ljson
# Note: we shouldn't need to link against gsl, libconfig!

# OK; Compile stuff:

%.o : %.c
	${CC} ${CFLAGS} -c $?

# OK; Link stuff:

bt-bus-util : bt-bus-util.o
	${CC} bt-bus-util.o ${LDFLAGS} -o bt-bus-util

clean:
	rm -f *~ *.o bt-bus-util


