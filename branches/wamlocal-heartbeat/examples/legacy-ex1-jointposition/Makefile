### Makefile for the "legacy-ex1-jointposition" WAM example program

CC = gcc

# Note: this code uses C++ comments,
# the usleep() function,
# and casts function pointers to object pointers and back.
# Ugh!
#CFLAGS += -std=c89 -pedantic-errors -Wall -Werror -Wno-long-long

default: ex1

# Link against btos for realtime stuff:
CFLAGS += -I/usr/local/include
LDFLAGS += -lbt -lbtlegacy -lgsl -lgslcblas -lconfig -lcurses -ljson
# Note: we shouldn't need to link against gsl, libconfig!

# OK; Compile stuff:

%.o : %.c
	${CC} ${CFLAGS} -c $?

# OK; Link stuff:

ex1 : main.o
	${CC} main.o ${LDFLAGS} -o ex1

clean:
	rm -f *~ *.o ex1
