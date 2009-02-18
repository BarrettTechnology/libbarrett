### Makefile for the "surface" WAM program

CC = gcc

CFLAGS += -std=c89 -pedantic-errors -Wall -Werror -Wno-long-long

default: surface

# Link against btos for realtime stuff:
CFLAGS += -I/usr/local/include
LDFLAGS += -lbt -lgsl -lgslcblas -lconfig -lcurses
# Note: we shouldn't need to link against gsl, libconfig!

# OK; Compile stuff:

%.o : %.c
	${CC} ${CFLAGS} -c $?

# OK; Link stuff:

surface : surface.o
	${CC} surface.o ${LDFLAGS} -o surface

clean:
	rm -f *~ *.o surface


