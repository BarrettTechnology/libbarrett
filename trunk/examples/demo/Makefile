### Makefile for the "demo" WAM program

CC = gcc

CFLAGS += -std=c89 -pedantic-errors -Wall -Werror -Wno-long-long

default: demo

# Link against btos for realtime stuff:
CFLAGS += -I/usr/local/include
LDFLAGS += -lbt -lgsl -lgslcblas -lconfig -lcurses
# Note: we shouldn't need to link against gsl, libconfig!

# OK; Compile stuff:

%.o : %.c
	${CC} ${CFLAGS} -c $?

# OK; Link stuff:

demo : demo.o
	${CC} demo.o ${LDFLAGS} -o demo

clean:
	rm *~ *.o demo


