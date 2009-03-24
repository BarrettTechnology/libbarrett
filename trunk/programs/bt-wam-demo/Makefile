### Makefile for the "bt-wam-demo" WAM program

CC = gcc

CFLAGS += -std=c89 -pedantic-errors -Wall -Werror -Wno-long-long

default: bt-wam-demo

# Link against btos for realtime stuff:
CFLAGS += -I/usr/local/include
LDFLAGS += -lbt -lgsl -lgslcblas -lconfig -lcurses -ljson
# Note: we shouldn't need to link against gsl, libconfig!

# OK; Compile stuff:

%.o : %.c
	${CC} ${CFLAGS} -c $?

# OK; Link stuff:

bt-wam-demo : bt-wam-demo.o
	${CC} bt-wam-demo.o ${LDFLAGS} -o bt-wam-demo

clean:
	rm -f *~ *.o bt-wam-demo


