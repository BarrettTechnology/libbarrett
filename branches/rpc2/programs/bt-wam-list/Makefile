### Makefile for the "bt-wam-list" WAM program

CC = gcc

CFLAGS += -std=c89 -pedantic-errors -Wall -Werror -Wno-long-long

default: bt-wam-list

# Link against btos for realtime stuff:
CFLAGS += -I/usr/local/include
LDFLAGS += -lbt -lgsl -lgslcblas -lconfig -lcurses -ljson
# Note: we shouldn't need to link against gsl, libconfig!

# OK; Compile stuff:

%.o : %.c
	${CC} ${CFLAGS} -c $?

# OK; Link stuff:

bt-wam-list : bt-wam-list.o
	${CC} bt-wam-list.o ${LDFLAGS} -o bt-wam-list

clean:
	rm -f *~ *.o bt-wam-list


