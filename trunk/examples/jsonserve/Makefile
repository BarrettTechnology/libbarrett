### Makefile for the "jsonserve" WAM program

CC = gcc

#CFLAGS += -std=c89 -pedantic-errors -Wall -Werror -Wno-long-long
# For select() in sys/types.h ...
CFLAGS += -pedantic-errors -Wall -Werror -Wno-long-long

default: jsonserve

# Link against btos for realtime stuff:
CFLAGS += -I/usr/local/include
LDFLAGS += -lbt -lgsl -lgslcblas -lconfig -lcurses -ljson
# Note: we shouldn't need to link against gsl, libconfig!

# OK; Compile stuff:

%.o : %.c
	${CC} ${CFLAGS} -c $?

# OK; Link stuff:

jsonserve : jsonserve.o
	${CC} jsonserve.o ${LDFLAGS} -o jsonserve

clean:
	rm -f *~ *.o jsonserve


