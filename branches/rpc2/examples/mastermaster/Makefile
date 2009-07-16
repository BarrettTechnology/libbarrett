### Makefile for the "mastermaster" WAM example program

CC = gcc

#CFLAGS += -std=c89 -pedantic-errors -Wall -Werror -Wno-long-long
#CFLAGS += -pedantic-errors -Wall -Werror -Wno-long-long

default: mastermaster

# Link against btos for realtime stuff:
CFLAGS += -I/usr/local/include
LDFLAGS += -lbarrett -lgsl -lgslcblas -lconfig -lcurses -ljson
# Note: we shouldn't need to link against gsl, libconfig!

# OK; Compile stuff:

%.o : %.c
	${CC} ${CFLAGS} -c $?

# OK; Link stuff:

mastermaster : mastermaster.o refgen_mastermaster.o
	${CC} mastermaster.o refgen_mastermaster.o ${LDFLAGS} -o mastermaster

clean:
	rm -f *~ *.o mastermaster
