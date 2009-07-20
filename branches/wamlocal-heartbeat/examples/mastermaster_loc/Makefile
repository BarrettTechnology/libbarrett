### Makefile for the "mastermaster_loc" WAM example program

CC = gcc

#CFLAGS += -std=c89 -pedantic-errors -Wall -Werror -Wno-long-long
#CFLAGS += -pedantic-errors -Wall -Werror -Wno-long-long

default: mastermaster_loc

# Link against btos for realtime stuff:
CFLAGS += -I/usr/local/include
LDFLAGS += -lbarrett -lgsl -lgslcblas -lconfig -lcurses -ljson
# Note: we shouldn't need to link against gsl, libconfig!

# OK; Compile stuff:

%.o : %.c
	${CC} ${CFLAGS} -c $?

# OK; Link stuff:

mastermaster_loc : mastermaster_loc.o refgen_mastermaster_loc.o
	${CC} mastermaster_loc.o refgen_mastermaster_loc.o ${LDFLAGS} -o mastermaster_loc

clean:
	rm -f *~ *.o mastermaster_loc
