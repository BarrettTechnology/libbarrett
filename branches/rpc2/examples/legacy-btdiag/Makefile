### Makefile for the "legacy-btdiag" WAM example program

CC = gcc

# Note: this code uses C++ comments,
# the usleep() function,
# and casts function pointers to object pointers and back.
# Ugh!
#CFLAGS += -std=c89 -pedantic-errors -Wall -Werror -Wno-long-long

default: btdiag

# Link against btos for realtime stuff:
CFLAGS += -I/usr/local/include
LDFLAGS += -lbt -lbtlegacy -lgsl -lgslcblas -lconfig -lcurses -ljson
# Note: we shouldn't need to link against gsl, libconfig!

# OK; Compile stuff:

%.o : %.c
	${CC} ${CFLAGS} -c $?

# OK; Link stuff:

btdiag : btdiag.o
	${CC} btdiag.o ${LDFLAGS} -o btdiag

clean:
	rm -f *~ *.o btdiag
