### Makefile for the "cylinder" WAM example program

CC = gcc

CFLAGS += -std=c89 -pedantic-errors -Wall -Werror -Wno-long-long

default: cylinder

# Link against btos for realtime stuff:
CFLAGS += -I/usr/local/include
LDFLAGS += -lbt -lgsl -lgslcblas -lconfig -lcurses
# Note: we shouldn't need to link against gsl, libconfig!

# OK; Compile stuff:

%.o : %.c
	${CC} ${CFLAGS} -c $?

# OK; Link stuff:

cylinder : cylinder.o refgen_cylinder.o
	${CC} cylinder.o refgen_cylinder.o ${LDFLAGS} -o cylinder

clean:
	rm -f *~ *.o cylinder
