### Makefile for the "mastermaster" WAM example program

CC = gcc

#CFLAGS += -std=c89 -pedantic-errors -Wall -Werror -Wno-long-long
#CFLAGS += -pedantic-errors -Wall -Werror -Wno-long-long

default: draw

# Link against btos for realtime stuff:
CFLAGS += -I/usr/local/include
LDFLAGS += -lbarrett -lgsl -lgslcblas -lconfig -lcurses -ljson
# Note: we shouldn't need to link against gsl, libconfig!

# OK; Compile stuff:

%.o : %.c
	${CC} ${CFLAGS} -c $?

# OK; Link stuff:

draw : draw.o control_draw.o
	${CC} draw.o control_draw.o ${LDFLAGS} -o draw

clean:
	rm -f *~ *.o draw
