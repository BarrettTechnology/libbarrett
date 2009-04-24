### Makefile for the "draw2d" WAM example program

CC = gcc

#CFLAGS += -std=c89 -pedantic-errors -Wall -Werror -Wno-long-long
#CFLAGS += -pedantic-errors -Wall -Werror -Wno-long-long

default: draw2d

# Link against btos for realtime stuff:
CFLAGS += -I/usr/local/include
LDFLAGS += -lbarrett -lgsl -lgslcblas -lconfig -lcurses -ljson
# Note: we shouldn't need to link against gsl, libconfig!

# OK; Compile stuff:

%.o : %.c
	${CC} ${CFLAGS} -c $?

# OK; Link stuff:

draw2d : draw2d.o control_draw2d.o
	${CC} draw2d.o control_draw2d.o ${LDFLAGS} -o draw2d

clean:
	rm -f *~ *.o draw2d
