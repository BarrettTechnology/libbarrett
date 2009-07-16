### Makefile for the "draw2dq" WAM example program

CC = gcc

#CFLAGS += -std=c89 -pedantic-errors -Wall -Werror -Wno-long-long
#CFLAGS += -pedantic-errors -Wall -Werror -Wno-long-long

default: draw2dq

# Link against btos for realtime stuff:
CFLAGS += -I/usr/local/include
LDFLAGS += -lbarrett -lgsl -lgslcblas -lconfig -lcurses -ljson
# Note: we shouldn't need to link against gsl, libconfig!

# OK; Compile stuff:

%.o : %.c
	${CC} ${CFLAGS} -c $?

# OK; Link stuff:

draw2dq : draw2dq.o control_draw2dq.o
	${CC} draw2dq.o control_draw2dq.o ${LDFLAGS} -o draw2dq

clean:
	rm -f *~ *.o draw2dq
