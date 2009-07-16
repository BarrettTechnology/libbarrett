### Makefile for the "bt-wam-gw" WAM daemon

CC = gcc

#CFLAGS += -std=c89 -pedantic-errors -Wall -Werror -Wno-long-long
# For select() in sys/types.h ...
CFLAGS += -pedantic-errors -Wall -Werror -Wno-long-long

default: bt-wam-gw

# Link against btos for realtime stuff:
CFLAGS += -I/usr/local/include
LDFLAGS += -lbt -lgsl -lgslcblas -lconfig -lcurses -ljson
# Note: we shouldn't need to link against gsl, libconfig!

# OK; Compile stuff:

%.o : %.c
	${CC} ${CFLAGS} -c $?

# OK; Link stuff:

bt-wam-gw : bt-wam-gw.o
	${CC} bt-wam-gw.o ${LDFLAGS} -o bt-wam-gw

clean:
	rm -f *~ *.o bt-wam-gw


