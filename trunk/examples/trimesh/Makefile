### Makefile for the "trimesh" WAM example program

CC = gcc

#CFLAGS += -std=c89 -pedantic-errors -Wall -Werror -Wno-long-long
#CFLAGS += -pedantic-errors -Wall -Werror -Wno-long-long

default: trimesh

# Link against btos for realtime stuff:
CFLAGS += -I/usr/local/include
LDFLAGS += -lbt -lgsl -lgslcblas -lconfig -lcurses -ljson
# Note: we shouldn't need to link against gsl, libconfig!

# OK; Compile stuff:

%.o : %.c
	${CC} ${CFLAGS} -c $?

# OK; Link stuff:

trimesh : trimesh.o refgen_trimesh.o
	${CC} trimesh.o refgen_trimesh.o ${LDFLAGS} -o trimesh

clean:
	rm -f *~ *.o trimesh
