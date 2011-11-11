CC = g++

#CFLAGS += -std=c89 -pedantic-errors -Wall -Werror -Wno-long-long
#CFLAGS += -pedantic-errors -Wall -Werror -Wno-long-long

default: all

# Link against btos for realtime stuff:
CFLAGS += -I/usr/local/include -I/usr/include 
LDFLAGS += -L/usr/local/include -L/usr/include -lbarrett -lgsl -lgslcblas -lcurses -ljson -lpthread -lconfig 
# Note: we shouldn't need to link against gsl, libconfig!

# OK; Compile stuff:

%.o : %.cpp
	${CC} ${CFLAGS} -c $?

main.o: main.cpp
	${CC} ${CFLAGS} -c main.cpp 

BtWam.o: ../BtWam.cpp
	${CC} ${CFLAGS} -c ../BtWam.cpp 

Posix.o: ../Posix.cpp
	${CC} ${CFLAGS} -c ../Posix.cpp

# OK; Link stuff:

all: main.o BtWam.o Posix.o
	${CC} ${CFLAGS} -o main main.o BtWam.o Posix.o ${LDFLAGS}

clean:
	rm -f *~ *.o main
