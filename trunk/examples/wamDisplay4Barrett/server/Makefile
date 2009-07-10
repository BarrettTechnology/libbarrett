CC = g++

#CFLAGS += -std=c89 -pedantic-errors -Wall -Werror -Wno-long-long
#CFLAGS += -pedantic-errors -Wall -Werror -Wno-long-long

default: Server

# Link against btos for realtime stuff:
CFLAGS += -I/usr/local/include -I/usr/include 
LDFLAGS += -L/usr/local/include -L/usr/include -lbarrett -lgsl -lgslcblas -lcurses -ljson -lpthread -lconfig
# Note: we shouldn't need to link against gsl, libconfig!

# OK; Compile stuff:

%.o : %.cpp
	${CC} ${CFLAGS} -c $?

# OK; Link stuff:

sockets.o: ../sockets/sockets.cpp
	${CC} ${CFLAGS} -c ../sockets/sockets.cpp 

main.o: main.cpp
	${CC} ${CFLAGS} -c main.cpp 

Server : main.o server_handler.o sockets.o server_controller.o server_manager.o
	${CC} ${CFLAGS} -o Server main.o server_handler.o sockets.o server_controller.o server_manager.o ${LDFLAGS}

clean:
	rm -f *~ *.o Server
