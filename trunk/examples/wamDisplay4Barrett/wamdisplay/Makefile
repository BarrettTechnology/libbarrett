### Wam Display ###

## sources: ##
OBJS = wamdisplay.o wamComponents.o dhTransform.o ClientController.o CommunicationThread.o GraphicThread.o
INCL_FILES = wamComponents.h dhTransform.h ClientController.h
PATH_DEPS = ../include/
DEPS = $(PATH_DEPS)sockets.h $(PATH_DEPS)wam_Spec.h

## targets: ##
TARG = wamdisplay

## internal paths: ##
PATH_INCS = -I../include
PATH_LIBS = -L../lib
PATH_INSTALL =

## internal libs and includes ##
INT_INCS = $(PATH_INCS)
INT_LIBS = $(PATH_LIBS) -lsockets -lbarrett

## external paths, libs and includes ##
EXT_INCS =

ifdef IS_WINDOWS   #compilation using cygwin libraries
EXT_LIBS = -L/usr/X11R6/lib/ -lglut -lgsl -lgslcblas -lpthread -lGL.dll -lGLU.dll -lbarrett -lconfig
else
EXT_LIBS = -L/usr/X11R6/lib/ -lglut -lgsl -lgslcblas -lpthread -lbarrett -lconfig
endif
##
INCS = $(INT_INCS) $(EXT_INCS)
LIBS = $(INT_LIBS) $(EXT_LIBS)

## commands and flags: ##
CC = g++
CFLAGS = $(INCS) -Wall -O2 -DNDEBUG 
CPPFLAGS = $(INCS) -Wall -O2 -DNDEBUG 
LDFLAGS = $(LIBS)


## rules: ##

default : exec

.PHONY : exec
exec: $(OBJS)
	$(CC) $(OBJS) -o $(TARG) $(LDFLAGS) 

$(OBJS) : $(INCL_FILES) $(DEPS)

.PHONY : clean
clean:
	-rm *.o $(TARG).exe $(TARG) *~

.PHONY : cleanc
cleanc:
	-rm testClient.exe testClient

.PHONY : contr
contr: sockets.o client_handler.o test.o 
	$(CC) -I/usr/local/include -I/usr/include -o testClient sockets.o client_handler.o test.o -L/usr/local/include -L/usr/include -lbarrett -lgsl -lgslcblas -lcurses -ljson -lpthread -lconfig

client_handler.o: client_handler.cpp
	$(CC) -I/usr/local/include -I/usr/include -c client_handler.cpp

sockets.o: ../sockets/sockets.cpp
	$(CC) -I/usr/local/include -I/usr/include -c ../sockets/sockets.cpp 

test.o: test.cpp
	$(CC) -I/usr/local/include -I/usr/include -c test.cpp

##

