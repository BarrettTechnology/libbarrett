### mrsocket.a ###

## sources: ##
LIB_OBJS= sockets.o
INCL_FILES = sockets.h


## targets: ##
LIB_TARG = libsockets.a

## internal paths: ##
PATH_INCS = -I../include
PATH_LIBS = -L../lib
PATH_INSTALL = ..

## internal libs and includes ##
INT_INCS = $(PATH_INCS)
INT_LIBS = $(PATH_LIBS)

## external paths, libs and includes ##
EXT_INCS =
EXT_LIBS =
# EXT_INCS += -I/usr/realtime/include
# EXT_LIBS += -L/usr/realtime/lib -L/usr/lib -L./lib -lm -lpthread -llxrt -lntcan

##
INCS = $(INT_INCS) $(EXT_INCS)
LIBS = $(INT_LIBS) $(EXT_LIBS)

## commands and flags: ##
CC = g++
CFLAGS = $(INCS) -Wall 
CPPFLAGS = $(INCS) -Wall -O2 -DNDEBUG -Dlinux
LDFLAGS = ${LIBS}


## rules: ##

default: lib

$(LIB_TARG) : ${LIB_OBJS}
	$(AR) rcs $(LIB_TARG) $(LIB_OBJS)

$(LIB_OBJS) : $(INCL_FILES)

.PHONY : lib
lib: $(LIB_TARG)

.PHONY : slib
slib: ${LIB_OBJS}
	${CC} -g ${LDFLAGS} -shared -o ${SLIB_TARG} ${LIB_OBJS}

.PHONY : clean
clean:
	-rm *.o $(LIB_TARG) 

.PHONY : rmall
rmall:
	-rm $(LIB_TARG) *.o *~ *.bak

.PHONY : install
install: lib
	cp $(LIB_TARG) $(PATH_INSTALL)/lib
	cp $(INCL_FILES) $(PATH_INSTALL)/include
