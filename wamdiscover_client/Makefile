# This Makefile makes both windows and linux binaries by default.

default: wamdiscover wamdiscover.exe

wamdiscover: linux-main.c common-linux.o
	gcc -pedantic -Wall -Werror linux-main.c common-linux.o -o wamdiscover

common-linux.o : common.c
	gcc -ansi -pedantic -Wall -Werror -c common.c -o common-linux.o

wamdiscover.exe: windows-main.c windows-resources.o common-windows.o
	i586-mingw32msvc-gcc -DWINDOWS -mwindows windows-main.c windows-resources.o common-windows.o -lws2_32 -o wamdiscover.exe

common-windows.o : common.c
	i586-mingw32msvc-gcc -DWINDOWS -c common.c -o common-windows.o

windows-resources.o: windows-resources.rc
	i586-mingw32msvc-windres windows-resources.rc -o windows-resources.o

clean:
	rm -f *.o wamdiscover wamdiscover.exe
