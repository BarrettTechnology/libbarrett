Wam Display code was built for linux os, however its compilation is possible in 
windows using cygwin.

INTALATION NOTES:
Make sure that during the cygwin instalation all the development packages and the
gsl, freegut and mesa libraries are selected. Make sure that the Mesa libraries 
version is at least 7.2otherwise install a new version.

To install Mesa a new mesa library (if needed!!) follow the linux instructions
(doing ./configure ;make;make install).
The make install probably will be interrupted by an error. If that happens make 
sure that the header files in $MESA_DIR/include/GL/ are copied to /usr/include/GL/
and the library files in $MESA_DIR/lib are copied to /usr/X11R6/lib.

APPLICATION COMPILATION:
To compile on linux just type 'make' in the konsole.
For windows compilation type 'make wamdisplay_win' in a cygwin terminal.
The executable file will be created in the wamdisplay directory.

Enjoy and have fun!!! :)

*********************************************************************************
Edited by vw. 7/29/2009
NOTE:	Make sure global variable in wamdisplay/wamdisplay.cpp (line 71) is
	correct (CAN vs ethernet connection)
*********************************************************************************
If WAM is connected through external CAN (CAN 1):
1. launch executable server/Server
2. launch wamdisplay/wamdisplay

If WAM is connected through ethernet (CAN 0):
1. ssh into internal WAM PC and run bt-wam-gw
2. launch wamdisplay/wamdisplay






