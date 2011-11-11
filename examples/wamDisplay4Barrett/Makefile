default : wamdisplay_linux



.PHONY : sockets
sockets :
	cd sockets && make && make install

.PHONY : wamdisplay_win
wamdisplay_win : sockets
	cd wamdisplay && make IS_WINDOWS=1

.PHONY : wamdisplay_linux
wamdisplay_linux : sockets
	cd wamdisplay && make


.PHONY : clean
clean :
	-cd  sockets && make clean
	-cd  wamdisplay && make clean
	-cd lib && rm *
	cd include && ls *|grep -v wam_Spec | grep -v definitions  |  xargs rm -rf #this line deletes all the files except the wam_Spec* files

	
	


