#ifndef WAMDISPLAY_GLOBALS_H
#define WAMDISPLAY_GLOBALS_H

struct shared_values{
  double angle[7];
  int finish;
  //pthread_mutex_t mutex1;
};

struct arguments{
	int argc0;
	char **argv0;
};

/* all variables here are defined in wamdisplay.cpp */

#endif //WAMDISPLAY_GLOBALS_H
