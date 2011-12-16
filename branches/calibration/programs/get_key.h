/*
 * get_key.h
 *
 *  Created on: Dec 16, 2011
 *      Author: dc
 */

#ifndef GET_KEY_H_
#define GET_KEY_H_


enum Key {
	K_UNKNOWN = -2,
	K_NOKEY = -1,
	K_TAB = 9,
	K_ENTER = 10,
	K_ESCAPE = 27,
	K_BACKSPACE = 127,
	K_UP = 256,
	K_DOWN = 257,
	K_LEFT = 258,
	K_RIGHT = 259
};

enum Key getKey();


#endif /* GET_KEY_H_ */
