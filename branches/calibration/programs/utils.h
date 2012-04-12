/*
	Copyright 2011, 2012 Barrett Technology <support@barrett.com>

	This file is part of libbarrett.

	This version of libbarrett is free software: you can redistribute it
	and/or modify it under the terms of the GNU General Public License as
	published by the Free Software Foundation, either version 3 of the
	License, or (at your option) any later version.

	This version of libbarrett is distributed in the hope that it will be
	useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License along
	with this version of libbarrett.  If not, see
	<http://www.gnu.org/licenses/>.

	Further, non-binding information about licensing is available at:
	<http://wiki.barrett.com/libbarrett/wiki/LicenseNotes>
*/

/*
 * utils.h
 *
 *  Created on: Dec 16, 2011
 *      Author: dc
 */

#ifndef UTILS_H_
#define UTILS_H_


void manageBackups(const char* file, int numBackups = 5);


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


#endif /* UTILS_H_ */
