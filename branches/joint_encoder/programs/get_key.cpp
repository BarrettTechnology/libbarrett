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
 * get_key.cpp
 *
 *  Created on: Dec 16, 2011
 *      Author: bz
 *      Author: dc
 */

#include <curses.h>
#include "utils.h"


enum Key getKey() {
	int c1, c2, c3;

	// Get the key from ncurses
	c1 = getch();
	if (c1 == ERR) {
		return K_NOKEY;
	}

	// Get all keyboard characters
	if (32 <= c1 && c1 <= 126) {
		return (enum Key) c1;
	}

	// Get special keys
	switch (c1) {
	case K_TAB:
	case K_ENTER:
	case K_BACKSPACE:
		return (enum Key) c1;
		break;
	case 27:
		// Get extended keyboard chars
		c2 = getch();
		if (c2 == ERR)
			return K_ESCAPE;
		if (c2 != 91)
			return K_UNKNOWN;
		c3 = getch();
		switch (c3) {
		case 65:
			return K_UP;
		case 66:
			return K_DOWN;
		case 67:
			return K_RIGHT;
		case 68:
			return K_LEFT;
		default:
			return K_UNKNOWN;
		}
		break;
	default:
		return K_UNKNOWN;
		break;
	}
}
