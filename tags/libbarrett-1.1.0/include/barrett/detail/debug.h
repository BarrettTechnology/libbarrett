/*
	Copyright 2009, 2010 Barrett Technology <support@barrett.com>

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
 * debug.h
 *
 *  Created on: Nov 10, 2009
 *      Author: dc
 */

#ifndef BARRETT_DETAIL_DEBUG_H_
#define BARRETT_DETAIL_DEBUG_H_


#include <iostream>

#define DEBUG_MARK  \
		std::cerr << "DEBUG:" __FILE__ ":" << __LINE__ << std::endl


#endif /* BARRETT_DETAIL_DEBUG_H_ */
