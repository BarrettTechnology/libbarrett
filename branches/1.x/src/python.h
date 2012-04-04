/*
	Copyright 2012 Barrett Technology <support@barrett.com>

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
 * python.h
 *
 *  Created on: Mar 26, 2012
 *      Author: dc
 */

#ifndef BARRETT_SRC_PYTHON_H_
#define BARRETT_SRC_PYTHON_H_


// These functions do the work of building the python wrappers.
void pythonBusInterface();
void pythonProductsInterface();


class Namespace {};
void makeNamespace(const char* name, void(&buildFunction)());


#endif /* BARRETT_SRC_PYTHON_H_ */
