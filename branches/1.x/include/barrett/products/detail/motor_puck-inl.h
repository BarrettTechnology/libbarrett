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
 * motor_puck-inl.h
 *
 *  Created on: Feb 1, 2011
 *      Author: dc
 */


#include <barrett/os.h>


namespace barrett {


template<typename ResultType>
int MotorPuck::MotorPositionParser<ResultType>::parse(int id, int propId, result_type* result, const unsigned char* data, size_t len) {
	if (len != 3 && len != 6) {
		logMessage("%s: expected message length of 3 or 6, got message length of %d") % __func__ % len;
		return 1;
	}

	*result = twentyTwoBit2<ResultType>(data[0], data[1], data[2]);
	return 0;
}
template<typename ResultType>
int MotorPuck::SecondaryPositionParser<ResultType>::parse(int id, int propId, result_type* result, const unsigned char* data, size_t len) {
	if (len != 3) {
		logMessage("%s: expected message length of 3, got message length of %d") % __func__ % len;
		return 1;
	}

	*result = twentyTwoBit2<ResultType>(data[0], data[1], data[2]);
	return 0;
}
template<typename ResultType>
int MotorPuck::CombinedPositionParser<ResultType>::parse(int id, int propId, result_type* result, const unsigned char* data, size_t len) {
	if (len == 6) {
		boost::get<0>(*result) = twentyTwoBit2<ResultType>(data[0], data[1], data[2]);
		boost::get<1>(*result) = twentyTwoBit2<ResultType>(data[3], data[4], data[5]);
	} else if (len == 3) {
		boost::get<0>(*result) = twentyTwoBit2<ResultType>(data[0], data[1], data[2]);
		boost::get<1>(*result) = std::numeric_limits<ResultType>::max();
	} else {
		logMessage("%s: expected message length of 3 or 6, got message length of %d") % __func__ % len;
		return 1;
	}

	return 0;
}


template<typename ResultType>
ResultType MotorPuck::twentyTwoBit2(unsigned char msb, unsigned char middle, unsigned char lsb)
{
	int intResult = 0;
	intResult |= ((long) msb << 16) & 0x003F0000;
	intResult |= ((long) middle << 8) & 0x0000FF00;
	intResult |= ((long) lsb) & 0x000000FF;

	if (intResult & 0x00200000) {  // If negative...
		intResult |= ~((int)0x3fffff); // sign-extend
	}

	return intResult;
}


}
