/*
	Copyright 2010, 2011, 2012 Barrett Technology <support@barrett.com>

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
 * motor_puck.cpp
 *
 *  Created on: Nov 2, 2010
 *      Author: cd
 *      Author: dc
 */

#include <stdexcept>
#include <limits>

#include <boost/tuple/tuple.hpp>

#include <barrett/bus/abstract/communications_bus.h>
#include <barrett/math/utils.h>
#include <barrett/products/puck.h>
#include <barrett/products/motor_puck.h>


namespace barrett {


void MotorPuck::setPuck(Puck* puck)
{
	// Call super
	SpecialPuck::setPuck(puck);

	if (p != NULL) {
		cts = p->getProperty(Puck::CTS);
		rpc = 2*M_PI / cts;
		cpr = cts / (2*M_PI);

		ipnm = p->getProperty(Puck::IPNM);
	}
}


void MotorPuck::sendPackedTorques(const bus::CommunicationsBus& bus, int groupId, int propId,
		const double* pt, int numTorques)
{
	unsigned char data[8];
	int tmp0, tmp1;

	if (numTorques < 0  ||  numTorques > 4) {
		throw std::logic_error("MotorPuck::sendPackedTorques(): numTorques must be >= 0 and <= PUCKS_PER_TORQUE_GROUP.");
		return;
	}

	// Special value-packing compilation: Packs (4) 14-bit values into 8 bytes
	//     0        1        2        3        4        5        6        7
	// ATPPPPPP AAAAAAaa aaaaaaBB BBBBbbbb bbbbCCCC CCcccccc ccDDDDDD dddddddd
	data[0] = propId | Puck::SET_MASK;

	tmp0 = (numTorques < 1) ? 0 : floor(math::saturate(pt[0], MAX_PUCK_TORQUE));
	data[1] = static_cast<unsigned char>( ( tmp0 >> 6) & 0x00FF );

	tmp1 = (numTorques < 2) ? 0 : floor(math::saturate(pt[1], MAX_PUCK_TORQUE));
	data[2] = static_cast<unsigned char>( ((tmp0 << 2) & 0x00FC) | ((tmp1 >> 12) & 0x0003) );
	data[3] = static_cast<unsigned char>( ( tmp1 >> 4) & 0x00FF );

	tmp0 = (numTorques < 3) ? 0 : floor(math::saturate(pt[2], MAX_PUCK_TORQUE));
	data[4] = static_cast<unsigned char>( ((tmp1 << 4) & 0x00F0) | ((tmp0 >> 10) & 0x000F) );
	data[5] = static_cast<unsigned char>( ( tmp0 >> 2) & 0x00FF );

	tmp1 = (numTorques < 4) ? 0 : floor(math::saturate(pt[3], MAX_PUCK_TORQUE));
	data[6] = static_cast<unsigned char>( ((tmp0 << 6) & 0x00C0) | ((tmp1 >> 8) & 0x003F) );
	data[7] = static_cast<unsigned char>(tmp1 & 0x00FF);


	bus.send(Puck::nodeId2BusId(groupId), data, 8);
}


}
