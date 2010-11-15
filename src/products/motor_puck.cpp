/*
 * motor_puck.cpp
 *
 *  Created on: Nov 2, 2010
 *      Author: cd
 *      Author: dc
 */

#include <stdexcept>

#include <syslog.h>

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


void MotorPuck::sendPackedTorques(const CommunicationsBus& bus, int groupId, int propId,
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


int MotorPuck::MotorPositionParser::parse(int id, int propId, result_type* result, const unsigned char* data, size_t len) {
	if (len != 3 && len != 6) {
		syslog(LOG_ERR,
				"%s: expected message length of 3 or 6, got message length of %d",
				__func__, len);
		return 1;
	}

	*result = twentyTwoBit2double(data[0], data[1], data[2]);
	return 0;
}
int MotorPuck::SecondaryPositionParser::parse(int id, int propId, result_type* result, const unsigned char* data, size_t len) {
	if (len != 3) {
		syslog(LOG_ERR,
				"%s: expected message length of 3, got message length of %d",
				__func__, len);
		return 1;
	}

	*result = twentyTwoBit2double(data[0], data[1], data[2]);
	return 0;
}
int MotorPuck::CombinedPositionParser::parse(int id, int propId, result_type* result, const unsigned char* data, size_t len) {
	if (len != 6) {
		syslog(LOG_ERR,
				"%s: expected message length of 6, got message length of %d",
				__func__, len);
		return 1;
	}

	result->get<0>() = twentyTwoBit2double(data[0], data[1], data[2]);
	result->get<1>() = twentyTwoBit2double(data[3], data[4], data[5]);
	return 0;
}


double MotorPuck::twentyTwoBit2double(unsigned char msb, unsigned char middle, unsigned char lsb)
{
	int intResult = 0;
	intResult |= ((long) msb << 16) & 0x003F0000;
	intResult |= ((long) middle << 8) & 0x0000FF00;
	intResult |= ((long) lsb) & 0x000000FF;

	if (intResult & 0x00200000) {  // If negative...
		intResult |= 0xFFC00000; // sign-extend
	}

	return intResult;
}


}
