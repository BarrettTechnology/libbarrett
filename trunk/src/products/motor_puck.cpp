/*
 * motor_puck.cpp
 *
 *  Created on: Nov 2, 2010
 *      Author: dc
 */

#include <stdexcept>

#include <syslog.h>

#include <barrett/bus/abstract/communications_bus.h>
#include <barrett/math/utils.h>
#include <barrett/products/puck.h>

#include <barrett/products/motor_puck.h>


namespace barrett {


MotorPuck::MotorPuck(Puck* puck) :
	cts(0), rpc(0.0), cpr(0.0), ipnm(0)
{
	setPuck(puck);
}

void MotorPuck::setPuck(Puck* puck, bool autoUpdate)
{
	p = puck;
	if (p != NULL  &&  autoUpdate) {
		update();
	}
}

void MotorPuck::update()
{
	cts = getProperty(Puck::CTS);
	rpc = 2*M_PI / cts;
	cpr = cts / (2*M_PI);

	ipnm = getProperty(Puck::IPNM);
}


void MotorPuck::sendPackedTorques(const CommunicationsBus& bus, int groupId, int propId,
		const double* pt, int numTorques)
{
	unsigned char data[8];
	int tmp1, tmp2;

	// Special value-packing compilation: Packs (4) 14-bit values into 8 bytes
	//     0        1        2        3        4        5        6        7
	// ATPPPPPP AAAAAAaa aaaaaaBB BBBBbbbb bbbbCCCC CCcccccc ccDDDDDD dddddddd
	switch (numTorques) {
	case 4:
		tmp1 = floor(math::saturate(pt[3], MAX_PUCK_TORQUE));
		data[7] = static_cast<unsigned char>(tmp1 & 0x00FF);
	case 3:
		tmp2 = floor(math::saturate(pt[2], MAX_PUCK_TORQUE));
		data[6] = static_cast<unsigned char>( ((tmp2 << 6) & 0x00C0) | ((tmp1 >> 8) & 0x003F) );
		data[5] = static_cast<unsigned char>( ( tmp2 >> 2) & 0x00FF );
	case 2:
		tmp1 = floor(math::saturate(pt[1], MAX_PUCK_TORQUE));
		data[4] = static_cast<unsigned char>( ((tmp1 << 4) & 0x00F0) | ((tmp2 >> 10) & 0x000F) );
		data[3] = static_cast<unsigned char>( ( tmp1 >> 4) & 0x00FF );
	case 1:
		tmp2 = floor(math::saturate(pt[0], MAX_PUCK_TORQUE));
		data[2] = static_cast<unsigned char>( ((tmp2 << 2) & 0x00FC) | ((tmp1 >> 12) & 0x0003) );
		data[1] = static_cast<unsigned char>( ( tmp2 >> 6) & 0x00FF );
		break;

	default:
		throw std::logic_error("MotorPuck::sendPackedTorques(): numTorques must be >= 1 and <= PUCKS_PER_TORQUE_GROUP.");
		break;
	}
	data[0] = propId | Puck::SET_MASK;

	bus.send(Puck::nodeId2BusId(groupId), data, (numTorques == 4) ? 8 : (1 + numTorques*2));
}


MotorPuck::PositionParser::result_type MotorPuck::PositionParser::parse(int id, int propId, const unsigned char* data, size_t len) {
	bool err = false;
	if (len != 3 && len != 6) {
		syslog(
				LOG_ERR,
				"%s: expected message length of 3 or 6, got message length of %d",
				__func__, len);
		err = true;
	}

	// TODO(dc): Check with BZ about this formatting condition
//			if ((data[0] & 0xc0) != 0) {
//				syslog(LOG_ERR, "%s: expected the 2 MSBs of the first byte to be 0", __func__);
//				err = true;
//			}

	if (err) {
		throw std::runtime_error("MotorPuck::PackedPositionParser::parse(): Unexpected "
			"message. Check /var/log/syslog for details.");
	}


	int value = 0;
	value |= ((long) data[0] << 16) & 0x003F0000;
	value |= ((long) data[1] << 8) & 0x0000FF00;
	value |= ((long) data[2]) & 0x000000FF;

	if (value & 0x00200000) {  // If negative...
		value |= 0xFFC00000; // sign-extend
	}

	return value;
}


}
