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
 * force_torque_sensor.cpp
 *
 *  Created on: Nov 8, 2010
 *      Author: dc
 */

#include <stdexcept>

#include <barrett/os.h>
#include <barrett/products/puck.h>
#include <barrett/products/abstract/special_puck.h>
#include <barrett/products/force_torque_sensor.h>


namespace barrett {


void ForceTorqueSensor::setPuck(Puck* puck)
{
	// Call super
	SpecialPuck::setPuck(puck);

	p->wake();
	bus = &p->getBus();
	id = p->getId();

	// TODO(dc): Fix this once FT sensors have working ROLE/VERS properties.
	propId = Puck::getPropertyId(Puck::FT, Puck::PT_ForceTorque, 0);

	tare();
}

void ForceTorqueSensor::update(bool realtime)
{
	int ret;

	ret = Puck::sendGetPropertyRequest(*bus, id, propId);
	if (ret != 0) {
		(logMessage("ForceTorqueSensor::%s(): Failed to send request. "
				"Puck::sendGetPropertyRequest() returned error %d.")
				% __func__ % ret).raise<std::runtime_error>();
	}


	// Receive force message
	ret = Puck::receiveGetPropertyReply<ForceParser>(*bus, id, propId, &cf, true, realtime);
	if (ret != 0) {
		(logMessage("ForceTorqueSensor::%s(): Failed to receive reply. "
				"Puck::receiveGetPropertyReply() returned error %d while receiving FT Force reply from ID=%d.")
				% __func__ % ret % id).raise<std::runtime_error>();
	}

	// Receive torque message
	ret = Puck::receiveGetPropertyReply<TorqueParser>(*bus, id, propId, &ct, true, realtime);
	if (ret != 0) {
		(logMessage("ForceTorqueSensor::%s(): Failed to receive reply. "
				"Puck::receiveGetPropertyReply() returned error %d while receiving FT Torque reply from ID=%d.")
				% __func__ % ret % id).raise<std::runtime_error>();
	}
}

void ForceTorqueSensor::updateAccel(bool realtime)
{
	int ret;

	// TODO(dc): Fix this once FT sensors have working ROLE/VERS properties.
	int accelPropId = Puck::getPropertyId(Puck::A, Puck::PT_ForceTorque, 0);

	ret = Puck::sendGetPropertyRequest(*bus, id, accelPropId);
	if (ret != 0) {
		(logMessage("ForceTorqueSensor::%s(): Failed to send request. "
				"Puck::sendGetPropertyRequest() returned error %d.")
				% __func__ % ret).raise<std::runtime_error>();
	}

	ret = Puck::receiveGetPropertyReply<AccelParser>(*bus, id, accelPropId, &ca, true, realtime);
	if (ret != 0) {
		(logMessage("ForceTorqueSensor::%s(): Failed to receive reply. "
				"Puck::receiveGetPropertyReply() returned error %d while receiving FT Accel reply from ID=%d.")
				% __func__ % ret % id).raise<std::runtime_error>();
	}
}

int ForceTorqueSensor::parse(int id, int propId, base_type* result, const unsigned char* data, size_t len, double scaleFactor)
{
	if (len != 6  &&  len != 7) {
		logMessage("ForceTorqueSensor::%s(): expected message length of 6 or 7, got message length of %d.")
				% __func__ % len;
		return 1;
	}


	(*result)[0] = twoByte2int(data[0], data[1]) / scaleFactor;
	(*result)[1] = twoByte2int(data[2], data[3]) / scaleFactor;
	(*result)[2] = twoByte2int(data[4], data[5]) / scaleFactor;

	return 0;
}

int ForceTorqueSensor::twoByte2int(unsigned char lsb, unsigned char msb)
{
	int res = ((int)msb << 8)  |  lsb;

	if (res & 0x00008000) {
		res |= ~((int)0xffff);
	}

	return res;
}


}
