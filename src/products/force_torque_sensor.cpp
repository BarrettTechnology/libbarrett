/*
 * force_torque_sensor.cpp
 *
 *  Created on: Nov 8, 2010
 *      Author: dc
 */

#include <stdexcept>

#include <syslog.h>

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
		syslog(LOG_ERR, "%s: Puck::sendGetPropertyRequest() returned error %d.", __func__, ret);
		throw std::runtime_error("ForceTorqueSensor::update(): Failed to send request. Check /var/log/syslog for details.");
	}


	// Receive force message
	ret = Puck::receiveGetPropertyReply<ForceParser>(*bus, id, propId, &cf, true, realtime);
	if (ret != 0) {
		syslog(LOG_ERR, "%s: Puck::receiveGetPropertyReply() returned error %d while receiving FT Force reply from ID=%d.",
				__func__, ret, id);
		throw std::runtime_error("ForceTorqueSensor::update(): Failed to receive reply. Check /var/log/syslog for details.");
	}

	// Receive torque message
	ret = Puck::receiveGetPropertyReply<TorqueParser>(*bus, id, propId, &ct, true, realtime);
	if (ret != 0) {
		syslog(LOG_ERR, "%s: Puck::receiveGetPropertyReply() returned error %d while receiving FT Torque reply from ID=%d.",
				__func__, ret, id);
		throw std::runtime_error("ForceTorqueSensor::update(): Failed to receive reply. Check /var/log/syslog for details.");
	}
}

void ForceTorqueSensor::updateAccel(bool realtime)
{
	int ret;

	// TODO(dc): Fix this once FT sensors have working ROLE/VERS properties.
	int accelPropId = Puck::getPropertyId(Puck::A, Puck::PT_ForceTorque, 0);

	ret = Puck::sendGetPropertyRequest(*bus, id, accelPropId);
	if (ret != 0) {
		syslog(LOG_ERR, "%s: Puck::sendGetPropertyRequest() returned error %d.", __func__, ret);
		throw std::runtime_error("ForceTorqueSensor::updateAccel(): Failed to send request. Check /var/log/syslog for details.");
	}

	ret = Puck::receiveGetPropertyReply<AccelParser>(*bus, id, accelPropId, &ca, true, realtime);
	if (ret != 0) {
		syslog(LOG_ERR, "%s: Puck::receiveGetPropertyReply() returned error %d while receiving FT Accel reply from ID=%d.",
				__func__, ret, id);
		throw std::runtime_error("ForceTorqueSensor::updateAccel(): Failed to receive reply. Check /var/log/syslog for details.");
	}
}

int ForceTorqueSensor::parse(int id, int propId, base_type* result, const unsigned char* data, size_t len, double scaleFactor)
{
	if (len != 6  &&  len != 7) {
		syslog(LOG_ERR, "%s: expected message length of 6, got message length of %d", __func__, len);
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
