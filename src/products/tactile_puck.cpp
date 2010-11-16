/*
 * tactile_puck.cpp
 *
 *  Created on: Nov 12, 2010
 *      Author: dc
 */

#include <stdexcept>

#include <syslog.h>

#include <barrett/bus/abstract/communications_bus.h>
#include <barrett/products/puck.h>
#include <barrett/products/abstract/special_puck.h>
#include <barrett/products/tactile_puck.h>


namespace barrett {


void TactilePuck::setPuck(Puck* puck)
{
	// Call super
	SpecialPuck::setPuck(puck);

	if (p != NULL) {
		bus = &p->getBus();
		id = p->getId();
		propId = p->getPropertyId(Puck::TACT);


		// Check for initialization error
		if (p->getProperty(Puck::TACTID) == -2) {
			syslog(LOG_ERR, "TactilePuck::setPuck(): TACT initialization error for ID = %d (TACTID = -2).", id);
			throw std::runtime_error("TactilePuck::setPuck(): TACT initialization error. Check /var/log/syslog for details.");
		}

		tact = NONE;
		p->setProperty(Puck::TACT, tact);

		tare();
	}
}

void TactilePuck::requestFull()
{
	if (tact == FULL_FORMAT) {
		int ret = Puck::sendGetPropertyRequest(*bus, id, propId);
		if (ret != 0) {
			syslog(LOG_ERR, "%s: Puck::sendGetPropertyRequest() returned error %d.", __func__, ret);
			throw std::runtime_error("TactilePuck::updateFull(): Failed to send request. Check /var/log/syslog for details.");
		}
	} else {
		tact = FULL_FORMAT;
		p->setProperty(Puck::TACT, tact);
	}
}
void TactilePuck::receiveFull(bool realtime)
{
	for (size_t i = 0 ; i < NUM_FULL_MESSAGES; ++i) {
		int ret = Puck::receiveGetPropertyReply<FullTactParser>(*bus, id, propId, &full, true, realtime);
		if (ret != 0) {
			syslog(LOG_ERR, "%s: Puck::receiveGetPropertyReply() returned error %d while receiving FULL TACT reply %d of %d from ID=%d.",
					__func__, ret, i+1, NUM_FULL_MESSAGES, id);
			throw std::runtime_error("TactilePuck::updateFull(): Failed to receive reply. Check /var/log/syslog for details.");
		}
	}
}

//void TactilePuck::updateTop10(bool realtime)
//{
//	int ret;
//
//	if (tact == TOP10_FORMAT) {
//		ret = Puck::sendGetPropertyRequest(*bus, id, propId);
//		if (ret != 0) {
//			syslog(LOG_ERR, "%s: Puck::sendGetPropertyRequest() returned error %d.", __func__, ret);
//			throw std::runtime_error("TactilePuck::updateTop10(): Failed to send request. Check /var/log/syslog for details.");
//		}
//	} else {
//		tact = TOP10_FORMAT;
//		p->setProperty(Puck::TACT, tact);
//	}
//
//	ret = Puck::receiveGetPropertyReply<Top10TactParser>(*bus, id, propId, &top10, true, realtime);
//	if (ret != 0) {
//		syslog(LOG_ERR, "%s: Puck::receiveGetPropertyReply() returned error %d while receiving TOP10 TACT reply from ID=%d.",
//				__func__, ret, id);
//		throw std::runtime_error("TactilePuck::updateTop10(): Failed to receive reply. Check /var/log/syslog for details.");
//	}
//}


int TactilePuck::FullTactParser::parse(int id, int propId, result_type* result, const unsigned char* data, size_t len)
{
	if (len != 8) {
		syslog(LOG_ERR, "%s: expected message length of 8, got message length of %d", __func__, len);
		return 1;
	}

	size_t i = data[0] >> 4;  // sequence number
	if (i > NUM_FULL_MESSAGES - 1) {
		syslog(LOG_ERR, "%s: invalid sequence number: %d", __func__, i);
		return 1;
	}
	i *= NUM_SENSORS_PER_FULL_MESSAGE;  // first cell index

	(*result)[i++] = ( (((int)data[0]&0x000F)<<8) | ((int)data[1]&0x00FF) ) / FULL_SCALE_FACTOR;
	(*result)[i++] = ( (((int)data[2]&0x00FF)<<4) | (((int)data[3]&0x00F0)>>4) ) / FULL_SCALE_FACTOR;
	(*result)[i++] = ( (((int)data[3]&0x000F)<<8) | ((int)data[4]&0x00FF) ) / FULL_SCALE_FACTOR;
	(*result)[i++] = ( (((int)data[5]&0x00FF)<<4) | (((int)data[6]&0x00F0)>>4) ) / FULL_SCALE_FACTOR;
    if (i < NUM_SENSORS) {
    	(*result)[i] = ( (((int)data[6]&0x000F)<<8) | ((int)data[7]&0x00FF) ) / FULL_SCALE_FACTOR;
    }

    return 0;
}


}
