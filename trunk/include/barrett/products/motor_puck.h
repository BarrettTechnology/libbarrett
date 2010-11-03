/*
 * motor_puck.h
 *
 *  Created on: Oct 13, 2010
 *      Author: cd
 *      Author: dc
 */

#ifndef BARRETT_PRODUCTS_MOTOR_PUCK_H_
#define BARRETT_PRODUCTS_MOTOR_PUCK_H_


#include <barrett/bus/abstract/communications_bus.h>
#include <barrett/math/utils.h>
#include <barrett/products/puck.h>


namespace barrett {


class MotorPuck {
public:
	MotorPuck(Puck* puck = NULL);

	void setPuck(Puck* puck, bool autoUpdate = true);
	Puck* getPuck() const { return p; }
	void update();

	int getProperty(enum Puck::Property prop) const {
		return p->getProperty(prop);
	}
	void setProperty(enum Puck::Property prop, int value) const {
		p->setProperty(prop, value);
	}

	int getId() const { return p->getId(); }
	int getVers() const { return p->getRole(); }
	int getRole() const { return p->getRole(); }

	int getCts() const { return cts; }
	double getRadsPerCount() const { return rpc; }
	double getCountsPerRad() const { return cpr; }
	double counts2rad(int counts) const { return rpc * counts; }
	int rad2counts(double rad) const { return floor(cpr * rad); }

	int getIpnm() const { return ipnm; }
	int nm2i(double torque) const { return floor(math::saturate(torque*ipnm, MAX_PUCK_TORQUE)); }


	static void sendPackedTorques(const CommunicationsBus& bus, int groupId, int propId,
			const double* pt, int numTorques);


	static const size_t PUCKS_PER_TORQUE_GROUP = 4;
	static const int MAX_PUCK_TORQUE = 8191;


	struct PositionParser {
		static int busId(int id, int propId) {
			return Puck::encodeBusId(id, PuckGroup::FGRP_POSITION);
		}

		typedef double result_type;
		static result_type parse(int id, int propId, const unsigned char* data, size_t len);
	};


protected:
	Puck* p;

	int cts;
	double rpc, cpr;
	int ipnm;
};


}


#endif /* BARRETT_PRODUCTS_MOTOR_PUCK_H_ */
