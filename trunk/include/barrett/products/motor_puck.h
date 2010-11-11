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
#include <barrett/products/abstract/special_puck.h>


namespace barrett {


class MotorPuck : public SpecialPuck {
public:
	MotorPuck(Puck* puck = NULL) :
		SpecialPuck(Puck::PT_Motor), cts(0), rpc(0.0), cpr(0.0), ipnm(0) { setPuck(puck); }
	virtual ~MotorPuck() {}

	virtual void update();

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
			return Puck::encodeBusId(id, PuckGroup::FGRP_MOTOR_POSITION);
		}

		typedef double result_type;
		static int parse(int id, int propId, result_type* result, const unsigned char* data, size_t len);
	};


protected:
	int cts;
	double rpc, cpr;
	int ipnm;
};


}


#endif /* BARRETT_PRODUCTS_MOTOR_PUCK_H_ */
