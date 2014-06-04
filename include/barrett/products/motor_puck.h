/**
 *	Copyright 2009-2014 Barrett Technology <support@barrett.com>
 *
 *	This file is part of libbarrett.
 *
 *	This version of libbarrett is free software: you can redistribute it
 *	and/or modify it under the terms of the GNU General Public License as
 *	published by the Free Software Foundation, either version 3 of the
 *	License, or (at your option) any later version.
 *
 *	This version of libbarrett is distributed in the hope that it will be
 *	useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License along
 *	with this version of libbarrett.  If not, see
 *	<http://www.gnu.org/licenses/>.
 *
 *
 *	Barrett Technology Inc.
 *	73 Chapel Street
 *	Newton, MA 02458
 */
/**
 * @file motor_puck.h
 * @date 10/13/2010
 * @author Chris Dellin
 * @author Dan Cody
 *  
 */

#ifndef BARRETT_PRODUCTS_MOTOR_PUCK_H_
#define BARRETT_PRODUCTS_MOTOR_PUCK_H_


#include <boost/tuple/tuple.hpp>

#include <barrett/bus/abstract/communications_bus.h>
#include <barrett/math/utils.h>
#include <barrett/products/puck.h>
#include <barrett/products/abstract/special_puck.h>


namespace barrett {


class MotorPuck : public SpecialPuck {
public:
	enum MotorMode {
		MODE_IDLE,
		MODE_DUTY,
		MODE_TORQUE,
		MODE_PID,
		MODE_VELOCITY,
		MODE_TRAPEZOIDAL
	};


	MotorPuck(Puck* puck = NULL) :
		SpecialPuck(Puck::PT_Motor), cts(0), rpc(0.0), cpr(0.0), ipnm(0) { setPuck(puck); }
	~MotorPuck() {}

	void setPuck(Puck* puck);

	int getCts() const { return cts; }
	double getRadsPerCount() const { return rpc; }
	double getCountsPerRad() const { return cpr; }
	double counts2rad(int counts) const { return rpc * counts; }
	int rad2counts(double rad) const { return floor(cpr * rad); }

	int getIpnm() const { return ipnm; }
	int nm2i(double torque) const { return floor(math::saturate(torque*ipnm, MAX_PUCK_TORQUE)); }

	bool foundIndexPulse() const {
		return (p->getProperty(Puck::ECMIN) != 0)  ||  (p->getProperty(Puck::ECMAX) != 0);
	}


	static void sendPackedTorques(const bus::CommunicationsBus& bus, int groupId, int propId,
			const double* pt, int numTorques);


	static const size_t PUCKS_PER_TORQUE_GROUP = 4;
	static const int MAX_PUCK_TORQUE = 8191;


	template<typename ResultType>
	struct MotorPositionParser {
		static int busId(int id, int propId) {
			return Puck::encodeBusId(id, PuckGroup::FGRP_MOTOR_POSITION);
		}

		typedef ResultType result_type;
		static int parse(int id, int propId, result_type* result, const unsigned char* data, size_t len);
	};
	template<typename ResultType>
	struct SecondaryPositionParser {
		static int busId(int id, int propId) {
			return Puck::encodeBusId(id, PuckGroup::FGRP_SECONDARY_POSITION);
		}

		typedef ResultType result_type;
		static int parse(int id, int propId, result_type* result, const unsigned char* data, size_t len);
	};
	template<typename ResultType>
	struct CombinedPositionParser {
		static int busId(int id, int propId) {
			return MotorPositionParser<ResultType>::busId(id, propId);
		}

		typedef boost::tuple<ResultType,ResultType> result_type;
		static int parse(int id, int propId, result_type* result, const unsigned char* data, size_t len);
	};


protected:
	int cts;
	double rpc, cpr;
	int ipnm;

private:
	template<typename ResultType>
	static ResultType twentyTwoBit2(unsigned char msb, unsigned char middle, unsigned char lsb);
};


}


// include template definitions
#include <barrett/products/detail/motor_puck-inl.h>


#endif /* BARRETT_PRODUCTS_MOTOR_PUCK_H_ */
