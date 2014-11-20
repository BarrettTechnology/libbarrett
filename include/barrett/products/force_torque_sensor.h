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
 * @file force_torque_sensor.h
 * @date 11/08/2010
 * @author Dan Cody
 * 
 */

#ifndef BARRETT_PRODUCTS_FT_SENSOR_H_
#define BARRETT_PRODUCTS_FT_SENSOR_H_


#include <Eigen/Core>

#include <barrett/detail/ca_macro.h>
#include <barrett/units.h>
#include <barrett/bus/abstract/communications_bus.h>
#include <barrett/products/puck.h>
#include <barrett/products/abstract/special_puck.h>


namespace barrett {


class ForceTorqueSensor : public SpecialPuck {
public:
	BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;

	/** ForceTorqueSensor Constructor */
	ForceTorqueSensor(Puck* puck = NULL) : SpecialPuck(/* TODO(dc): Puck::PT_ForceTorque */), bus(NULL) { setPuck(puck); }
	/** ForceTorqueSensor Destructor */
	~ForceTorqueSensor() {}
	
	/** setPuck Method changes puck properties to be ForceTorqueSensor settings  */
	void setPuck(Puck* puck);
	/** tare Method establishes new baseline for differential calculations. */
	void tare() { Puck::setProperty(*bus, id, propId, 0); }
	/** update Method establishes new force and torque values from the sensor */
	void update(bool realtime = false);
	/** getForce Method returns cartesian force values for each axis in n/m */
	const cf_type& getForce() const { return cf; }
	/** getTorque Method returns cartesian torque values for each axis in torque units */
	const ct_type& getTorque() const { return ct; }
	/** updateAccel Method clears stored acceleration values in each axis */
	void updateAccel(bool realtime = false);
	/** getAccel Method returns cartesian acceleration for each axis in n/m^2 */
	const ca_type& getAccel() const { return ca; }

	/** */
	struct ForceParser {
		static int busId(int id, int propId) {
			return Puck::encodeBusId(id, PuckGroup::FGRP_FT_FORCE);
		}

		static const double SCALE_FACTOR = 256.0;
		typedef cf_type result_type;
		static int parse(int id, int propId, result_type* result, const unsigned char* data, size_t len) {
			return ForceTorqueSensor::parse(id, propId, result, data, len, SCALE_FACTOR);
		}
	};
	/** */
	struct TorqueParser {
		static int busId(int id, int propId) {
			return Puck::encodeBusId(id, PuckGroup::FGRP_FT_TORQUE);
		}

		static const double SCALE_FACTOR = 4096.0;
		typedef ct_type result_type;
		static int parse(int id, int propId, result_type* result, const unsigned char* data, size_t len) {
			return ForceTorqueSensor::parse(id, propId, result, data, len, SCALE_FACTOR);
		}
	};
	/** */
	struct AccelParser {
		static int busId(int id, int propId) {
			return Puck::encodeBusId(id, PuckGroup::FGRP_FT_ACCEL);
		}

		static const double SCALE_FACTOR = 1024.0;
		typedef ca_type result_type;
		static int parse(int id, int propId, result_type* result, const unsigned char* data, size_t len) {
			return ForceTorqueSensor::parse(id, propId, result, data, len, SCALE_FACTOR);
		}
	};


protected:
	const bus::CommunicationsBus* bus;
	int id;
	int propId;

	cf_type cf;
	ct_type ct;
	ca_type ca;

private:
	typedef cf_type::Base base_type;

	static int twoByte2int(unsigned char lsb, unsigned char msb);
	static int parse(int id, int propId, base_type* result, const unsigned char* data, size_t len, double scaleFactor);

	DISALLOW_COPY_AND_ASSIGN(ForceTorqueSensor);

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


}


#endif /* BARRETT_PRODUCTS_FT_SENSOR_H_ */
