/*
 * force_torque_sensor.h
 *
 *  Created on: Nov 8, 2010
 *      Author: dc
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


	ForceTorqueSensor(Puck* puck = NULL) : SpecialPuck(/* TODO(dc): Puck::PT_ForceTorque */), bus(NULL) { setPuck(puck); }
	~ForceTorqueSensor() {}

	void setPuck(Puck* puck);

	void tare() { Puck::setProperty(*bus, id, propId, 0); }

	void update(bool realtime = false);
	const cf_type& getForce() const { return cf; }
	const ct_type& getTorque() const { return ct; }

	void updateAccel(bool realtime = false);
	const ca_type& getAccel() const { return ca; }


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
	struct AccelParser {
		static int busId(int id, int propId) {
			return Puck::encodeBusId(id, PuckGroup::FGRP_FT_ACCEL);
		}

		static const double SCALE_FACTOR = 16.0;
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
