/*
 * low_level_wam.h
 *
 *  Created on: Oct 13, 2010
 *      Author: cd
 *      Author: dc
 */

#ifndef BARRETT_PRODUCTS_LOW_LEVEL_WAM_H_
#define BARRETT_PRODUCTS_LOW_LEVEL_WAM_H_


#include <vector>
#include <native/timer.h>

#include <boost/array.hpp>
#include <Eigen/Core>
#include <libconfig.h++>

#include <barrett/units.h>
#include <barrett/products/puck.h>
#include <barrett/products/motor_puck.h>
#include <barrett/products/safety_module.h>
#include <barrett/products/puck_group.h>
#include <barrett/products/abstract/multi_puck_product.h>


namespace barrett {


template<size_t DOF>
class LowLevelWam : public MultiPuckProduct {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

public:
	// pucks must be ordered by joint and must break into torque groups as arranged
	LowLevelWam(const std::vector<Puck*>& pucks, SafetyModule* safetyModule,
			const libconfig::Setting& setting,
			std::vector<int> torqueGroupIds = std::vector<int>());
	~LowLevelWam();


	enum PositionSensor { PS_BEST, PS_MOTOR_ENCODER, PS_JOINT_ENCODER };
	const jp_type& getJointPositions(enum PositionSensor sensor = PS_BEST) const {
		switch (sensor) {
		case PS_MOTOR_ENCODER:
			return jp_motorEncoder;
			break;
		case PS_JOINT_ENCODER:
			return jp_jointEncoder;
			break;
		default:
			return jp_best;
		}
	}
	const jv_type& getJointVelocities() const { return jv_best; }


	bool hasJointEncoders() const { return !noJointEncoders; }
	void setPositionSensor(enum PositionSensor sensor);


	const jp_type& getHomePosition() const { return home; }

	const sqm_type& getJointToMotorPositionTransform() const { return j2mp; }
	const sqm_type& getMotorToJointPositionTransform() const { return m2jp; }
	const sqm_type& getJointToMotorTorqueTransform() const { return j2mt; }

	const sqm_type& getJointToPuckPositionTransform() const { return j2pp; }
	const sqm_type& getPuckToJointPositionTransform() const { return p2jp; }
	const sqm_type& getJointToPuckTorqueTransform() const { return j2pt; }


	void update();
	void setTorques(const jt_type& jt);
	void definePosition(const jp_type& jp);


	SafetyModule* getSafetyModule() const { return safetyModule; }

protected:
	SafetyModule* safetyModule;
	std::vector<PuckGroup*> torqueGroups;

	jp_type home;
	sqm_type j2mp, m2jp, j2mt;
	sqm_type j2pp, p2jp, j2pt;
	v_type jointEncoder2jp;

	bool noJointEncoders;
	boost::array<bool, DOF> useJointEncoder;

	RTIME lastUpdate;
	v_type pp;
	math::Matrix<DOF,2> pp_jep;
	jp_type jp_motorEncoder, jp_jointEncoder;
	jp_type jp_best, jp_best_1;
	jv_type jv_best;

	v_type pt;
	int torquePropId;

private:
	static const enum Puck::Property props[];

	DISALLOW_COPY_AND_ASSIGN(LowLevelWam);

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


}


// include template definitions
#include <barrett/products/detail/low_level_wam-inl.h>


#endif /* BARRETT_PRODUCTS_LOW_LEVEL_WAM_H_ */
