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

#include <libconfig.h++>

#include <barrett/units.h>
#include <barrett/products/puck.h>
#include <barrett/products/motor_puck.h>
#include <barrett/products/safety_module.h>
#include <barrett/products/puck_group.h>


namespace barrett {


template<size_t DOF>
class LowLevelWam {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

public:
	// genericPucks must be ordered by joint and must break into torque groups as arranged
	LowLevelWam(const std::vector<Puck*>& genericPucks, SafetyModule* safetyModule,
			const libconfig::Setting& setting,
			std::vector<int> torqueGroupIds = std::vector<int>());
	~LowLevelWam();


	const jp_type& getJointPositions() const { return jp; }
	const jv_type& getJointVelocities() const { return jv; }


	const std::vector<MotorPuck>& getPucks() const { return pucks; }
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

protected:
	const CommunicationsBus& bus;
	std::vector<MotorPuck> pucks;
	SafetyModule* safetyModule;
	PuckGroup wamGroup;
	std::vector<PuckGroup*> torqueGroups;

	jp_type home;
	sqm_type j2mp, m2jp, j2mt;
	sqm_type j2pp, p2jp, j2pt;

	RTIME lastUpdate;
	v_type pp;
	jp_type jp, jp_1;
	jv_type jv;

	v_type pt;
	int torquePropId;

private:
	DISALLOW_COPY_AND_ASSIGN(LowLevelWam);
};


}


// include template definitions
#include <barrett/products/detail/low_level_wam-inl.h>


#endif /* BARRETT_PRODUCTS_LOW_LEVEL_WAM_H_ */
