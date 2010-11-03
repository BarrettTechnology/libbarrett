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

#include <libconfig.h++>

#include <barrett/units.h>
#include <barrett/products/puck.h>
#include <barrett/products/motor_puck.h>
#include <barrett/products/puck_group.h>


namespace barrett {


template<size_t DOF>
class LowLevelWam {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

public:
	// genericPucks must be ordered by joint and must break into torque groups as arranged
	LowLevelWam(const std::vector<Puck*>& genericPucks, Puck* safetyPuck,
			const libconfig::Setting& setting,
			std::vector<int> torqueGroupIds = std::vector<int>());
	~LowLevelWam();

	void update();
	void setTorques(const jt_type& jt);

	void definePosition(const jp_type& jp);

protected:
	const CommunicationsBus& bus;
	std::vector<MotorPuck> pucks;
	Puck* safetyPuck;
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
