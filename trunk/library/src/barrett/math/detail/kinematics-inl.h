/*
 * kinematics-inl.h
 *
 *  Created on: Jan 14, 2010
 *      Author: dc
 */

#ifndef KINEMATICSINL_H_
#define KINEMATICSINL_H_


#include <libconfig.h++>

#include "../../units.h"
#include "../../cdlbt/kinematics/kinematics.h"


namespace barrett {
namespace math {


Kinematics::Kinematics(const libconfig::Setting& setting)
{
	if (bt_kinematics_create(&impl, setting.getCSetting(), setting["moving"].getLength())) {
		throw(std::runtime_error("(math::Kinematics::Kinematics): Couldn't initialize Kinematics struct."));
	}
}

Kinematics::~Kinematics()
{
	bt_kinematics_destroy(impl);
}

void Kinematics::eval(const jp_type& jp, const jv_type& jv)
{
	bt_kinematics_eval(impl, jp.asGslType(), jv.asGslType());
}

units::cp_type Kinematics::operator() (const boost::tuple<jp_type, jv_type>& jointState)
{
	eval(boost::tuples::get<0>(jointState), boost::tuples::get<1>(jointState));
	return cp_type(impl->tool->origin_pos);
}


}
}


#endif /* KINEMATICSINL_H_ */
