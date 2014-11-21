/*
 * kinematics-inl.h
 *
 *  Created on: Jan 14, 2010
 *      Author: dc
 */

#include <libconfig.h++>

#include <barrett/units.h>
#include <barrett/cdlbt/kinematics.h>


namespace barrett {
namespace math {


template<size_t DOF>
Kinematics<DOF>::Kinematics(const libconfig::Setting& setting)
{
	if (bt_kinematics_create(&impl, setting.getCSetting(), DOF)) {
		throw(std::runtime_error("(math::Kinematics::Kinematics): Couldn't initialize Kinematics struct."));
	}
}

template<size_t DOF>
Kinematics<DOF>::~Kinematics()
{
	bt_kinematics_destroy(impl);
}

template<size_t DOF>
void Kinematics<DOF>::eval(const jp_type& jp, const jv_type& jv)
{
	bt_kinematics_eval(impl, jp.asGslType(), jv.asGslType());
}

template<size_t DOF>
units::CartesianPosition::type Kinematics<DOF>::operator() (const boost::tuple<jp_type, jv_type>& jointState)
{
	eval(boost::tuples::get<0>(jointState), boost::tuples::get<1>(jointState));
	return units::CartesianPosition::type(impl->tool->origin_pos);
}


}
}
