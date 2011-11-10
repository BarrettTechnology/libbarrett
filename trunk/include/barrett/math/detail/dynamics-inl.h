/*
 * dynamics-inl.h
 *
 *  Created on: Nov 9, 2011
 *      Author: dc
 */

#include <libconfig.h++>

#include <barrett/units.h>
#include <barrett/cdlbt/dynamics.h>


namespace barrett {
namespace math {


template<size_t DOF>
Dynamics<DOF>::Dynamics(const libconfig::Setting& setting)
{
	if (bt_dynamics_create(&impl, setting.getCSetting(), DOF)) {
		throw(std::runtime_error("(math::Dynamics::Dynamics): Couldn't initialize Dynamics struct."));
	}
}

template<size_t DOF>
Dynamics<DOF>::~Dynamics()
{
	bt_dynamics_destroy(impl);
}

template<size_t DOF>
const typename units::JointTorques<DOF>::type& Dynamics<DOF>::evalInverse(const Kinematics<DOF>& kin, const jv_type& jv, const ja_type& ja)
{
	bt_dynamics_eval_inverse(impl, kin.impl, jv.asGslType(), ja.asGslType(), jt.asGslType());
	return jt;
}

//template<size_t DOF>
//const units::JointTorques<DOF>::type& Dynamics<DOF>::operator() (const boost::tuple<jv_type, ja_type>& jointState)
//{
//	return eval(boost::tuples::get<0>(jointState), boost::tuples::get<1>(jointState));
//}


}
}
