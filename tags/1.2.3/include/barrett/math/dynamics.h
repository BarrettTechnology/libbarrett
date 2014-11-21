/*
 * dynamics.h
 *
 *  Created on: Nov 9, 2011
 *      Author: dc
 */

#ifndef BARRETT_MATH_DYNAMICS_H_
#define BARRETT_MATH_DYNAMICS_H_


#include <libconfig.h++>
#include <boost/tuple/tuple.hpp>

#include <barrett/detail/ca_macro.h>
#include <barrett/units.h>
#include <barrett/math/kinematics.h>


// forward declaration from <barrett/cdlbt/dynamics.h>
struct bt_dynamics;


namespace barrett {
namespace math {


template<size_t DOF>
class Dynamics {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

public:
	Dynamics(const libconfig::Setting& setting);
	~Dynamics();

	const jt_type& evalInverse(const Kinematics<DOF>& kin, const jv_type& jv, const ja_type& ja);

//	typedef const jt_type& result_type;  ///< For use with boost::bind().
//	result_type operator() (const boost::tuple<jv_type, ja_type>& jointState);

protected:
	struct bt_dynamics* impl;
	jt_type jt;

private:
	DISALLOW_COPY_AND_ASSIGN(Dynamics);
};


}
}


// include template definitions
#include <barrett/math/detail/dynamics-inl.h>


#endif /* BARRETT_MATH_DYNAMICS_H_ */
