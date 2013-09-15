/*
 * kinematics.h
 *
 *  Created on: Jan 14, 2010
 *      Author: dc
 */

#ifndef BARRETT_MATH_KINEMATICS_H_
#define BARRETT_MATH_KINEMATICS_H_


#include <libconfig.h++>
#include <boost/tuple/tuple.hpp>

#include <barrett/detail/ca_macro.h>
#include <barrett/units.h>


// forward declaration from <barrett/cdlbt/kinematics.h>
struct bt_kinematics;


namespace barrett {
namespace math {


template<size_t DOF>
class Kinematics {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

public:
	explicit Kinematics(const libconfig::Setting& setting);
	~Kinematics();

	void eval(const jp_type& jp, const jv_type& jv);

	typedef typename units::CartesianPosition::type result_type;  ///< For use with boost::bind().
	result_type operator() (const boost::tuple<jp_type, jv_type>& jointState);

//protected:
	struct bt_kinematics* impl;

private:
	DISALLOW_COPY_AND_ASSIGN(Kinematics);
};


}
}


// include template definitions
#include <barrett/math/detail/kinematics-inl.h>


#endif /* BARRETT_MATH_KINEMATICS_H_ */
