/*
 * kinematics.h
 *
 *  Created on: Jan 14, 2010
 *      Author: dc
 */

#ifndef KINEMATICS_H_
#define KINEMATICS_H_


#include <libconfig.h>
#include <boost/tuple/tuple.hpp>

#include "../detail/ca_macro.h"
#include "../units.h"


// forward declaration from <barrett/kinematics/kinematics.h>
struct bt_kinematics;


namespace barrett {
namespace math {


template<size_t DOF>
class Kinematics {
public:
	typedef units::JointPositions<DOF> jp_type;
	typedef units::JointVelocities<DOF> jv_type;

	Kinematics(config_setting_t * config);
	~Kinematics();

	void eval(const jp_type& jp, const jv_type& jv);

	typedef units::CartesianPosition result_type;  ///< For use with boost::bind().
	result_type operator() (const boost::tuple<jp_type, jv_type>& jointState);

protected:
	struct bt_kinematics* impl;

private:
	DISALLOW_COPY_AND_ASSIGN(Kinematics);
};


}
}


// include template definitions
#include "./detail/kinematics-inl.h"


#endif /* KINEMATICS_H_ */
