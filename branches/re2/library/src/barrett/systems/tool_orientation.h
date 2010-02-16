/*
 * tool_orientation.h
 *
 *  Created on: Jan 21, 2010
 *      Author: dc
 */

#ifndef TOOL_ORIENTATION_H_
#define TOOL_ORIENTATION_H_

#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gsl/gsl_matrix.h>

#include "../detail/ca_macro.h"
#include "../units.h"
#include "./abstract/system.h"
#include "./abstract/single_io.h"
#include "./kinematics_base.h"


namespace barrett {
namespace systems {


// yields a quaternion representing the rotation from the world frame to the tool frame.
template<size_t DOF>
class ToolOrientation : public System, public KinematicsInput<DOF>,
						public SingleOutput<Eigen::Quaterniond> {
public:
	ToolOrientation() :
		KinematicsInput<DOF>(this),
		SingleOutput<Eigen::Quaterniond>(this) {}
	virtual ~ToolOrientation() {}

protected:
	virtual void operate() {
		Eigen::Matrix3d rot;

		for (size_t r = 0; r < 3; ++r) {
			for (size_t c = 0; c < 3; ++c) {
				rot(c,r) = gsl_matrix_get(this->kinInput.getValue()->impl->tool->rot_to_world, r,c);  // transpose to get tool to world transform
			}
		}

		this->outputValue->setValue(Eigen::Quaterniond(rot));
	}

private:
	DISALLOW_COPY_AND_ASSIGN(ToolOrientation);
};


}
}


#endif /* TOOL_ORIENTATION_H_ */
