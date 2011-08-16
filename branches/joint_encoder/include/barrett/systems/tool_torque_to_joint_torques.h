/*
	Copyright 2009, 2010 Barrett Technology <support@barrett.com>

	This file is part of libbarrett.

	This version of libbarrett is free software: you can redistribute it
	and/or modify it under the terms of the GNU General Public License as
	published by the Free Software Foundation, either version 3 of the
	License, or (at your option) any later version.

	This version of libbarrett is distributed in the hope that it will be
	useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License along
	with this version of libbarrett.  If not, see
	<http://www.gnu.org/licenses/>.

	Further, non-binding information about licensing is available at:
	<http://wiki.barrett.com/libbarrett/wiki/LicenseNotes>
*/

/*
 * tool_torque_to_joint_torques.h
 *
 *  Created on: Jan 24, 2011
 *      Author: dc
 */

#ifndef BARRETT_SYSTEMS_TOOL_TORQUE_TO_JOINT_TORQUES_H_
#define BARRETT_SYSTEMS_TOOL_TORQUE_TO_JOINT_TORQUES_H_


#include <Eigen/Core>
#include <gsl/gsl_blas.h>

#include <barrett/detail/ca_macro.h>
#include <barrett/units.h>
#include <barrett/math/kinematics.h>
#include <barrett/systems/abstract/single_io.h>
#include <barrett/systems/kinematics_base.h>


namespace barrett {
namespace systems {


template<size_t DOF>
class ToolTorqueToJointTorques :
	public SingleIO<units::CartesianTorque::type, typename units::JointTorques<DOF>::type>,
	public KinematicsInput<DOF>
{

	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

public:
	ToolTorqueToJointTorques(const std::string& sysName = "ToolTorqueToJointTorques") :
		SingleIO<ct_type, jt_type>(sysName), KinematicsInput<DOF>(this) {}
	virtual ~ToolTorqueToJointTorques() { this->mandatoryCleanUp(); }

protected:
	jt_type data;

	virtual void operate() {
		// Multiply by the Jacobian-transpose at the tool
		gsl_blas_dgemv(CblasTrans, 1.0,
				this->kinInput.getValue().impl->tool_jacobian_angular,
				this->input.getValue().asGslType(), 0.0, data.asGslType());

		this->outputValue->setData(&data);
	}

private:
	DISALLOW_COPY_AND_ASSIGN(ToolTorqueToJointTorques);

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


}
}


#endif /* BARRETT_SYSTEMS_TOOL_TORQUE_TO_JOINT_TORQUES_H_ */
