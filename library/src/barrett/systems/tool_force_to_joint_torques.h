/*
 * tool_force_to_joint_torques.h
 *
 *  Created on: Jan 18, 2010
 *      Author: dc
 */

#ifndef TOOL_FORCE_TO_JOINT_TORQUES_H_
#define TOOL_FORCE_TO_JOINT_TORQUES_H_


#include <gsl/gsl_blas.h>

#include "../detail/ca_macro.h"
#include "../units.h"
#include "../math/kinematics.h"
#include "./abstract/single_io.h"


namespace barrett {
namespace systems {


template<size_t DOF>
class ToolForceToJointTorques : public SingleIO<units::CartesianForce, units::JointTorques<DOF> > {
// IO
public:	System::Input<const math::Kinematics<DOF>*> kinInput;


public:
	ToolForceToJointTorques() :
		SingleIO<units::CartesianForce, units::JointTorques<DOF> >(), kinInput(this) {}
	virtual ~ToolForceToJointTorques() {}

protected:
	virtual void operate() {
		units::JointTorques<DOF> jt;

		// Multiply by the Jacobian-transpose at the tool
		gsl_blas_dgemv(CblasTrans, 1.0, kinInput.getValue()->impl->tool_jacobian_linear,
					this->input.getValue().asGslVector(), 1.0, jt.asGslVector());

		this->outputValue->setValue(jt);
	}

private:
	DISALLOW_COPY_AND_ASSIGN(ToolForceToJointTorques);
};


}
}


#endif /* TOOL_FORCE_TO_JOINT_TORQUES_H_ */
