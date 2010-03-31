/*
 * tool_orientation_controller.h
 *
 *  Created on: Jan 22, 2010
 *      Author: dc
 */

#ifndef TOOL_ORIENTATION_CONTROLLER_H_
#define TOOL_ORIENTATION_CONTROLLER_H_


#include <Eigen/Geometry>

#include "../detail/ca_macro.h"
#include "../math/utils.h"
#include "../units.h"
#include "./abstract/controller.h"
#include "./kinematics_base.h"


#include <iostream>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>


namespace barrett {
namespace systems {



//inline std::ostream& operator<< (std::ostream& os, const Eigen::Quaterniond& q) {
//	os << "[" << q.w() << ", (" << q.x() << ", " << q.y() << ", " << q.z() << ")]";
//	return os;
//}


template<size_t DOF>
class ToolOrientationController : public Controller<Eigen::Quaterniond,
													units::JointTorques<DOF> >,
								  public KinematicsInput<DOF> {
public:
	ToolOrientationController() :
		KinematicsInput<DOF>(this) {}

protected:
	virtual void operate() {
//		Eigen::Quaterniond qError;
		Eigen::AngleAxisd error;
		double angle;
		Eigen::Vector3d torque;
		Eigen::Matrix<double, 3,DOF> jacobian;
		Eigen::Matrix<double, DOF,1> jTau;

//		error = this->referenceInput.getValue() * this->feedbackInput.getValue().inverse();  // I think it should be this way
		error = this->feedbackInput.getValue() * this->referenceInput.getValue().inverse();  // but CD's math (that works, BTW) does it this way
//		error = qError;

		angle = error.angle();
		// TODO(dc): I looked into Eigen's implementation and noticed that angle will always be between 0 and 2*pi. We should test for this so if Eigen changes, we notice.
		if (angle > M_PI) {
			angle -= 2.0*M_PI;
		}

//		if (math::abs(qError.w()) < 0.0005) {
		if (math::abs(angle) > 3.13) {	// a little dead-zone near the discontinuity at +/-180 degrees
//		if (false) {
			torque.setConstant(0.0);
		} else {
			torque = this->referenceInput.getValue().inverse() * (error.axis() * angle * 1.0);  // K_p = 5.0
		}
//		std::cout << qError << "\n" << error.axis() * angle * 1.0 << "\n" << torque << "\n";

		for (size_t r = 0; r < 3; ++r) {
			for (size_t c = 0; c < DOF; ++c) {
				jacobian(r,c) = gsl_matrix_get(this->kinInput.getValue()->impl->tool_jacobian_angular, r,c);
			}
		}
		jTau = jacobian.transpose() * torque;


		units::JointTorques<DOF> jt;
		for (size_t i = 0; i < DOF; ++i) {
			jt[i] = jTau[i];
		}
		this->controlOutputValue->setValue(jt);


//		std::cout << jTau << "\n\n";



//		math::Array<3> caa, tau;
//		math::Array<4> p, r, e;
//		units::JointTorques<DOF> jTau;
//		Eigen::Quaterniond tmp;
//
//		tmp = this->feedbackInput.getValue();
//		p << tmp.w(), tmp.x(), tmp.y(), tmp.z();
//
//		tmp = this->referenceInput.getValue();
//		r << tmp.w(), tmp.x(), tmp.y(), tmp.z();
//
//		q_mult_conj(e.asGslVector(), p.asGslVector(), r.asGslVector());
//		q_to_angle_axis(e.asGslVector(), caa.asGslVector());
//
//	      gsl_blas_dscal( 5.0, caa.asGslVector()); /* P TERM */
//
//	      /* Next, multiply it by the to_world transform
//	       * to get it in the world frame */
//	      gsl_blas_dgemv( CblasNoTrans, 1.0, this->kinInput.getValue()->impl->tool->rot_to_world,
//	                      caa.asGslVector(),
//	                      0.0, tau.asGslVector() );
//
////	      /* Also, add in the angular velocity (D term) */
////	      gsl_blas_daxpy( - c->rot_d, c->kin->tool_velocity_angular, c->torque ); /* D TERM */
//
//	      /* Multiply by the Jacobian-transpose at the tool (torque) */
//	      gsl_blas_dgemv( CblasTrans, 1.0, this->kinInput.getValue()->impl->tool_jacobian_angular,
//	                      tau.asGslVector(),
//	                      1.0, jTau.asGslVector());
//
//		std::cout << jTau << "\n";

	}

private:
	DISALLOW_COPY_AND_ASSIGN(ToolOrientationController);
};


}
}


#endif /* TOOL_ORIENTATION_CONTROLLER_H_ */