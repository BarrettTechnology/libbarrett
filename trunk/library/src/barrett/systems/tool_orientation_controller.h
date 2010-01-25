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
#include "../units.h"
#include "./abstract/controller.h"
#include "./kinematics_base.h"


#include <iostream>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <math.h> /* For sqrt() */
#include <gsl/gsl_math.h> /* For M_PI */



namespace barrett {
namespace systems {

inline int rot_to_q( gsl_matrix * rot, gsl_vector * quat )
{
   if ( gsl_matrix_get(rot,0,0) + gsl_matrix_get(rot,1,1) + gsl_matrix_get(rot,2,2) > 0.0 )
   {
      double t, s;
      t = gsl_matrix_get(rot,0,0) + gsl_matrix_get(rot,1,1) + gsl_matrix_get(rot,2,2) + 1.0;
      s = 0.5 / sqrt(t);
      gsl_vector_set(quat,0, s * t);
      gsl_vector_set(quat,3, s * ( gsl_matrix_get(rot,0,1) - gsl_matrix_get(rot,1,0) ) );
      gsl_vector_set(quat,2, s * ( gsl_matrix_get(rot,2,0) - gsl_matrix_get(rot,0,2) ) );
      gsl_vector_set(quat,1, s * ( gsl_matrix_get(rot,1,2) - gsl_matrix_get(rot,2,1) ) );
   }
   else if ( gsl_matrix_get(rot,0,0) > gsl_matrix_get(rot,1,1) && gsl_matrix_get(rot,0,0) > gsl_matrix_get(rot,2,2) )
   {
      double t, s;
      t = gsl_matrix_get(rot,0,0) - gsl_matrix_get(rot,1,1) - gsl_matrix_get(rot,2,2) + 1.0;
      s = 0.5 / sqrt(t);
      gsl_vector_set(quat,1, s * t );
      gsl_vector_set(quat,2, s * ( gsl_matrix_get(rot,0,1) + gsl_matrix_get(rot,1,0) ) );
      gsl_vector_set(quat,3, s * ( gsl_matrix_get(rot,2,0) + gsl_matrix_get(rot,0,2) ) );
      gsl_vector_set(quat,0, s * ( gsl_matrix_get(rot,1,2) - gsl_matrix_get(rot,2,1) ) );
   }
   else if ( gsl_matrix_get(rot,1,1) > gsl_matrix_get(rot,2,2) )
   {
      double t, s;
      t = - gsl_matrix_get(rot,0,0) + gsl_matrix_get(rot,1,1) - gsl_matrix_get(rot,2,2) + 1.0;
      s = 0.5 / sqrt(t);
      gsl_vector_set(quat,2, s * t );
      gsl_vector_set(quat,1, s * ( gsl_matrix_get(rot,0,1) + gsl_matrix_get(rot,1,0) ) );
      gsl_vector_set(quat,0, s * ( gsl_matrix_get(rot,2,0) - gsl_matrix_get(rot,0,2) ) );
      gsl_vector_set(quat,3, s * ( gsl_matrix_get(rot,1,2) + gsl_matrix_get(rot,2,1) ) );
   }
   else
   {
      double t, s;
      t = - gsl_matrix_get(rot,0,0) - gsl_matrix_get(rot,1,1) + gsl_matrix_get(rot,2,2) + 1.0;
      s = 0.5 / sqrt(t);
      gsl_vector_set(quat,3, s * t );
      gsl_vector_set(quat,0, s * ( gsl_matrix_get(rot,0,1) - gsl_matrix_get(rot,1,0) ) );
      gsl_vector_set(quat,1, s * ( gsl_matrix_get(rot,2,0) + gsl_matrix_get(rot,0,2) ) );
      gsl_vector_set(quat,2, s * ( gsl_matrix_get(rot,1,2) + gsl_matrix_get(rot,2,1) ) );
   }

   return 0;
}

/* Here, we calculate q_a = q_b * q_c' */
inline int q_mult_conj( gsl_vector * qa,  gsl_vector * qb,  gsl_vector * qc )
{
   gsl_vector_set(qa,0, + gsl_vector_get(qb,0) * gsl_vector_get(qc,0)
                        + gsl_vector_get(qb,1) * gsl_vector_get(qc,1)
                        + gsl_vector_get(qb,2) * gsl_vector_get(qc,2)
                        + gsl_vector_get(qb,3) * gsl_vector_get(qc,3));

   gsl_vector_set(qa,1, - gsl_vector_get(qb,0) * gsl_vector_get(qc,1)
                        + gsl_vector_get(qb,1) * gsl_vector_get(qc,0)
                        - gsl_vector_get(qb,2) * gsl_vector_get(qc,3)
                        + gsl_vector_get(qb,3) * gsl_vector_get(qc,2));

   gsl_vector_set(qa,2, - gsl_vector_get(qb,0) * gsl_vector_get(qc,2)
                        + gsl_vector_get(qb,1) * gsl_vector_get(qc,3)
                        + gsl_vector_get(qb,2) * gsl_vector_get(qc,0)
                        - gsl_vector_get(qb,3) * gsl_vector_get(qc,1));

   gsl_vector_set(qa,3, - gsl_vector_get(qb,0) * gsl_vector_get(qc,3)
                        - gsl_vector_get(qb,1) * gsl_vector_get(qc,2)
                        + gsl_vector_get(qb,2) * gsl_vector_get(qc,1)
                        + gsl_vector_get(qb,3) * gsl_vector_get(qc,0));

   return 0;
}

inline int q_to_angle_axis( gsl_vector * q, gsl_vector * angleaxis )
{
   double sin_a;
   double angle;

   sin_a = sqrt( 1.0 - gsl_vector_get(q,0)*gsl_vector_get(q,0) );

   angle = 2 * acos( gsl_vector_get(q,0) ); /* 0 - 2pi */
   if (angle > M_PI ) angle -= 2*M_PI;

   if ( fabs(gsl_vector_get(q,0)) < 0.0005 )
   {
      gsl_vector_set_zero(angleaxis);
      return 0;
   }

   gsl_vector_set(angleaxis,0, gsl_vector_get(q,1) / sin_a * angle );
   gsl_vector_set(angleaxis,1, gsl_vector_get(q,2) / sin_a * angle );
   gsl_vector_set(angleaxis,2, gsl_vector_get(q,3) / sin_a * angle );

   return 0;
}


template<size_t DOF>
class ToolOrientationController : public Controller<Eigen::Quaterniond,
													units::JointTorques<DOF> >,
								  public KinematicsInput<DOF> {
public:
	ToolOrientationController() :
		KinematicsInput<DOF>(this) {}

protected:
	virtual void operate() {
		Eigen::AngleAxisd error;
		Eigen::Vector3d torque;
		Eigen::Matrix<double, 3,DOF> jacobian;
		Eigen::Matrix<double, DOF,1> jTau;

//		error = this->referenceInput.getValue() * this->feedbackInput.getValue().inverse();  // I think it should be this way
		error = this->feedbackInput.getValue() * this->referenceInput.getValue().inverse();  // but CD's math (that works, BTW) does it this way
		torque = this->referenceInput.getValue() * (error.axis() * error.angle() * 5.0);  // K_p = 5.0

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
