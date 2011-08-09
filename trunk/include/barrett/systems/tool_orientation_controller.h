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
 * tool_orientation_controller.h
 *
 *  Created on: Jan 22, 2010
 *      Author: dc
 */

#ifndef BARRETT_SYSTEMS_TOOL_ORIENTATION_CONTROLLER_H_
#define BARRETT_SYSTEMS_TOOL_ORIENTATION_CONTROLLER_H_


#include <libconfig.h++>
#include <Eigen/Geometry>
#include <gsl/gsl_blas.h>

#include <barrett/detail/ca_macro.h>
#include <barrett/detail/libconfig_utils.h>
#include <barrett/math/utils.h>
#include <barrett/units.h>
#include <barrett/systems/abstract/controller.h>
#include <barrett/systems/kinematics_base.h>


namespace barrett {
namespace systems {


template<size_t DOF>
class ToolOrientationController : public Controller<Eigen::Quaterniond,
													units::CartesianTorque::type>,
								  public KinematicsInput<DOF> {

	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

public:
	explicit ToolOrientationController(const std::string& sysName = "ToolOrientationController") :
		Controller<Eigen::Quaterniond, ct_type>(sysName), KinematicsInput<DOF>(this),
		kp(0.0), kd(0.0) {}
	explicit ToolOrientationController(const libconfig::Setting& setting,
			const std::string& sysName = "ToolOrientationController") :
		Controller<Eigen::Quaterniond, ct_type>(sysName), KinematicsInput<DOF>(this),
		kp(0.0), kd(0.0)
	{
		setFromConfig(setting);
	}
	virtual ~ToolOrientationController() { mandatoryCleanUp(); }


	void setFromConfig(const libconfig::Setting& setting) {
		setKp(barrett::detail::numericToDouble(setting["kp"]));
		setKd(barrett::detail::numericToDouble(setting["kd"]));
	}
	void setKp(double proportionalGain) { kp = proportionalGain; }
	void setKd(double derivitiveGain) { kd = derivitiveGain; }

	double getKp() const { return kp; }
	double getKd() const { return kd; }

protected:
	double kp;
	double kd;

	Eigen::AngleAxisd error;
	ct_type ct;

	virtual void operate() {
//		error = this->referenceInput.getValue() * this->feedbackInput.getValue().inverse();  // I think it should be this way
		error = this->feedbackInput.getValue() * this->referenceInput.getValue().inverse();  // but CD's math (that works, BTW) does it this way

		double angle = error.angle();
		// TODO(dc): I looked into Eigen's implementation and noticed that angle will always be between 0 and 2*pi. We should test for this so if Eigen changes, we notice.
		if (angle > M_PI) {
			angle -= 2.0*M_PI;
		}

		if (math::abs(angle) > 3.13) {	// a little dead-zone near the discontinuity at +/-180 degrees
			ct.setZero();
		} else {
			ct = this->referenceInput.getValue().inverse() * (error.axis() * angle * kp);
		}

		gsl_blas_daxpy( -kd, this->kinInput.getValue().impl->tool_velocity_angular, ct.asGslType());

		this->controlOutputValue->setData(&ct);
	}

private:
	DISALLOW_COPY_AND_ASSIGN(ToolOrientationController);

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


}
}


#endif /* BARRETT_SYSTEMS_TOOL_ORIENTATION_CONTROLLER_H_ */
