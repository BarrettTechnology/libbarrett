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
 * pid_controller.h
 *
 *  Created on: Oct 20, 2009
 *      Author: dc
 */

#ifndef BARRETT_SYSTEMS_PID_CONTROLLER_H_
#define BARRETT_SYSTEMS_PID_CONTROLLER_H_


#include <Eigen/Core>
#include <libconfig.h++>

#include <barrett/detail/ca_macro.h>
#include <barrett/math/traits.h>
#include <barrett/systems/abstract/execution_manager.h>
#include <barrett/systems/abstract/controller.h>


namespace barrett {
namespace systems {


// TODO(dc): rewrite as a PIDFilter + Controller<SingleIO>

template<typename InputType,
		 typename OutputType = typename InputType::actuator_type,
		 typename MathTraits = math::Traits<InputType> >
class PIDController : public Controller<InputType, OutputType> {
public:
	typedef typename MathTraits::unitless_type unitless_type;

	explicit PIDController(const std::string& sysName = "PIDController");
	explicit PIDController(const libconfig::Setting& setting, const std::string& sysName = "PIDController");
	virtual ~PIDController() { this->mandatoryCleanUp(); }

	void setFromConfig(const libconfig::Setting& setting);
	void setKp(const unitless_type& proportionalGains);
	void setKi(const unitless_type& integralGains);
	void setKd(const unitless_type& derivitiveGains);
	void setIntegratorState(const unitless_type& integratorState);
	void setIntegratorLimit(const unitless_type& intSaturations);
	void setControlSignalLimit(const unitless_type& csSaturations);

	void resetIntegrator();

	unitless_type& getKp() {  return kp;  }
	const unitless_type& getKp() const {  return kp;  }
	unitless_type& getKi() {  return ki;  }
	const unitless_type& getKi() const {  return ki;  }
	unitless_type& getKd() {  return kd;  }
	const unitless_type& getKd() const {  return kd;  }
	const unitless_type& getIntegratorState() const {  return intError;  }
	unitless_type& getIntegratorLimit() {  return intErrorLimit;  }
	const unitless_type& getIntegratorLimit() const {  return intErrorLimit;  }
	OutputType& getControlSignalLimit() {  return controlSignalLimit;  }
	const OutputType& getControlSignalLimit() const {  return controlSignalLimit;  }

protected:
	void setSamplePeriod(double timeStep);

	virtual void operate();
	virtual void onExecutionManagerChanged() {
		Controller<InputType, OutputType>::onExecutionManagerChanged();  // First, call super
		getSamplePeriodFromEM();
	}

	double T_s;
	InputType error, error_1;
	unitless_type intError, intErrorLimit;
	unitless_type kp, ki, kd;
	OutputType controlSignal, controlSignalLimit;
//	unitless_type controlSignal, controlSignalLimit;

	void getSamplePeriodFromEM();

private:
	DISALLOW_COPY_AND_ASSIGN(PIDController);

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF(MathTraits::RequiresAlignment)
};


}
}


// include template definitions
#include <barrett/systems/detail/pid_controller-inl.h>


#endif /* BARRETT_SYSTEMS_PID_CONTROLLER_H_ */
