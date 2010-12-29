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
//	typedef typename InputType::unitless_type unitless_type;
	typedef InputType unitless_type;

	PIDController();
	explicit PIDController(const libconfig::Setting& setting);

	PIDController& setSamplePeriod(double timeStep);
	PIDController& setFromConfig(const libconfig::Setting& setting);
	PIDController& setKp(unitless_type proportionalGains);
	PIDController& setKi(unitless_type integralGains);
	PIDController& setKd(unitless_type derivitiveGains);
	PIDController& setIntegratorState(unitless_type integratorState);
	PIDController& setIntegratorLimit(unitless_type intSaturations);
	PIDController& setControlSignalLimit(unitless_type csSaturations);

	void resetIntegrator();

	double getSamplePeriod() const {  return T_s;  }
	unitless_type getKp() const {  return kp;  }
	unitless_type getKi() const {  return ki;  }
	unitless_type getKd() const {  return kd;  }
	unitless_type getIntegratorState() const {  return intError;  }
	unitless_type getIntegratorLimit() const {  return intErrorLimit;  }
	unitless_type getControlSignalLimit() const {  return controlSignalLimit;  }

	virtual void setExecutionManager(ExecutionManager* newEm);

protected:
	virtual void operate();

	double T_s;
	InputType error, error_1;
	unitless_type intError, intErrorLimit;
	unitless_type kp, ki, kd;
//	OutputType controlSignal, controlSignalLimit;
	unitless_type controlSignal, controlSignalLimit;

private:
	void getSamplePeriodFromEM();

	DISALLOW_COPY_AND_ASSIGN(PIDController);
};


}
}


// include template definitions
#include <barrett/systems/detail/pid_controller-inl.h>


#endif /* BARRETT_SYSTEMS_PID_CONTROLLER_H_ */
