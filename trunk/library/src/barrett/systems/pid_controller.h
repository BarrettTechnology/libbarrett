/*
 * pid_controller.h
 *
 *  Created on: Oct 20, 2009
 *      Author: dc
 */

#ifndef PID_CONTROLLER_H_
#define PID_CONTROLLER_H_


#include <libconfig.h++>

#include "../detail/ca_macro.h"
#include "../math/traits.h"
#include "./abstract/execution_manager.h"
#include "./abstract/controller.h"


namespace barrett {
namespace systems {


// TODO(dc): rewrite as a PIDFilter + Controller<SingleIO>

template<typename InputType,
		 typename OutputType = typename InputType::actuator_type,
		 typename MathTraits = math::Traits<InputType> >
class PIDController : public Controller<InputType, OutputType> {
public:
	typedef typename InputType::unitless_type unitless_type;

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
	OutputType controlSignal, controlSignalLimit;

private:
	void getSamplePeriodFromEM();

	DISALLOW_COPY_AND_ASSIGN(PIDController);
};


}
}


// include template definitions
#include "./detail/pid_controller-inl.h"


#endif /* PID_CONTROLLER_H_ */
