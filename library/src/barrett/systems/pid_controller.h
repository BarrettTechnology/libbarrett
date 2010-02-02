/*
 * pid_controller.h
 *
 *  Created on: Oct 20, 2009
 *      Author: dc
 */

#ifndef PID_CONTROLLER_H_
#define PID_CONTROLLER_H_


#include <libconfig.h>  // TODO(dc): remove this once everything uses the C++ version
#include <libconfig.h++>

#include "../detail/ca_macro.h"
#include "./abstract/execution_manager.h"
#include "./abstract/controller.h"


namespace barrett {
namespace systems {


// TODO(dc): rewrite as a PIDFilter + Controller<SingleIO>

template<typename InputType,
		 typename OutputType = typename InputType::actuator_type>
class PIDController : public Controller<InputType, OutputType> {
public:
	typedef typename InputType::array_type array_type;

	PIDController();
	explicit PIDController(const libconfig::Setting& setting);

	PIDController& setSamplePeriod(double timeStep);
	PIDController& setFromConfig(const libconfig::Setting& setting);
	PIDController& setKp(array_type proportionalGains);
	PIDController& setKi(array_type integralGains);
	PIDController& setKd(array_type derivitiveGains);
	PIDController& setIntegratorState(array_type integratorState);
	PIDController& setIntegratorLimit(array_type intSaturations);
	PIDController& setControlSignalLimit(array_type csSaturations);

	void resetIntegrator();

	double getSamplePeriod() const {  return T_s;  }
	array_type getKp() const {  return kp;  }
	array_type getKi() const {  return ki;  }
	array_type getKd() const {  return kd;  }
	array_type getIntegratorState() const {  return intError;  }
	array_type getIntegratorLimit() const {  return intErrorLimit;  }
	array_type getControlSignalLimit() const {  return controlSignalLimit;  }

	virtual void setExecutionManager(ExecutionManager* newEm);

protected:
	virtual void operate();

	double T_s;
	InputType error, error_1;
	array_type intError, intErrorLimit;
	array_type kp, ki, kd;
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
