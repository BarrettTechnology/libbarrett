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
#include "./abstract/execution_manager.h"
#include "./abstract/controller.h"


namespace barrett {
namespace systems {


// TODO(dc): rewrite as a PIDFilter + Controller<SingleIO>

template<typename InputType,
		 typename OutputType = typename InputType::actuator_type>
class PIDController : public Controller<InputType, OutputType> {
public:
	typedef typename InputType::vector_type vector_type;

	PIDController();
	explicit PIDController(const libconfig::Setting& setting);

	PIDController& setSamplePeriod(double timeStep);
	PIDController& setFromConfig(const libconfig::Setting& setting);
	PIDController& setKp(vector_type proportionalGains);
	PIDController& setKi(vector_type integralGains);
	PIDController& setKd(vector_type derivitiveGains);
	PIDController& setIntegratorState(vector_type integratorState);
	PIDController& setIntegratorLimit(vector_type intSaturations);
	PIDController& setControlSignalLimit(vector_type csSaturations);

	void resetIntegrator();

	double getSamplePeriod() const {  return T_s;  }
	vector_type getKp() const {  return kp;  }
	vector_type getKi() const {  return ki;  }
	vector_type getKd() const {  return kd;  }
	vector_type getIntegratorState() const {  return intError;  }
	vector_type getIntegratorLimit() const {  return intErrorLimit;  }
	vector_type getControlSignalLimit() const {  return controlSignalLimit;  }

	virtual void setExecutionManager(ExecutionManager* newEm);

protected:
	virtual void operate();

	double T_s;
	InputType error, error_1;
	vector_type intError, intErrorLimit;
	vector_type kp, ki, kd;
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
