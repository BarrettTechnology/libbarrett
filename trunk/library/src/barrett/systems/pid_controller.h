/*
 * pid_controller.h
 *
 *  Created on: Oct 20, 2009
 *      Author: dc
 */

#ifndef PID_CONTROLLER_H_
#define PID_CONTROLLER_H_


#include <iostream>
#include "../units.h"
#include "./abstract/controller.h"


namespace barrett {
namespace systems {


// TODO(dc): rewrite as a PIDFilter + Controller<SingleIO>

template<typename InputType,
		 typename OutputType = typename InputType::actuator_type>
class PIDController : public Controller<InputType, OutputType> {
public:
	typedef typename InputType::array_type array_type;

	PIDController& setKp(array_type proportionalGains);
	PIDController& setKi(array_type integralGains);
	PIDController& setKd(array_type derivitiveGains);
	PIDController& setIntegratorState(array_type integratorState);
	PIDController& setIntegratorLimit(array_type intSaturations);
	PIDController& setControlSignalLimit(array_type csSaturations);

	void resetIntegrator();

protected:
	virtual void operate();

	InputType error, error_1;
	array_type intError, intErrorLimit;
	array_type kp, ki, kd;
	OutputType controlSignal, controlSignalLimit;
};


}
}


// include template definitions
#include "./detail/pid_controller-inl.h"


#endif /* PID_CONTROLLER_H_ */
