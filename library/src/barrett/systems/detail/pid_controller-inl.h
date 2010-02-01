/*
 * pid_controller-inl.h
 *
 *  Created on: Oct 20, 2009
 *      Author: dc
 */


#include <libconfig.h>  // TODO(dc): remove this once everything uses the C++ version
#include <libconfig.h++>

#include "../../math/utils.h"
#include "../../thread/abstract/mutex.h"


namespace barrett {
namespace systems {


template<typename InputType, typename OutputType>
PIDController<InputType, OutputType>::PIDController(const libconfig::Setting& setting)
{
	setFromConfig(setting);
}

template<typename InputType, typename OutputType>
PIDController<InputType, OutputType>&
PIDController<InputType, OutputType>::setFromConfig(const libconfig::Setting& setting)
{
	if (setting.exists("kp")) {
		setKp(array_type(setting["kp"]));
	}
	if (setting.exists("ki")) {
		setKi(array_type(setting["ki"]));
	}
	if (setting.exists("kd")) {
		setKd(array_type(setting["kd"]));
	}
	if (setting.exists("integrator_limit")) {
		setIntegratorLimit(array_type(setting["integrator_limit"]));
	}
	if (setting.exists("control_signal_limit")) {
		setControlSignalLimit(array_type(setting["control_signal_limit"]));
	}

	return *this;
}

template<typename InputType, typename OutputType>
PIDController<InputType, OutputType>&
PIDController<InputType, OutputType>::setKp(array_type proportionalGains)
{
	kp = proportionalGains;
	return *this;
}

template<typename InputType, typename OutputType>
PIDController<InputType, OutputType>&
PIDController<InputType, OutputType>::setKi(array_type integralGains)
{
	ki = integralGains;
	return *this;
}

template<typename InputType, typename OutputType>
PIDController<InputType, OutputType>&
PIDController<InputType, OutputType>::setKd(array_type derivitiveGains)
{
	kd = derivitiveGains;
	return *this;
}

template<typename InputType, typename OutputType>
PIDController<InputType, OutputType>&
PIDController<InputType, OutputType>::setIntegratorState(
		array_type integratorState)
{
	// intError is written and read in operate(), so it needs to be locked.
	SCOPED_LOCK(this->getEmMutex());
	intError = integratorState;
	return *this;
}

template<typename InputType, typename OutputType>
PIDController<InputType, OutputType>&
PIDController<InputType, OutputType>::setIntegratorLimit(
		array_type intSaturations)
{
	intErrorLimit = intSaturations;
	return *this;
}

template<typename InputType, typename OutputType>
PIDController<InputType, OutputType>&
PIDController<InputType, OutputType>::setControlSignalLimit(
		array_type csSaturations)
{
	controlSignalLimit = csSaturations;
	return *this;
}


template<typename InputType, typename OutputType>
inline void PIDController<InputType, OutputType>::resetIntegrator()
{
	setIntegratorState(array_type(0.0));
}


template<typename InputType, typename OutputType>
void PIDController<InputType, OutputType>::operate()
{
	// TODO(dc): ugly! ugly! should have a time input or something...
	double T_s = 0.002;

	error = this->referenceInput.getValue() - this->feedbackInput.getValue();

	intError = intError + ki * T_s * error_1;  // TODO(dc): += operators
	if ( !intErrorLimit.isZero() ) {
		intError = math::saturate(intError, intErrorLimit);
	}

	controlSignal = kp * error +
					intError +
					kd * (error - error_1) / T_s;
	if ( !controlSignalLimit.isZero() ) {
		controlSignal = math::saturate(controlSignal, controlSignalLimit);
	}

	error_1 = error;

	this->controlOutputValue->setValue(controlSignal);
}


}
}
