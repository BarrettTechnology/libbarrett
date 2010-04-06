/*
 * pid_controller-inl.h
 *
 *  Created on: Oct 20, 2009
 *      Author: dc
 */


#include <libconfig.h++>

#include "../../math/utils.h"
#include "../../thread/abstract/mutex.h"


namespace barrett {
namespace systems {


template<typename InputType, typename OutputType, typename MathTraits>
PIDController<InputType, OutputType, MathTraits>::PIDController()
{
	getSamplePeriodFromEM();
}

template<typename InputType, typename OutputType, typename MathTraits>
PIDController<InputType, OutputType, MathTraits>::PIDController(const libconfig::Setting& setting)
{
	getSamplePeriodFromEM();
	setFromConfig(setting);
}

template<typename InputType, typename OutputType, typename MathTraits>
PIDController<InputType, OutputType, MathTraits>&
PIDController<InputType, OutputType, MathTraits>::setFromConfig(const libconfig::Setting& setting)
{
	if (setting.exists("kp")) {
		setKp(unitless_type(setting["kp"]));
	}
	if (setting.exists("ki")) {
		setKi(unitless_type(setting["ki"]));
	}
	if (setting.exists("kd")) {
		setKd(unitless_type(setting["kd"]));
	}
	if (setting.exists("integrator_limit")) {
		setIntegratorLimit(unitless_type(setting["integrator_limit"]));
	}
	if (setting.exists("control_signal_limit")) {
		setControlSignalLimit(unitless_type(setting["control_signal_limit"]));
	}

	return *this;
}

template<typename InputType, typename OutputType, typename MathTraits>
PIDController<InputType, OutputType, MathTraits>&
PIDController<InputType, OutputType, MathTraits>::setSamplePeriod(double timeStep)
{
	T_s = timeStep;
	return *this;
}

template<typename InputType, typename OutputType, typename MathTraits>
PIDController<InputType, OutputType, MathTraits>&
PIDController<InputType, OutputType, MathTraits>::setKp(unitless_type proportionalGains)
{
	kp = proportionalGains;
	return *this;
}

template<typename InputType, typename OutputType, typename MathTraits>
PIDController<InputType, OutputType, MathTraits>&
PIDController<InputType, OutputType, MathTraits>::setKi(unitless_type integralGains)
{
	ki = integralGains;
	return *this;
}

template<typename InputType, typename OutputType, typename MathTraits>
PIDController<InputType, OutputType, MathTraits>&
PIDController<InputType, OutputType, MathTraits>::setKd(unitless_type derivitiveGains)
{
	kd = derivitiveGains;
	return *this;
}

template<typename InputType, typename OutputType, typename MathTraits>
PIDController<InputType, OutputType, MathTraits>&
PIDController<InputType, OutputType, MathTraits>::setIntegratorState(
		unitless_type integratorState)
{
	// intError is written and read in operate(), so it needs to be locked.
	SCOPED_LOCK(this->getEmMutex());
	intError = integratorState;
	return *this;
}

template<typename InputType, typename OutputType, typename MathTraits>
PIDController<InputType, OutputType, MathTraits>&
PIDController<InputType, OutputType, MathTraits>::setIntegratorLimit(
		unitless_type intSaturations)
{
	intErrorLimit = intSaturations;
	return *this;
}

template<typename InputType, typename OutputType, typename MathTraits>
PIDController<InputType, OutputType, MathTraits>&
PIDController<InputType, OutputType, MathTraits>::setControlSignalLimit(
		unitless_type csSaturations)
{
	controlSignalLimit = csSaturations;
	return *this;
}


template<typename InputType, typename OutputType, typename MathTraits>
inline void PIDController<InputType, OutputType, MathTraits>::resetIntegrator()
{
	setIntegratorState(unitless_type(0.0));
}

template<typename InputType, typename OutputType, typename MathTraits>
void PIDController<InputType, OutputType, MathTraits>::setExecutionManager(ExecutionManager* newEm)
{
	Controller<InputType, OutputType>::setExecutionManager(newEm);  // call super
	getSamplePeriodFromEM();
}


template<typename InputType, typename OutputType, typename MathTraits>
void PIDController<InputType, OutputType, MathTraits>::operate()
{
	typedef MathTraits MT;

	error = MT::sub(this->referenceInput.getValue(), this->feedbackInput.getValue());

	intError = MT::add(intError, MT::mult(ki, MT::mult(T_s, error_1)));
	if ( !intErrorLimit.isZero() ) {
		intError = math::saturate(intError, intErrorLimit);
	}

	controlSignal = MT::add(MT::mult(kp, error),
							MT::add(intError,
								MT::mult(kd, MT::div(MT::sub(error, error_1), T_s))));
	if ( !controlSignalLimit.isZero() ) {
		controlSignal = math::saturate(controlSignal, controlSignalLimit);
	}

	error_1 = error;

	this->controlOutputValue->setValue(controlSignal);
}


template<typename InputType, typename OutputType, typename MathTraits>
inline void PIDController<InputType, OutputType, MathTraits>::getSamplePeriodFromEM()
{
	if (this->isExecutionManaged()) {
		T_s = this->getExecutionManager()->getPeriod();
	} else {
		T_s = 0.0;
	}
}


}
}
