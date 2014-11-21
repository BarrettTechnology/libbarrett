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
 * pid_controller-inl.h
 *
 *  Created on: Oct 20, 2009
 *      Author: dc
 */


#include <cassert>

#include <libconfig.h++>

#include <barrett/math/utils.h>
#include <barrett/thread/abstract/mutex.h>


namespace barrett {
namespace systems {


template<typename InputType, typename OutputType, typename MathTraits>
PIDController<InputType, OutputType, MathTraits>::PIDController(const std::string& sysName) :
	Controller<InputType, OutputType>(sysName),
	T_s(0.0), error(0.0), error_1(0.0), intError(0.0), intErrorLimit(0.0),
	kp(0.0), ki(0.0), kd(0.0), controlSignal(0.0), controlSignalLimit(0.0)
{
	getSamplePeriodFromEM();
}

template<typename InputType, typename OutputType, typename MathTraits>
PIDController<InputType, OutputType, MathTraits>::PIDController(
		const libconfig::Setting& setting, const std::string& sysName) :
	Controller<InputType, OutputType>(sysName),
	T_s(0.0), error(0.0), error_1(0.0), intError(0.0), intErrorLimit(0.0),
	kp(0.0), ki(0.0), kd(0.0), controlSignal(0.0), controlSignalLimit(0.0)
{
	getSamplePeriodFromEM();
	setFromConfig(setting);
}

template<typename InputType, typename OutputType, typename MathTraits>
void PIDController<InputType, OutputType, MathTraits>::setFromConfig(const libconfig::Setting& setting)
{
	if (setting.exists("kp")) {
		setKp(unitless_type(setting["kp"]));
	} else {
		setKp(unitless_type(0.0));
	}
	if (setting.exists("ki")) {
		setKi(unitless_type(setting["ki"]));
	} else {
		setKi(unitless_type(0.0));
	}
	if (setting.exists("kd")) {
		setKd(unitless_type(setting["kd"]));
	} else {
		setKd(unitless_type(0.0));
	}
	if (setting.exists("integrator_limit")) {
		setIntegratorLimit(unitless_type(setting["integrator_limit"]));
	} else {
		setIntegratorLimit(unitless_type(0.0));
	}
	if (setting.exists("control_signal_limit")) {
		setControlSignalLimit(unitless_type(setting["control_signal_limit"]));
	} else {
		setControlSignalLimit(unitless_type(0.0));
	}
}

template<typename InputType, typename OutputType, typename MathTraits>
void PIDController<InputType, OutputType, MathTraits>::setSamplePeriod(double timeStep)
{
	T_s = timeStep;
}

template<typename InputType, typename OutputType, typename MathTraits>
void PIDController<InputType, OutputType, MathTraits>::setKp(const unitless_type& proportionalGains)
{
	kp = proportionalGains;
}

template<typename InputType, typename OutputType, typename MathTraits>
void PIDController<InputType, OutputType, MathTraits>::setKi(const unitless_type& integralGains)
{
	ki = integralGains;
}

template<typename InputType, typename OutputType, typename MathTraits>
void PIDController<InputType, OutputType, MathTraits>::setKd(const unitless_type& derivitiveGains)
{
	kd = derivitiveGains;
}

template<typename InputType, typename OutputType, typename MathTraits>
void PIDController<InputType, OutputType, MathTraits>::setIntegratorState(
		const unitless_type& integratorState)
{
	// intError is written and read in operate(), so it needs to be locked.
	BARRETT_SCOPED_LOCK(this->getEmMutex());
	intError = integratorState;
}

template<typename InputType, typename OutputType, typename MathTraits>
void PIDController<InputType, OutputType, MathTraits>::setIntegratorLimit(
		const unitless_type& intSaturations)
{
	intErrorLimit = intSaturations;
}

template<typename InputType, typename OutputType, typename MathTraits>
void PIDController<InputType, OutputType, MathTraits>::setControlSignalLimit(
		const unitless_type& csSaturations)
{
	controlSignalLimit = csSaturations;
}


template<typename InputType, typename OutputType, typename MathTraits>
inline void PIDController<InputType, OutputType, MathTraits>::resetIntegrator()
{
	setIntegratorState(unitless_type(0.0));
}


template<typename InputType, typename OutputType, typename MathTraits>
void PIDController<InputType, OutputType, MathTraits>::operate()
{
	typedef MathTraits MT;

	error = MT::sub(this->referenceInput.getValue(), this->feedbackInput.getValue());

	intError = MT::add(intError, MT::mult(ki, MT::mult(T_s, error_1)));
	if (intErrorLimit != MT::zero()) {
		intError = math::saturate(intError, intErrorLimit);
	}

	controlSignal = MT::add(MT::mult(kp, error),
							MT::add(intError,
								MT::mult(kd, MT::div(MT::sub(error, error_1), T_s))));
	if (controlSignalLimit != MT::zero()) {
		controlSignal = math::saturate(controlSignal, controlSignalLimit);
	}

	error_1 = error;

	this->controlOutputValue->setData(&controlSignal);
}


template<typename InputType, typename OutputType, typename MathTraits>
void PIDController<InputType, OutputType, MathTraits>::getSamplePeriodFromEM()
{
	if (this->hasExecutionManager()) {
		assert(this->getExecutionManager()->getPeriod() > 0.0);
		setSamplePeriod(this->getExecutionManager()->getPeriod());
	} else {
		setSamplePeriod(0.0);
	}
}


}
}
