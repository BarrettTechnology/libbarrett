/*
 * first_order_filter-inl.h
 *
 *  Created on: Nov 18, 2009
 *      Author: dc
 */


#include "../abstract/single_io.h"


namespace barrett {
namespace systems {


template<typename T>
FirstOrderFilter<T>::FirstOrderFilter(bool updateEveryExecutionCycle) :
	SingleIO<T, T>(updateEveryExecutionCycle),
	a(), b(), c(), T_s(),
	c1(), c2(), c3(),
	y_0(), y_1(), x_0(), x_1()
{
	getSamplePeriodFromEM();
}

template<typename T>
void FirstOrderFilter<T>::setSamplePeriod(double timeStep)
{
	T_s = timeStep;
	updateCoefficients();
}

template<typename T>
void FirstOrderFilter<T>::setLowPass(T omega_p, T dcGain)
{
	a = T();
	b = dcGain * omega_p;
	c = omega_p;

	updateCoefficients();
}

template<typename T>
void FirstOrderFilter<T>::setHighPass(T omega_p, T dcGain)
{
	a = dcGain;
	b = T();
	c = omega_p;

	updateCoefficients();
}

template<typename T>
void FirstOrderFilter<T>::setZPK(T omega_z, T omega_p, T dcGain)
{
	a = dcGain * omega_p / omega_z;
	b = dcGain * omega_p;
	c = omega_p;

	updateCoefficients();
}

template<typename T>
void FirstOrderFilter<T>::setIntegrator(T gain)
{
	a = T();
	b = gain;
	c = T();

	updateCoefficients();
}

template<typename T>
void FirstOrderFilter<T>::setParameters(T a_, T b_, T c_)
{
	a = a_;
	b = b_;
	c = c_;

	updateCoefficients();
}

// TODO(dc): anyway to remove the code duplication with PIDController?
template<typename T>
void FirstOrderFilter<T>::setExecutionManager(ExecutionManager* newEm)
{
	SingleIO<T, T>::setExecutionManager(newEm);  // call super
	getSamplePeriodFromEM();
}


// filter parameters:
//         a*s + b
// H(s) = ---------
//          s + c
//
// difference equation:
//             1                    a + b*T_s                  a
// y[n] = ----------- * y[n-1]  +  ----------- * x[n]  -  ----------- * x[n-1]
//         1 + c*T_s                1 + c*T_s              1 + c*T_s
template<typename T>
void FirstOrderFilter<T>::updateCoefficients()
{
	T den = 1.0 + c*T_s;

	c1 = 1.0 / den;
	c2 = (a + b*T_s) / den;
	c3 = a / den;
}

template<typename T>
void FirstOrderFilter<T>::operate()
{
	x_0 = this->input.getValue();

	y_0 = c1*y_1 + c2*x_0 - c3*x_1;

	y_1 = y_0;
	x_1 = x_0;

	this->outputValue->setValue(y_0);
}


// TODO(dc): anyway to remove the code duplication with PIDController?
template<typename T>
inline void FirstOrderFilter<T>::getSamplePeriodFromEM()
{
	if (this->isExecutionManaged()) {
		T_s = this->getExecutionManager()->getPeriod();
	} else {
		T_s = 0.0;
	}
}


}
}
