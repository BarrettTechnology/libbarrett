/*
 * first_order_filter-inl.h
 *
 *  Created on: Nov 18, 2009
 *      Author: dc
 */


#include <barrett/systems/abstract/single_io.h>


namespace barrett {
namespace systems {


template<typename T>
FirstOrderFilter<T>::FirstOrderFilter(bool updateEveryExecutionCycle) :
	SingleIO<T, T>(updateEveryExecutionCycle)
{
	getSamplePeriodFromEM();
}

template<typename T>
FirstOrderFilter<T>::FirstOrderFilter(const libconfig::Setting& setting, bool updateEveryExecutionCycle) :
	SingleIO<T, T>(updateEveryExecutionCycle), math::FirstOrderFilter<T>(setting)
{
	getSamplePeriodFromEM();
}

// TODO(dc): anyway to remove the code duplication with PIDController?
template<typename T>
void FirstOrderFilter<T>::setExecutionManager(ExecutionManager* newEm)
{
	SingleIO<T, T>::setExecutionManager(newEm);  // call super
	getSamplePeriodFromEM();
}

template<typename T>
void FirstOrderFilter<T>::operate()
{
	this->outputValue->setValue(eval(this->input.getValue()));
}


// TODO(dc): anyway to remove the code duplication with PIDController?
template<typename T>
inline void FirstOrderFilter<T>::getSamplePeriodFromEM()
{
	if (this->isExecutionManaged()) {
		this->setSamplePeriod(this->getExecutionManager()->getPeriod());
	} else {
		this->setSamplePeriod(0.0);
	}
}


}
}
