/*
 * controller-inl.h
 *
 *  Created on: Oct 6, 2009
 *      Author: dc
 */


#include <list>
#include <stdexcept>


namespace Systems {


template<typename InputType, typename OutputType>
inline System::Input<InputType>*
Controller<InputType, OutputType>::getReferenceInput()
{
	return &referenceInput;
}

template<typename InputType, typename OutputType>
inline System::Input<InputType>*
Controller<InputType, OutputType>::getFeedbackInput()
{
	return &feedbackInput;
}

template<typename InputType, typename OutputType>
inline System::Output<OutputType>*
Controller<InputType, OutputType>::getControlOutput()
{
	return &controlOutput;
}

template<typename InputType, typename OutputType>
void Controller<InputType, OutputType>::selectAdapter(
		const std::list<JointTorqueAdapter*>& adapters) const
throw(std::invalid_argument)
{
	// TODO(dc): stub
}

template<typename InputType, typename OutputType>
void Controller<InputType, OutputType>::operate()
{
	// TODO(dc): stub
}


}
