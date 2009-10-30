/*
 * controller-inl.h
 *
 *  Created on: Oct 6, 2009
 *      Author: dc
 */


#include <list>
#include <stdexcept>

#include "../../../units.h"
#include "../system.h"
#include "../../helpers.h"


namespace barrett {
namespace systems {


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


}
}
