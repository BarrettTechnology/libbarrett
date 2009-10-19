/*
 * controller-inl.h
 *
 *  Created on: Oct 6, 2009
 *      Author: dc
 */


#include <list>
#include <stdexcept>

#include "../abstract/system.h"
#include "../helpers.h"
#include "../supervisory_controller.h"
#include "../abstract/joint_torque_adapter.h"


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

template<typename InputType, typename OutputType>
void Controller<InputType, OutputType>::selectAndConnectAdapter(
		const SupervisoryController& sc)
throw(std::invalid_argument)
{
	// will throw if no adapter is found
	JointTorqueAdapter& jta = sc.selectAdapter(controlOutput);

	// selectController guarantees this downcast won't fail
	Input<OutputType>* controlInput = NULL;
	controlInput = dynamic_cast<Input<OutputType>*>(  //NOLINT: see RTTI note in
			jta.getControlInput() );            // supervisory_controller-inl.h
	forceConnect(controlOutput, *controlInput);
}

template<typename InputType, typename OutputType>
void Controller<InputType, OutputType>::operate()
{
	// TODO(dc): stub
}


}
}
