/*
 * pid_controller-inl.h
 *
 *  Created on: Oct 20, 2009
 *      Author: dc
 */


namespace barrett {
namespace systems {


// TODO(dc): make PID controller
template<typename InputType, typename OutputType>
void PIDController<InputType, OutputType>::operate()
{
	InputType error =
			this->referenceInput.getValue() - this->feedbackInput.getValue();

	// copy error to output
	OutputType controlSignal;
	for (size_t i = 0; i < error.size(); ++i) {
		controlSignal[i] = error[i];
	}

	this->controlOutputValue->setValue(controlSignal);
}


}
}
