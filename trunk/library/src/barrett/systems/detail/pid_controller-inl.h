/*
 * pid_controller-inl.h
 *
 *  Created on: Oct 20, 2009
 *      Author: dc
 */


//#include <algorithm>
#include "../../detail/math_utils.h"


namespace barrett {
namespace systems {


// TODO(dc): make PID controller
template<typename InputType, typename OutputType>
void PIDController<InputType, OutputType>::operate()
{
	if ( !(this->referenceInput.valueDefined()  &&
		   this->feedbackInput.valueDefined()) ) {
		this->controlOutputValue->setValueUndefined();
		return;
	}

//	InputType error;
	InputType error =
			this->referenceInput.getValue() - this->feedbackInput.getValue();

	double kp[] = { 3e3, 1e3, 1e2, 1e2, 0.0, 0.0, 0.0 };
	double limits[] = { 25.0, 20.0, 15.0, 15.0, 0.0, 0.0, 0.0 };
	OutputType controlSignal;
	for (size_t i = 0; i < error.size(); ++i) {
		controlSignal[i] = symLimit(kp[i] * error[i], limits[i]);
	}

	this->controlOutputValue->setValue(controlSignal);
}


}
}
