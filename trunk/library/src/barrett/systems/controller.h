/*
 * controller.h
 *
 *  Created on: Oct 4, 2009
 *      Author: dc
 */

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "abstract/system.h"
#include "abstract/abstract_controller.h"
#include "../detail/ca_macro.h"


namespace Systems {


template<typename InputType, typename OutputType = InputType>
class Controller : public AbstractController {
// IO
public:		System::Input<InputType> referenceInput;
public:		System::Input<InputType> feedbackInput;


public:
	Controller() :
		referenceInput(this), feedbackInput(this) {}

	virtual System::Input<InputType>* getReferenceInput();
	virtual System::Input<InputType>* getFeedbackInput();

protected:
	virtual void operate();

private:
	DISALLOW_COPY_AND_ASSIGN(Controller);
};


}


// include template definitions
#include "./detail/controller-inl.h"


#endif /* CONTROLLER_H_ */
