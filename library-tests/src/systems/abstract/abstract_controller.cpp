/*
 * abstract_controller.cpp
 *
 *  Created on: Oct 7, 2009
 *      Author: dc
 */

#include <gtest/gtest.h>
#include <barrett/detail/ca_macro.h>
#include <barrett/systems.h>
#include <barrett/systems/abstract/abstract_controller.h>


namespace {


class Controller : public Systems::AbstractController {
	// IO
	public:		System::Input<double> referenceInput;
	public:		System::Input<double> feedbackInput;


	public:
		Controller() :
			referenceInput(this), feedbackInput(this) {}

		virtual System::Input<double>* getReferenceInput()	{ return &referenceInput; }
		virtual System::Input<double>* getFeedbackInput()	{ return &feedbackInput; }

	protected:
		virtual void operate() {}

	private:
		DISALLOW_COPY_AND_ASSIGN(Controller);
	};

// we just want this to compile
TEST(AbstractControllerTest, InterfaceExists) {
	Controller c;
	c.getReferenceInput();
	c.getFeedbackInput();
}


}
