/*
 * abstract_controller.cpp
 *
 *  Created on: Oct 7, 2009
 *      Author: dc
 */

#include <stdexcept>
#include <list>

#include <gtest/gtest.h>
#include <barrett/detail/ca_macro.h>
#include <barrett/systems.h>
#include <barrett/systems/abstract/abstract_controller.h>
#include <barrett/systems/supervisory_controller.h>
#include <barrett/systems/abstract/joint_torque_adapter.h>


namespace {
using namespace barrett;


class Controller : public systems::AbstractController {
// IO
public:		System::Input<double> referenceInput;
public:		System::Input<double> feedbackInput;
public:		Output<double> controlOutput;
protected:	Output<double>::Value* controlOutputValue;


public:
	Controller() :
		referenceInput(this),
		feedbackInput(this),
		controlOutput(&controlOutputValue) {}
	virtual ~Controller() {}

	virtual System::Input<double>* getReferenceInput()	{ return &referenceInput; }
	virtual System::Input<double>* getFeedbackInput()	{ return &feedbackInput; }
	virtual System::Output<double>* getControlOutput()	{ return &controlOutput; }

	virtual void selectAndConnectAdapter(const systems::SupervisoryController& sc)
	throw(std::invalid_argument) {}

protected:
	virtual void operate() {}

private:
	DISALLOW_COPY_AND_ASSIGN(Controller);
};

// we just want this to compile
TEST(AbstractControllerTest, Interface) {
	Controller c;
	c.getReferenceInput();
	c.getFeedbackInput();
	c.getControlOutput();
	c.selectAndConnectAdapter(systems::SupervisoryController());
}


}
