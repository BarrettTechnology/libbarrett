/*
 * abstract_controller.cpp
 *
 *  Created on: Oct 7, 2009
 *      Author: dc
 */

#include <stdexcept>
#include <gtest/gtest.h>

#include <barrett/systems/abstract/abstract_controller.h>
#include <barrett/systems/supervisory_controller.h>

#include "./controller_impl.h"


namespace {
using namespace barrett;


// we just want this to compile
TEST(AbstractControllerTest, Interface) {
	ControllerImpl<double> c;
	systems::AbstractController& ac = c;

	ac.getReferenceInput();
	ac.getFeedbackInput();
	ac.getControlOutput();

	// this one is allowed to throw std::invalid_argument
	try {
		ac.selectAndConnectAdapter(systems::SupervisoryController());
	} catch (std::invalid_argument e) {
		// do nothing
	}
}


}
