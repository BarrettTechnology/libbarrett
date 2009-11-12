/*
 * controller.cpp
 *
 *  Created on: Oct 4, 2009
 *      Author: dc
 */

#include <gtest/gtest.h>
#include <barrett/units.h>
#include <barrett/systems/abstract/controller.h>

#include "./controller_impl.h"
#include "../exposed_io_system.h"


namespace {
using namespace barrett;


// TODO(dc): actually test this

class ControllerTest : public ::testing::Test {
public:
	ControllerTest() :
		controllerImpl(), controller(controllerImpl) {}

protected:
	ControllerImpl<double> controllerImpl;
	systems::Controller<double>& controller;
};


TEST_F(ControllerTest, InitialOutputValueCtor) {
	ControllerImpl<double> ivControllerImpl(-88.1);
	systems::Controller<double>& ivController = ivControllerImpl;

	ExposedIOSystem<double> eios;
	systems::connect(ivController.controlOutput, eios.input);

	EXPECT_TRUE(eios.inputValueDefined()) << "initial value undefined";
	EXPECT_EQ(-88.1, eios.getInputValue())
		<< "wrong initial value given";
}


}
