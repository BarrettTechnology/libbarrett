/*
 * controller.cpp
 *
 *  Created on: Oct 4, 2009
 *      Author: dc
 */

#include <gtest/gtest.h>
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


}
