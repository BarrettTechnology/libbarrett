/*
 * supervisory_controller.cpp
 *
 *  Created on: Oct 29, 2009
 *      Author: dc
 */


#include <utility>
#include <stdexcept>

#include <gtest/gtest.h>
#include <barrett/units.h>
#include <barrett/systems/abstract/system.h>
#include <barrett/systems/supervisory_controller.h>

#include "./abstract/supervisory_controllable_impl.h"
#include "./exposed_io_system.h"


namespace {
using namespace barrett;


typedef units::JointTorques<12> jt_type;


class SupervisoryControllerTest : public ::testing::Test {
protected:
	systems::SupervisoryController<jt_type> sc;
};

// TODO(dc): actually test this
TEST_F(SupervisoryControllerTest, DefaultCtor) {

}

// TODO(dc): actually test this
TEST_F(SupervisoryControllerTest, RegisterControllable) {
	sc.registerControllable(
			new SupervisoryControllableImpl<jt_type, jt_type>());
}

// TODO(dc): actually test this
TEST_F(SupervisoryControllerTest, TrackReferenceSignalExplicit) {
	ExposedIOSystem<jt_type> eios;
	sc.trackReferenceSignal(eios.output,
			new SupervisoryControllableImpl<jt_type, jt_type>());
}

// TODO(dc): actually test this
TEST_F(SupervisoryControllerTest, TrackReferenceSignalAutomatic) {
	ExposedIOSystem<jt_type> eios;

	// this method is allowed to throw an invalid_argument
	try {
		sc.trackReferenceSignal(eios.output);
	} catch (std::invalid_argument) {}
}


}
