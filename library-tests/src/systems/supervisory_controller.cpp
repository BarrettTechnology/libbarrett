/*
 * supervisory_controller.cpp
 *
 *  Created on: Oct 4, 2009
 *      Author: dc
 */

#include <stdexcept>

#include <gtest/gtest.h>
#include <barrett/systems.h>
#include <barrett/systems/abstract/system.h>
#include <barrett/systems/abstract/abstract_controller.h>
#include <barrett/units.h>

#include "./exposed_io_system.h"


namespace {
using namespace barrett;


// TODO(dc): actually test this
TEST(SupervisoryControllerTest, DefaultCtor) {
	systems::SupervisoryController sc;

	// verify default controllers and adapters are available
}

// TODO(dc): actually test this
TEST(SupervisoryControllerTest, NoDefaultCACtor) {
	systems::SupervisoryController sc(false);

	// verify default controllers and adapters are absent
}

TEST(SupervisoryControllerTest, SelectMethodsThrow) {
	systems::SupervisoryController sc(false);
	ExposedIOSystem<double> eios;  // make an Input and an Output

	// the default controllers/adapters/feedback signals are turned off,
	// so these should all fail
	EXPECT_THROW(sc.selectController(eios.output), std::invalid_argument);
	EXPECT_THROW(sc.selectFeedbackSignal(eios.input), std::invalid_argument);
	EXPECT_THROW(sc.selectAdapter(eios.output), std::invalid_argument);
	EXPECT_THROW(sc.trackReferenceSignal(eios.output), std::invalid_argument);
}


template<typename T>
class SupervisoryControllerTypedTest : public ::testing::Test {
public:
	SupervisoryControllerTypedTest() :
		sc(), referenceOutput(&referenceOutputValue) {}

protected:
	systems::SupervisoryController sc;
	systems::System::Output<T> referenceOutput;
	typename systems::System::Output<T>::Value* referenceOutputValue;
};

// template parameters list the types the following tests should be run over
typedef ::testing::Types<units::JointAngles> SCTypes;
TYPED_TEST_CASE(SupervisoryControllerTypedTest, SCTypes);


TYPED_TEST(SupervisoryControllerTypedTest, SelectController) {
	// fail gracefully if this function throws
	ASSERT_NO_THROW(this->sc.selectController(this->referenceOutput));
	systems::AbstractController& controller =
			this->sc.selectController(this->referenceOutput);

	systems::System::Input<TypeParam>* referenceInput = NULL;
	systems::System::Input<TypeParam>* feedbackInput = NULL;

	referenceInput = dynamic_cast<systems::System::Input<TypeParam>*>(  //NOLINT: RTTI
			controller.getReferenceInput() );
	feedbackInput = dynamic_cast<systems::System::Input<TypeParam>*>(  //NOLINT: RTTI
			controller.getFeedbackInput() );

	EXPECT_TRUE(referenceInput != NULL);
	EXPECT_TRUE(feedbackInput != NULL);
}

TYPED_TEST(SupervisoryControllerTypedTest, TrackReferenceSignal) {
	ASSERT_NO_THROW(this->sc.trackReferenceSignal(this->referenceOutput));

	// TODO(dc): make sure things are actually connected
}


}
