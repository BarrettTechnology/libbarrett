/*
 * manual_execution_manager.cpp
 *
 *  Created on: Dec 9, 2009
 *      Author: dc
 */


#include <gtest/gtest.h>
#include <barrett/systems/manual_execution_manager.h>
#include "./exposed_io_system.h"


namespace {
using namespace barrett;


class ManualExecutionManagerTest : public ::testing::Test {
public:
	ManualExecutionManagerTest() :
		me() {
		systems::System::defaultExecutionManager = &me;
	}

protected:
	systems::ManualExecutionManager me;
};


TEST_F(ManualExecutionManagerTest, Dtor) {
	systems::ManualExecutionManager* localMe = new systems::ManualExecutionManager();
	systems::System::defaultExecutionManager = localMe;

	ExposedIOSystem<double> eios;
	EXPECT_TRUE(eios.isExecutionManaged());

	delete localMe;

	EXPECT_EQ(NULL, systems::System::defaultExecutionManager);
	EXPECT_FALSE(eios.isExecutionManaged());
}

TEST_F(ManualExecutionManagerTest, StartManaging) {
	systems::System::defaultExecutionManager = NULL;

	ExposedIOSystem<double> eios;
	EXPECT_FALSE(eios.isExecutionManaged());

	me.startManaging(&eios);
	EXPECT_TRUE(eios.isExecutionManaged());
}

TEST_F(ManualExecutionManagerTest, StopManaging) {
	ExposedIOSystem<double> eios;
	EXPECT_TRUE(eios.isExecutionManaged());

	me.stopManaging(&eios);
	EXPECT_FALSE(eios.isExecutionManaged());
}


}
