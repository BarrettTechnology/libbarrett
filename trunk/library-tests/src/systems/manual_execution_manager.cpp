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
		mem() {
		systems::System::defaultExecutionManager = &mem;
	}

protected:
	systems::ManualExecutionManager mem;
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

	mem.runExecutionCycle();
	EXPECT_FALSE(eios.operateCalled);

	mem.startManaging(&eios, true);
	EXPECT_TRUE(eios.isExecutionManaged());
	EXPECT_EQ(&mem, eios.getExecutionManager());

	mem.runExecutionCycle();
	EXPECT_TRUE(eios.operateCalled);
}

TEST_F(ManualExecutionManagerTest, StopManaging) {
	ExposedIOSystem<double> eios;
	EXPECT_TRUE(eios.isExecutionManaged());

	mem.runExecutionCycle();
	EXPECT_TRUE(eios.operateCalled);

	mem.stopManaging(&eios);
	EXPECT_FALSE(eios.isExecutionManaged());
	EXPECT_EQ(NULL, eios.getExecutionManager());

	eios.operateCalled = false;
	mem.runExecutionCycle();
	EXPECT_FALSE(eios.operateCalled);
}

TEST_F(ManualExecutionManagerTest, ExecutionCycle) {
	ExposedIOSystem<double> eios;
	EXPECT_TRUE(eios.isExecutionManaged());

	mem.runExecutionCycle();
	EXPECT_TRUE(eios.operateCalled);

	eios.operateCalled = false;
	mem.update();
	EXPECT_FALSE(eios.operateCalled);

	mem.runExecutionCycle();
	EXPECT_TRUE(eios.operateCalled);
}


}
