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

const double T_s = 0.002;

class ManualExecutionManagerTest : public ::testing::Test {
public:
	ManualExecutionManagerTest() : mem(T_s) {
		mem.startManaging(eios);
	}

protected:
	systems::ManualExecutionManager mem;
	ExposedIOSystem<double> eios;
};


TEST_F(ManualExecutionManagerTest, PeriodCtor) {
	EXPECT_EQ(T_s, mem.getPeriod());
}

TEST_F(ManualExecutionManagerTest, ConfigCtor) {
	libconfig::Config config;
	config.readFile("test.config");
	systems::ManualExecutionManager mem2(config.lookup("manual_execution_manager_test"));
	EXPECT_EQ(0.5386, mem2.getPeriod());
}

TEST_F(ManualExecutionManagerTest, Dtor) {
	systems::ManualExecutionManager* localMem = new systems::ManualExecutionManager(T_s);

	localMem->startManaging(eios);
	EXPECT_TRUE(eios.hasExecutionManager());

	delete localMem;

	EXPECT_FALSE(eios.hasExecutionManager());
}

TEST_F(ManualExecutionManagerTest, StartManaging) {
	ExposedIOSystem<double> eios2;
	EXPECT_FALSE(eios2.hasExecutionManager());

	mem.runExecutionCycle();
	EXPECT_FALSE(eios2.operateCalled);

	mem.startManaging(eios2);
	EXPECT_TRUE(eios2.hasExecutionManager());
	EXPECT_EQ(&mem, eios2.getExecutionManager());

	mem.runExecutionCycle();
	EXPECT_TRUE(eios2.operateCalled);
}

TEST_F(ManualExecutionManagerTest, StartManagingAlreadyManaged) {
	systems::ManualExecutionManager localMem(T_s);

	EXPECT_TRUE(eios.hasExecutionManager());
	EXPECT_EQ(&mem, eios.getExecutionManager());
	localMem.startManaging(eios);
	EXPECT_TRUE(eios.hasExecutionManager());
	EXPECT_EQ(&localMem, eios.getExecutionManager());
}

TEST_F(ManualExecutionManagerTest, StopManaging) {
	EXPECT_TRUE(eios.hasExecutionManager());

	mem.runExecutionCycle();
	EXPECT_TRUE(eios.operateCalled);

	mem.stopManaging(eios);
	EXPECT_FALSE(eios.hasExecutionManager());
	EXPECT_EQ(NULL, eios.getExecutionManager());

	eios.operateCalled = false;
	mem.runExecutionCycle();
	EXPECT_FALSE(eios.operateCalled);
}

TEST_F(ManualExecutionManagerTest, ExecutionCycle) {
	EXPECT_TRUE(eios.hasExecutionManager());

	for (int i = 0; i < 10; ++i) {
		eios.operateCalled = false;
		mem.runExecutionCycle();
		EXPECT_TRUE(eios.operateCalled);
	}
}


}
