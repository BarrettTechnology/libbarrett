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
	ManualExecutionManagerTest() {
		mem.startManaging(eios);
	}

protected:
	systems::ManualExecutionManager mem;
	ExposedIOSystem<double> eios;
};


TEST_F(ManualExecutionManagerTest, DefaultCtor) {
	EXPECT_LT(mem.getPeriod(), 0.0);
}

TEST_F(ManualExecutionManagerTest, PeriodCtor) {
	const double T_s = 0.002;
	systems::ManualExecutionManager mem2(T_s);

	EXPECT_EQ(T_s, mem2.getPeriod());
}

TEST_F(ManualExecutionManagerTest, ConfigCtor) {
	libconfig::Config config;
	config.readFile("test.config");
	systems::ManualExecutionManager mem2(config.lookup("manual_execution_manager_test"));
	EXPECT_EQ(0.5386, mem2.getPeriod());
}

TEST_F(ManualExecutionManagerTest, Dtor) {
	systems::ManualExecutionManager* localMem = new systems::ManualExecutionManager;

	localMem->startManaging(eios);
	EXPECT_TRUE(eios.hasExecutionManager());
	EXPECT_TRUE(eios.hasDirectExecutionManager());

	delete localMem;

	EXPECT_FALSE(eios.hasExecutionManager());
	EXPECT_FALSE(eios.hasDirectExecutionManager());
}

TEST_F(ManualExecutionManagerTest, StartManaging) {
	ExposedIOSystem<double> eios2;
	EXPECT_FALSE(eios2.hasExecutionManager());
	EXPECT_FALSE(eios2.hasDirectExecutionManager());

	mem.runExecutionCycle();
	EXPECT_FALSE(eios2.operateCalled);

	mem.startManaging(eios2);
	EXPECT_TRUE(eios2.hasExecutionManager());
	EXPECT_TRUE(eios2.hasDirectExecutionManager());
	EXPECT_EQ(&mem, eios2.getExecutionManager());

	mem.runExecutionCycle();
	EXPECT_TRUE(eios2.operateCalled);
}

TEST_F(ManualExecutionManagerTest, StartManagingAlreadyManaged) {
	systems::ManualExecutionManager localMem;

	EXPECT_TRUE(eios.hasExecutionManager());
	EXPECT_TRUE(eios.hasDirectExecutionManager());
	EXPECT_EQ(&mem, eios.getExecutionManager());
	localMem.startManaging(eios);
	EXPECT_TRUE(eios.hasExecutionManager());
	EXPECT_TRUE(eios.hasDirectExecutionManager());
	EXPECT_EQ(&localMem, eios.getExecutionManager());
}

TEST_F(ManualExecutionManagerTest, StartManagingIndirect) {
	ExposedIOSystem<double> eios2;
	EXPECT_FALSE(eios2.hasExecutionManager());
	EXPECT_FALSE(eios2.hasDirectExecutionManager());
	EXPECT_EQ(NULL, eios2.getExecutionManager());

	ExposedIOSystem<double> eios3;
	EXPECT_FALSE(eios3.hasExecutionManager());
	EXPECT_FALSE(eios3.hasDirectExecutionManager());
	EXPECT_EQ(NULL, eios3.getExecutionManager());

	systems::connect(eios2.output, eios3.input);
	mem.startManaging(eios3);

	EXPECT_TRUE(eios2.hasExecutionManager());
	EXPECT_FALSE(eios2.hasDirectExecutionManager());
	EXPECT_EQ(&mem, eios2.getExecutionManager());

	EXPECT_TRUE(eios3.hasExecutionManager());
	EXPECT_TRUE(eios3.hasDirectExecutionManager());
	EXPECT_EQ(&mem, eios3.getExecutionManager());
}

TEST_F(ManualExecutionManagerTest, StartDirectlyManagingWhenAlreadyManaged) {
	ExposedIOSystem<double> eios1;
	ExposedIOSystem<double> eios2;
	ExposedIOSystem<double> eios3;

	systems::connect(eios1.output, eios2.input);
	systems::connect(eios2.output, eios3.input);

	mem.startManaging(eios3);
	EXPECT_FALSE(eios1.hasDirectExecutionManager());
	EXPECT_FALSE(eios2.hasDirectExecutionManager());
	EXPECT_TRUE(eios3.hasDirectExecutionManager());

	mem.startManaging(eios1);
	EXPECT_TRUE(eios1.hasDirectExecutionManager());
	EXPECT_FALSE(eios2.hasDirectExecutionManager());
	EXPECT_TRUE(eios3.hasDirectExecutionManager());

	mem.startManaging(eios2);
	EXPECT_TRUE(eios1.hasDirectExecutionManager());
	EXPECT_TRUE(eios2.hasDirectExecutionManager());
	EXPECT_TRUE(eios3.hasDirectExecutionManager());
}

TEST_F(ManualExecutionManagerTest, StartManagingInMiddleOfChain) {
	ExposedIOSystem<double> eios1;
	ExposedIOSystem<double> eios2;
	ExposedIOSystem<double> eios3;

	systems::connect(eios1.output, eios2.input);
	systems::connect(eios2.output, eios3.input);

	mem.startManaging(eios2);

	EXPECT_TRUE(eios1.hasExecutionManager());
	EXPECT_FALSE(eios1.hasDirectExecutionManager());
	EXPECT_EQ(&mem, eios1.getExecutionManager());

	EXPECT_TRUE(eios2.hasExecutionManager());
	EXPECT_TRUE(eios2.hasDirectExecutionManager());
	EXPECT_EQ(&mem, eios2.getExecutionManager());

	EXPECT_FALSE(eios3.hasExecutionManager());
	EXPECT_FALSE(eios3.hasDirectExecutionManager());
	EXPECT_EQ(NULL, eios3.getExecutionManager());
}

TEST_F(ManualExecutionManagerTest, StopManaging) {
	EXPECT_TRUE(eios.hasExecutionManager());
	EXPECT_TRUE(eios.hasDirectExecutionManager());

	mem.runExecutionCycle();
	EXPECT_TRUE(eios.operateCalled);

	mem.stopManaging(eios);
	EXPECT_FALSE(eios.hasExecutionManager());
	EXPECT_FALSE(eios.hasDirectExecutionManager());
	EXPECT_EQ(NULL, eios.getExecutionManager());

	eios.operateCalled = false;
	mem.runExecutionCycle();
	EXPECT_FALSE(eios.operateCalled);
}

TEST_F(ManualExecutionManagerTest, StopManagingIndirect) {
	ExposedIOSystem<double> eios1;
	ExposedIOSystem<double> eios2;
	ExposedIOSystem<double> eios3;

	systems::connect(eios1.output, eios2.input);
	systems::connect(eios2.output, eios3.input);
	mem.startManaging(eios3);

	EXPECT_TRUE(eios1.hasExecutionManager());
	EXPECT_FALSE(eios1.hasDirectExecutionManager());
	EXPECT_EQ(&mem, eios1.getExecutionManager());

	EXPECT_TRUE(eios2.hasExecutionManager());
	EXPECT_FALSE(eios2.hasDirectExecutionManager());
	EXPECT_EQ(&mem, eios2.getExecutionManager());

	EXPECT_TRUE(eios3.hasExecutionManager());
	EXPECT_TRUE(eios3.hasDirectExecutionManager());
	EXPECT_EQ(&mem, eios3.getExecutionManager());


	mem.stopManaging(eios3);

	EXPECT_FALSE(eios1.hasExecutionManager());
	EXPECT_FALSE(eios1.hasDirectExecutionManager());
	EXPECT_EQ(NULL, eios1.getExecutionManager());

	EXPECT_FALSE(eios2.hasExecutionManager());
	EXPECT_FALSE(eios2.hasDirectExecutionManager());
	EXPECT_EQ(NULL, eios2.getExecutionManager());

	EXPECT_FALSE(eios3.hasExecutionManager());
	EXPECT_FALSE(eios3.hasDirectExecutionManager());
	EXPECT_EQ(NULL, eios3.getExecutionManager());
}

TEST_F(ManualExecutionManagerTest, StopDirectlyManaging) {
	ExposedIOSystem<double> eios1;
	ExposedIOSystem<double> eios2;
	ExposedIOSystem<double> eios3;

	systems::connect(eios1.output, eios2.input);
	systems::connect(eios2.output, eios3.input);


	mem.startManaging(eios3);
	mem.startManaging(eios2);

	EXPECT_TRUE(eios1.hasExecutionManager());
	EXPECT_FALSE(eios1.hasDirectExecutionManager());
	EXPECT_EQ(&mem, eios1.getExecutionManager());

	EXPECT_TRUE(eios2.hasExecutionManager());
	EXPECT_TRUE(eios2.hasDirectExecutionManager());
	EXPECT_EQ(&mem, eios2.getExecutionManager());

	EXPECT_TRUE(eios3.hasExecutionManager());
	EXPECT_TRUE(eios3.hasDirectExecutionManager());
	EXPECT_EQ(&mem, eios3.getExecutionManager());


	mem.stopManaging(eios2);

	EXPECT_TRUE(eios1.hasExecutionManager());
	EXPECT_FALSE(eios1.hasDirectExecutionManager());
	EXPECT_EQ(&mem, eios1.getExecutionManager());

	EXPECT_TRUE(eios2.hasExecutionManager());
	EXPECT_FALSE(eios2.hasDirectExecutionManager());
	EXPECT_EQ(&mem, eios2.getExecutionManager());

	EXPECT_TRUE(eios3.hasExecutionManager());
	EXPECT_TRUE(eios3.hasDirectExecutionManager());
	EXPECT_EQ(&mem, eios3.getExecutionManager());
}

TEST_F(ManualExecutionManagerTest, StopManagingAtEndOfChain) {
	ExposedIOSystem<double> eios1;
	ExposedIOSystem<double> eios2;
	ExposedIOSystem<double> eios3;

	systems::connect(eios1.output, eios2.input);
	systems::connect(eios2.output, eios3.input);


	mem.startManaging(eios3);
	mem.startManaging(eios2);

	EXPECT_TRUE(eios1.hasExecutionManager());
	EXPECT_FALSE(eios1.hasDirectExecutionManager());
	EXPECT_EQ(&mem, eios1.getExecutionManager());

	EXPECT_TRUE(eios2.hasExecutionManager());
	EXPECT_TRUE(eios2.hasDirectExecutionManager());
	EXPECT_EQ(&mem, eios2.getExecutionManager());

	EXPECT_TRUE(eios3.hasExecutionManager());
	EXPECT_TRUE(eios3.hasDirectExecutionManager());
	EXPECT_EQ(&mem, eios3.getExecutionManager());


	mem.stopManaging(eios3);

	EXPECT_TRUE(eios1.hasExecutionManager());
	EXPECT_FALSE(eios1.hasDirectExecutionManager());
	EXPECT_EQ(&mem, eios1.getExecutionManager());

	EXPECT_TRUE(eios2.hasExecutionManager());
	EXPECT_TRUE(eios2.hasDirectExecutionManager());
	EXPECT_EQ(&mem, eios2.getExecutionManager());

	EXPECT_FALSE(eios3.hasExecutionManager());
	EXPECT_FALSE(eios3.hasDirectExecutionManager());
	EXPECT_EQ(NULL, eios3.getExecutionManager());
}

TEST_F(ManualExecutionManagerTest, StopManagingWithNoOutputs) {
	class NoOutputs : public systems::System, public systems::SingleInput<double> {
	public:
		NoOutputs() : systems::SingleInput<double>(this) {}
	protected:
		virtual void operate() {}
	};


	NoOutputs no;
	mem.startManaging(no);

	EXPECT_TRUE(no.hasExecutionManager());
	EXPECT_TRUE(no.hasDirectExecutionManager());
	EXPECT_EQ(&mem, no.getExecutionManager());

	mem.stopManaging(no);

	EXPECT_FALSE(no.hasExecutionManager());
	EXPECT_FALSE(no.hasDirectExecutionManager());
	EXPECT_EQ(NULL, no.getExecutionManager());
}

TEST_F(ManualExecutionManagerTest, ExecutionCycle) {
	EXPECT_TRUE(eios.hasExecutionManager());

	for (int i = 0; i < 10; ++i) {
		eios.operateCalled = false;
		mem.runExecutionCycle();
		EXPECT_TRUE(eios.operateCalled);
	}
}

// See https://www.pivotaltracker.com/story/show/28313197
TEST_F(ManualExecutionManagerTest, UpdateTokensDontCollide) {
	eios.operateCalled = false;
	mem.runExecutionCycle();
	EXPECT_TRUE(eios.operateCalled);

	systems::ManualExecutionManager localMem;
	localMem.startManaging(eios);

	eios.operateCalled = false;
	localMem.runExecutionCycle();
	EXPECT_TRUE(eios.operateCalled);
}


// death tests
typedef ManualExecutionManagerTest ManualExecutionManagerDeathTest;

TEST_F(ManualExecutionManagerDeathTest, StartManagingMixedEm) {
	systems::ManualExecutionManager localMem;
	ExposedIOSystem<double> eios1;
	ExposedIOSystem<double> eios2;

	systems::connect(eios1.output, eios2.input);
	mem.startManaging(eios2);

	EXPECT_DEATH(localMem.startManaging(eios1), "");
}

TEST_F(ManualExecutionManagerDeathTest, StopManagingWhenUnmanaged) {
	ExposedIOSystem<double> eios1;

	EXPECT_DEATH(mem.stopManaging(eios1), "");
}


}
