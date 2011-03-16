/*
 * helpers.cpp
 *
 *  Created on: Sep 18, 2009
 *      Author: dc
 */


#include <stdexcept>
#include <gtest/gtest.h>
#include <barrett/systems/manual_execution_manager.h>
#include <barrett/systems/helpers.h>
#include "./exposed_io_system.h"

namespace {
using namespace barrett;

class SystemHelperTest : public ::testing::Test {
public:
	SystemHelperTest() {
		mem.startManaging(in);
	}

protected:
	systems::ManualExecutionManager mem;
	ExposedIOSystem<double> out;
	ExposedIOSystem<double> in;
	ExposedIOSystem<double> other;
};


TEST_F(SystemHelperTest, ConnectConnects) {
	systems::connect(out.output, in.input);
	checkConnected(mem, &out, in, 5.7);
}

TEST_F(SystemHelperTest, ConnectPropagatesEm) {
	EXPECT_FALSE(out.hasExecutionManager());
	EXPECT_FALSE(out.hasDirectExecutionManager());
	EXPECT_EQ(NULL, out.getExecutionManager());

	systems::connect(out.output, in.input);

	EXPECT_TRUE(out.hasExecutionManager());
	EXPECT_FALSE(out.hasDirectExecutionManager());
	EXPECT_EQ(&mem, out.getExecutionManager());
}

TEST_F(SystemHelperTest, ConnectAllowsInputWithNullEm) {
	mem.startManaging(out);

	EXPECT_FALSE(other.hasExecutionManager());
	EXPECT_FALSE(other.hasDirectExecutionManager());
	EXPECT_EQ(NULL, other.getExecutionManager());

	systems::connect(out.output, other.input);

	EXPECT_FALSE(other.hasExecutionManager());
	EXPECT_FALSE(other.hasDirectExecutionManager());
	EXPECT_EQ(NULL, other.getExecutionManager());

}

TEST_F(SystemHelperTest, ReconnectReconnects) {
	systems::connect(out.output, in.input);
	checkConnected(mem, &out, in, -8.6e4);

	systems::reconnect(other.output, in.input);
	checkNotConnected(mem, &out, in, -34.8);
	checkConnected(mem, &other, in, 712.0);
}

TEST_F(SystemHelperTest, ReconnectPropagatesEm) {
	EXPECT_FALSE(out.hasExecutionManager());
	EXPECT_FALSE(out.hasDirectExecutionManager());
	EXPECT_EQ(NULL, out.getExecutionManager());

	EXPECT_FALSE(other.hasExecutionManager());
	EXPECT_FALSE(other.hasDirectExecutionManager());
	EXPECT_EQ(NULL, other.getExecutionManager());

	systems::connect(out.output, in.input);
	systems::reconnect(other.output, in.input);

	EXPECT_FALSE(out.hasExecutionManager());
	EXPECT_FALSE(out.hasDirectExecutionManager());
	EXPECT_EQ(NULL, out.getExecutionManager());

	EXPECT_TRUE(other.hasExecutionManager());
	EXPECT_FALSE(other.hasDirectExecutionManager());
	EXPECT_EQ(&mem, other.getExecutionManager());
}

TEST_F(SystemHelperTest, ForceConnectConnects) {
	systems::forceConnect(out.output, in.input);
	checkConnected(mem, &out, in, 5.7);
}

TEST_F(SystemHelperTest, ForceConnectPropagatesEmOnConnect) {
	EXPECT_FALSE(out.hasExecutionManager());
	EXPECT_FALSE(out.hasDirectExecutionManager());
	EXPECT_EQ(NULL, out.getExecutionManager());

	systems::forceConnect(out.output, in.input);

	EXPECT_TRUE(out.hasExecutionManager());
	EXPECT_FALSE(out.hasDirectExecutionManager());
	EXPECT_EQ(&mem, out.getExecutionManager());
}

TEST_F(SystemHelperTest, ForceConnectReconnects) {
	systems::connect(out.output, in.input);
	checkConnected(mem, &out, in, -8.6e4);

	systems::forceConnect(other.output, in.input);
	checkNotConnected(mem, &out, in, -34.8);
	checkConnected(mem, &other, in, 712.0);
}

TEST_F(SystemHelperTest, ForceConnectPropagatesEmOnReconnect) {
	EXPECT_FALSE(out.hasExecutionManager());
	EXPECT_FALSE(out.hasDirectExecutionManager());
	EXPECT_EQ(NULL, out.getExecutionManager());

	EXPECT_FALSE(other.hasExecutionManager());
	EXPECT_FALSE(other.hasDirectExecutionManager());
	EXPECT_EQ(NULL, other.getExecutionManager());

	systems::connect(out.output, in.input);
	systems::forceConnect(other.output, in.input);

	EXPECT_FALSE(out.hasExecutionManager());
	EXPECT_FALSE(out.hasDirectExecutionManager());
	EXPECT_EQ(NULL, out.getExecutionManager());

	EXPECT_TRUE(other.hasExecutionManager());
	EXPECT_FALSE(other.hasDirectExecutionManager());
	EXPECT_EQ(&mem, other.getExecutionManager());
}

TEST_F(SystemHelperTest, DisconnectInputDisconnects) {
	systems::connect(out.output, in.input);
	checkConnected(mem, &out, in, 42.0);

	systems::disconnect(in.input);

	checkNotConnected(mem, &out, in, 5.0);
	checkDisconnected(in);
}

TEST_F(SystemHelperTest, DisconnectInputPropagatesEm) {
	EXPECT_FALSE(out.hasExecutionManager());
	EXPECT_FALSE(out.hasDirectExecutionManager());
	EXPECT_EQ(NULL, out.getExecutionManager());

	systems::connect(out.output, in.input);

	EXPECT_TRUE(out.hasExecutionManager());
	EXPECT_FALSE(out.hasDirectExecutionManager());
	EXPECT_EQ(&mem, out.getExecutionManager());

	systems::disconnect(in.input);

	EXPECT_FALSE(out.hasExecutionManager());
	EXPECT_FALSE(out.hasDirectExecutionManager());
	EXPECT_EQ(NULL, out.getExecutionManager());
}

TEST_F(SystemHelperTest, DisconnectOutputDisconnects) {
	systems::connect(out.output, in.input);
	checkConnected(mem, &out, in, 42.0);

	mem.startManaging(other);
	systems::connect(out.output, other.input);
	checkConnected(mem, &out, other, 42.0);

	systems::disconnect(out.output);

	checkNotConnected(mem, &out, in, 5.0);
	checkDisconnected(in);

	checkNotConnected(mem, &out, other, 5.0);
	checkDisconnected(other);
}

TEST_F(SystemHelperTest, DisconnectOutputPropagatesEm) {
	EXPECT_FALSE(out.hasExecutionManager());
	EXPECT_FALSE(out.hasDirectExecutionManager());
	EXPECT_EQ(NULL, out.getExecutionManager());

	mem.startManaging(other);
	systems::connect(out.output, in.input);
	systems::connect(out.output, other.input);

	EXPECT_TRUE(out.hasExecutionManager());
	EXPECT_FALSE(out.hasDirectExecutionManager());
	EXPECT_EQ(&mem, out.getExecutionManager());

	systems::disconnect(out.output);

	EXPECT_FALSE(out.hasExecutionManager());
	EXPECT_FALSE(out.hasDirectExecutionManager());
	EXPECT_EQ(NULL, out.getExecutionManager());
}


// death tests
typedef SystemHelperTest SystemHelperDeathTest;

TEST_F(SystemHelperDeathTest, ConnectDoesntConnectTwice) {
	systems::connect(out.output, in.input);
	ASSERT_DEATH(systems::connect(in.output, in.input), "");
}

TEST_F(SystemHelperDeathTest, ReconnectDiesIfInputNotConnected) {
	ASSERT_DEATH(systems::reconnect(out.output, in.input), "");
}

TEST_F(SystemHelperDeathTest, NoMixedEm) {
	systems::ManualExecutionManager localMem;
	localMem.startManaging(out);

	EXPECT_DEATH(systems::connect(out.output, in.input), "");

	systems::connect(other.output, in.input);
	EXPECT_DEATH(systems::reconnect(out.output, in.input), "");
}


}
