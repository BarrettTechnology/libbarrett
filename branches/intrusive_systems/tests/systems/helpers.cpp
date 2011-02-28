/*
 * helpers.cpp
 *
 *  Created on: Sep 18, 2009
 *      Author: dc
 */


#include <stdexcept>
#include <gtest/gtest.h>
#include <barrett/systems/helpers.h>
#include "./exposed_io_system.h"

namespace {
using namespace barrett;


// TODO(dc): make this a type-parameterized test case so users can test their
// own code
class SystemHelperTest : public ::testing::Test {
protected:
	ExposedIOSystem<double> out;
	ExposedIOSystem<double> in;
	ExposedIOSystem<double> other;
};


TEST_F(SystemHelperTest, ConnectConnects) {
	systems::connect(out.output, in.input);
	checkConnected(&out, in, 5.7);
}

TEST_F(SystemHelperTest, ReconnectReconnects) {
	systems::connect(out.output, in.input);
	checkConnected(&out, in, -8.6e4);

	systems::reconnect(other.output, in.input);
	checkNotConnected(&out, in, -34.8);
	checkConnected(&other, in, 712.0);
}

TEST_F(SystemHelperTest, ForceConnectConnects) {
	systems::forceConnect(out.output, in.input);
	checkConnected(&out, in, 5.7);
}

TEST_F(SystemHelperTest, ForceConnectReconnects) {
	systems::connect(out.output, in.input);
	checkConnected(&out, in, -8.6e4);

	systems::forceConnect(other.output, in.input);
	checkNotConnected(&out, in, -34.8);
	checkConnected(&other, in, 712.0);
}

TEST_F(SystemHelperTest, DisconnectDisconnects) {
	systems::connect(out.output, in.input);
	checkConnected(&out, in, 42.0);

	ASSERT_NO_THROW(systems::disconnect(in.input))
		<< "disconnect() threw when passed a connected input";
	checkNotConnected(&out, in, 5.0);
	checkDisconnected(in);
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


}
