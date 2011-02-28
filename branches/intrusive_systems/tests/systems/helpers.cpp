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


// systems::connect
TEST_F(SystemHelperTest, ConnectConnects) {
	systems::connect(out.output, in.input);
	checkConnected(&out, in, 5.7);
}

TEST_F(SystemHelperTest, ConnectDoesntConnectTwice) {
	ASSERT_NO_THROW(systems::connect(out.output, in.input))
		<< "connect() threw on first connection";
	ASSERT_THROW(systems::connect(in.output, in.input), std::invalid_argument)
		<< "connect() didn't throw on second connection";
}


// systems::reconnect
TEST_F(SystemHelperTest, ReconnectReconnects) {
	systems::connect(out.output, in.input);
	checkConnected(&out, in, -8.6e4);

	systems::reconnect(other.output, in.input);
	checkNotConnected(&out, in, -34.8);
	checkConnected(&other, in, 712.0);
}

TEST_F(SystemHelperTest, ReconnectThrowsIfInputNotConnected) {
	ASSERT_THROW(systems::reconnect(out.output, in.input),
			std::invalid_argument)
		<< "reconnect() didn't throw when passed a disconnected input";
}


// systems::forceConnect
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


// systems::disconnect
TEST_F(SystemHelperTest, DisconnectDisconnects) {
	systems::connect(out.output, in.input);
	checkConnected(&out, in, 42.0);

	ASSERT_NO_THROW(systems::disconnect(in.input))
		<< "disconnect() threw when passed a connected input";
	checkNotConnected(&out, in, 5.0);
	checkDisconnected(in);
}


}
