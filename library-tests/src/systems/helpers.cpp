/*
 * helpers.cpp
 *
 *  Created on: Sep 18, 2009
 *      Author: dc
 */


#include <stdexcept>
#include <gtest/gtest.h>
#include <barrett/systems.h>
#include "./exposed_io_system.h"

namespace {


using Systems::checkConnected;
using Systems::checkNotConnected;
using Systems::checkDisconnected;


// TODO(dc): make this a type-parameterized test case so users can test their
// own code
class SystemHelperTest : public ::testing::Test {
protected:
	Systems::ExposedIO<double> eios;
};


// Systems::connect
TEST_F(SystemHelperTest, ConnectConnects) {
	Systems::connect(eios.output, eios.input);
	checkConnected(&eios, eios, 5.7);
}

TEST_F(SystemHelperTest, ConnectDoesntConnectTwice) {
	Systems::ExposedIO<double> eios2;

	ASSERT_NO_THROW(Systems::connect(eios.output, eios.input))
		<< "connect() threw on first connection";
	ASSERT_THROW(Systems::connect(eios2.output, eios.input), std::invalid_argument)
		<< "connect() didn't throw on second connection";
}


// Systems::reconnect
TEST_F(SystemHelperTest, ReconnectReconnects) {
	Systems::ExposedIO<double> eios2;

	Systems::connect(eios.output, eios.input);
	checkConnected(&eios, eios, -8.6e4);

	Systems::reconnect(eios2.output, eios.input);
	checkNotConnected(&eios, eios, -34.8);
	checkConnected(&eios2, eios, 712.0);
}

TEST_F(SystemHelperTest, ReconnectThrowsIfInputNotConnected) {
	ASSERT_THROW(Systems::reconnect(eios.output, eios.input),
			std::invalid_argument)
		<< "reconnect() didn't throw when passed a disconnected input";
}


// Systems::forceConnect
TEST_F(SystemHelperTest, ForceConnectConnects) {
	Systems::forceConnect(eios.output, eios.input);
	checkConnected(&eios, eios, 5.7);
}

TEST_F(SystemHelperTest, ForceConnectReconnects) {
	Systems::ExposedIO<double> eios2;

	Systems::connect(eios.output, eios.input);
	checkConnected(&eios, eios, -8.6e4);

	Systems::forceConnect(eios2.output, eios.input);
	checkNotConnected(&eios, eios, -34.8);
	checkConnected(&eios2, eios, 712.0);
}


// Systems::disconnect
TEST_F(SystemHelperTest, DisconnectDisconnects) {
	Systems::connect(eios.output, eios.input);
	checkConnected(&eios, eios, 42.0);

	ASSERT_NO_THROW(Systems::disconnect(eios.input))
		<< "disconnect() threw when passed a connected input";
	checkNotConnected(&eios, eios, 5.0);
	checkDisconnected(eios);
}

TEST_F(SystemHelperTest, DisconnectThrowsIfInputNotConnected) {
	ASSERT_THROW(Systems::disconnect(eios.input), std::invalid_argument)
		<< "disconnect() didn't throw when passed an already disconnected input";
}


}
