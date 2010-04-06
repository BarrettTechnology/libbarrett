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
#include <barrett/systems/converter.h>

#include "./abstract/conversion_impl.h"
#include "./exposed_io_system.h"


namespace {
using namespace barrett;


typedef units::JointTorques<12>::type jt_type;


class ConverterTest : public ::testing::Test {
protected:
	systems::Converter<jt_type> sc;
};

// TODO(dc): actually test this
TEST_F(ConverterTest, DefaultCtor) {

}

// we just want this to compile
TEST_F(ConverterTest, RegisterConversion) {
	// register conversions of different types
	sc.registerConversion(
			new ConversionImpl<jt_type, jt_type>());
	sc.registerConversion(
		new ConversionImpl<units::JointPositions<12>::type, jt_type>());
}

TEST_F(ConverterTest, ConnectInputToExplicit) {
	ExposedIOSystem<jt_type> eios;

	systems::connect(sc.output, eios.input);
	EXPECT_TRUE(sc.connectInputTo(eios.output,
			new ConversionImpl<jt_type, jt_type>()));

	jt_type jt(jt_type::Random());
	checkConnected(&eios, eios, jt);

	// can't be connected because output and conversion input are
	// different types
	EXPECT_FALSE(sc.connectInputTo(eios.output,
		new ConversionImpl<units::JointPositions<12>::type, jt_type>()));
}

TEST_F(ConverterTest, ConnectInputToAutomatic) {
	ExposedIOSystem<jt_type> eios;
	systems::connect(sc.output, eios.input);

	// no conversions registered
	EXPECT_THROW(sc.connectInputTo(eios.output), std::invalid_argument);

	// wrong kind of conversion registered
	sc.registerConversion(
		new ConversionImpl<units::JointPositions<12>::type, jt_type>());
	EXPECT_THROW(sc.connectInputTo(eios.output), std::invalid_argument);

	// compatible conversion registered
	sc.registerConversion(
			new ConversionImpl<jt_type, jt_type>());
	EXPECT_NO_THROW(sc.connectInputTo(eios.output));

	jt_type jt(jt_type::Random());
	checkConnected(&eios, eios, jt);
}

TEST_F(ConverterTest, DisconnectInput) {
	ExposedIOSystem<jt_type> eios;
	systems::connect(sc.output, eios.input);

	// compatible conversion registered
	sc.registerConversion(
			new ConversionImpl<jt_type, jt_type>());
	EXPECT_NO_THROW(sc.connectInputTo(eios.output));

	sc.disconnectInput();
	checkDisconnected(eios);
}


}
