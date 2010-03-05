/*
 * io_conversion.cpp
 *
 *  Created on: Oct 30, 2009
 *      Author: dc
 */


#include <gtest/gtest.h>

#include <barrett/units.h>
#include <barrett/systems/abstract/conversion.h>
#include <barrett/systems/io_conversion.h>

#include "./exposed_io_system.h"


namespace {
using namespace barrett;


typedef units::JointTorques<3>::type jt_type;


// we just want this to compile
TEST(IOConversionTest, Ctor) {
	ExposedIOSystem<jt_type> eios;
	systems::Conversion<jt_type>* conversion =
			systems::makeIOConversion(eios.input, eios.output);
	delete conversion;
}


}
