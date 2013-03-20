/*
 * puck.cpp
 *
 *  Created on: Feb 17, 2012
 *      Author: dc
 */


#include <stdexcept>
#include <gtest/gtest.h>
#include <barrett/products/puck.h>


namespace {
using namespace barrett;


TEST(PuckTest, GetPropertyStrTest) {
	EXPECT_STREQ("A", Puck::getPropertyStr(Puck::A));
	EXPECT_STREQ("DIG1", Puck::getPropertyStr(Puck::DIG1));
	EXPECT_STREQ("FET0", Puck::getPropertyStr(Puck::FET0));
	EXPECT_STREQ("GRPA", Puck::getPropertyStr(Puck::GRPA));
	EXPECT_STREQ("IFAULT", Puck::getPropertyStr(Puck::IFAULT));
	EXPECT_STREQ("IPNM", Puck::getPropertyStr(Puck::IPNM));
	EXPECT_STREQ("JP", Puck::getPropertyStr(Puck::JP));
	EXPECT_STREQ("LSG", Puck::getPropertyStr(Puck::LSG));
	EXPECT_STREQ("MODE", Puck::getPropertyStr(Puck::MODE));
	EXPECT_STREQ("PIDX", Puck::getPropertyStr(Puck::PIDX));
	EXPECT_STREQ("STAT", Puck::getPropertyStr(Puck::STAT));
	EXPECT_STREQ("THERM", Puck::getPropertyStr(Puck::THERM));
	EXPECT_STREQ("VERS", Puck::getPropertyStr(Puck::VERS));
	EXPECT_STREQ("ZERO", Puck::getPropertyStr(Puck::ZERO));
}

TEST(PuckTest, GetPropertyEnumTestNoThrow) {
	for (int i = 0; i < Puck::NUM_PROPERTIES; ++i) {
		EXPECT_EQ(i, Puck::getPropertyEnumNoThrow(Puck::getPropertyStr((enum Puck::Property) i)));
	}

	EXPECT_EQ(Puck::ZERO, Puck::getPropertyEnumNoThrow("ZERO"));
	EXPECT_EQ(Puck::ZERO, Puck::getPropertyEnumNoThrow("zero"));
	EXPECT_EQ(Puck::ZERO, Puck::getPropertyEnumNoThrow("zeRO"));
	EXPECT_EQ(Puck::ZERO, Puck::getPropertyEnumNoThrow("ZerO"));

	EXPECT_EQ(-1, Puck::getPropertyEnumNoThrow("ZEROS"));
	EXPECT_EQ(-1, Puck::getPropertyEnumNoThrow("ZER"));
	EXPECT_EQ(-1, Puck::getPropertyEnumNoThrow("ZERO\n"));
	EXPECT_EQ(-1, Puck::getPropertyEnumNoThrow(" ZERO"));
	EXPECT_EQ(-1, Puck::getPropertyEnumNoThrow(""));
	EXPECT_EQ(-1, Puck::getPropertyEnumNoThrow("omgthispropertynameisreallyreallylong"));
}

TEST(PuckTest, GetPropertyEnumTest) {
	for (int i = 0; i < Puck::NUM_PROPERTIES; ++i) {
		EXPECT_EQ(i, Puck::getPropertyEnum(Puck::getPropertyStr((enum Puck::Property) i)));
	}

	EXPECT_EQ(Puck::ZERO, Puck::getPropertyEnum("ZERO"));
	EXPECT_EQ(Puck::ZERO, Puck::getPropertyEnum("zero"));
	EXPECT_EQ(Puck::ZERO, Puck::getPropertyEnum("zeRO"));
	EXPECT_EQ(Puck::ZERO, Puck::getPropertyEnum("ZerO"));

	EXPECT_THROW(Puck::getPropertyEnum("ZEROS"), std::invalid_argument);
	EXPECT_THROW(Puck::getPropertyEnum("ZER"), std::invalid_argument);
	EXPECT_THROW(Puck::getPropertyEnum("ZERO\n"), std::invalid_argument);
	EXPECT_THROW(Puck::getPropertyEnum(" ZERO"), std::invalid_argument);
	EXPECT_THROW(Puck::getPropertyEnum(""), std::invalid_argument);
	EXPECT_THROW(Puck::getPropertyEnum("omgthispropertynameisreallyreallylong"), std::invalid_argument);
}


}
