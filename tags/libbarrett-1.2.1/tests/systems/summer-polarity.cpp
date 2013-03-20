/*
 * summer-polarity.cpp
 *
 *  Created on: Mar 29, 2010
 *      Author: dc
 */

#include <stdexcept>
#include <bitset>

#include <gtest/gtest.h>
#include <barrett/systems/summer.h>


namespace {
using namespace barrett;


TEST(SummerPolarityTest, DefaultCtor) {
	{
		systems::Summer<double>::Polarity p;

		size_t i = 0;
		EXPECT_EQ(1, p[i++]);
		EXPECT_EQ(1, p[i++]);
	}
	{
		systems::Summer<double, 5>::Polarity p;

		size_t i = 0;
		EXPECT_EQ(1, p[i++]);
		EXPECT_EQ(1, p[i++]);
		EXPECT_EQ(1, p[i++]);
		EXPECT_EQ(1, p[i++]);
		EXPECT_EQ(1, p[i++]);
	}
}

TEST(SummerPolarityTest, StringCtor) {
	{
		systems::Summer<double>::Polarity p("--");

		size_t i = 0;
		EXPECT_EQ(-1, p[i++]);
		EXPECT_EQ(-1, p[i++]);
	}
	{
		systems::Summer<double>::Polarity p("+-");

		size_t i = 0;
		EXPECT_EQ(1, p[i++]);
		EXPECT_EQ(-1, p[i++]);
	}

	{
		systems::Summer<double, 5>::Polarity p("+--+-");

		size_t i = 0;
		EXPECT_EQ(1, p[i++]);
		EXPECT_EQ(-1, p[i++]);
		EXPECT_EQ(-1, p[i++]);
		EXPECT_EQ(1, p[i++]);
		EXPECT_EQ(-1, p[i++]);
	}
	{
		systems::Summer<double, 5>::Polarity p("-+---");

		size_t i = 0;
		EXPECT_EQ(-1, p[i++]);
		EXPECT_EQ(1, p[i++]);
		EXPECT_EQ(-1, p[i++]);
		EXPECT_EQ(-1, p[i++]);
		EXPECT_EQ(-1, p[i++]);
	}
}

TEST(SummerPolarityTest, StringCtorThrows) {
	EXPECT_THROW(systems::Summer<double>::Polarity p("-+-"), std::invalid_argument);
	EXPECT_THROW(systems::Summer<double>::Polarity p("-a"), std::invalid_argument);
	EXPECT_THROW(systems::Summer<double>::Polarity p("-"), std::invalid_argument);

	typedef systems::Summer<double, 5>::Polarity polarity_type;  // no commas allowed in a macro parameter...
	EXPECT_THROW(polarity_type p("-++-+-"), std::invalid_argument);
	EXPECT_THROW(polarity_type p("--+q+"), std::invalid_argument);
	EXPECT_THROW(polarity_type p("-+++"), std::invalid_argument);
}

TEST(SummerPolarityTest, BitsetCtor) {
	{
		size_t i = 0;
		std::bitset<2> b;
		b[i++] = false;
		b[i++] = true;

		systems::Summer<double>::Polarity p(b);

		i = 0;
		EXPECT_EQ(-1, p[i++]);
		EXPECT_EQ(1, p[i++]);
	}
	{
		size_t i = 0;
		std::bitset<5> b;
		b[i++] = false;
		b[i++] = false;
		b[i++] = false;
		b[i++] = true;
		b[i++] = true;

		systems::Summer<double, 5>::Polarity p(b);

		i = 0;
		EXPECT_EQ(-1, p[i++]);
		EXPECT_EQ(-1, p[i++]);
		EXPECT_EQ(-1, p[i++]);
		EXPECT_EQ(1, p[i++]);
		EXPECT_EQ(1, p[i++]);
	}
}


}

