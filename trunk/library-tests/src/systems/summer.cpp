/*
 * summer.cpp
 *
 *  Created on: Oct 16, 2009
 *      Author: dc
 */

#include <gtest/gtest.h>
#include <barrett/systems.h>


namespace {


// TODO(dc): actually test this
class SummerTest : public ::testing::Test {
protected:
	Systems::Summer<double> summer;
};


TEST_F(SummerTest, ) {
	summer.getInput(0);
}


}
