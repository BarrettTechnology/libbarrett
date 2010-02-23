/*
 * summer.cpp
 *
 *  Created on: Oct 16, 2009
 *      Author: dc
 */

#include <gtest/gtest.h>
#include <barrett/systems.h>


namespace {
using namespace barrett;


// TODO(dc): actually test this
class SummerTest : public ::testing::Test {
protected:
	systems::Summer<double> summer;
};


TEST_F(SummerTest, ) {
	summer.getInput(0);
}


}
