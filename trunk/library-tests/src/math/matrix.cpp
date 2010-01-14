/*
 * matrix.cpp
 *
 *  Created on: Jan 13, 2010
 *      Author: dc
 */


#include <gtest/gtest.h>
#include <barrett/math/matrix.h>


namespace {
using namespace barrett;

// TODO(dc): actually test this

TEST(MatrixTest, DefaultCtor) {
	math::Matrix<2,3> m;

	EXPECT_EQ(0, m(0,0));
}


}
