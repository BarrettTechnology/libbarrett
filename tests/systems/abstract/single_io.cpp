/*
 * single_io.cpp
 *
 *  Created on: Sep 27, 2009
 *      Author: dc
 */


#include <gtest/gtest.h>
#include "../exposed_io_system.h"


namespace {
using namespace barrett;


TEST(SingleIOTest, DefaultCtor) {
	ExposedIOSystem<double> eios;
	checkDisconnected(eios);
}


}
