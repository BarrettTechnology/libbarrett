/*
 * os.cpp
 *
 *  Created on: Jun 7, 2013
 *      Author: dc
 */


#include <boost/thread.hpp>

#include <gtest/gtest.h>
#include <barrett/os.h>


namespace {
using namespace barrett;


TEST(HighResolutionSystemTimeTest, AgreesWithBoostThreadSleep) {
	for (int i = 1; i <= 10; ++i) {
		double duration = i * 0.01;

		double before = highResolutionSystemTime();
		boost::this_thread::sleep(boost::posix_time::microseconds(long(duration * 1e6)));
		double after = highResolutionSystemTime();

		// Because this test relies on the and OS environment and scheduler,
		// occasional failures are expected :(
		ASSERT_NEAR(duration, after - before, 0.001);
	}
}


}
