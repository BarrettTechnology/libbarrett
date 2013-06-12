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


void verifySleepDurations(void (*sleepFunction)(double)) {
	for (int i = 1; i <= 10; ++i) {
		double duration = i * 0.01;

		double before = highResolutionSystemTime();
		sleepFunction(duration);
		double after = highResolutionSystemTime();

		ASSERT_NEAR(duration, after - before, 0.001)
			<< "Because this test relies on the Linux scheduler, occasional failures are expected.";
	}
}

void boostSleep(double duration) {
	boost::this_thread::sleep(boost::posix_time::microseconds(long(duration * 1e6)));
}

TEST(HighResolutionSystemTimeTest, AgreesWithBoostThreadSleep) {
	verifySleepDurations(&boostSleep);
}

TEST(BtsleepTest, AgreesWithHRST) {
	verifySleepDurations(&btsleep);
}


}
