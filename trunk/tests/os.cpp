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

		EXPECT_NEAR(duration, after - before, 0.001)
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


TEST(PeriodicLoopTimerTest, LoopRateIsCorrect) {
	const int LOOP_COUNT = 10;

	for (double period = 0.05; period <= 0.10; period += 0.025) {
		PeriodicLoopTimer plt(period);
		plt.wait();  // There might be first-run timing effects. These are not important.

		double before = highResolutionSystemTime();
		for (int i = 0; i < LOOP_COUNT; ++i) {
			ASSERT_EQ(0, plt.wait());
		}
		double after = highResolutionSystemTime();

		// Average jitter should be small compared to a 1kHz loop rate.
		ASSERT_NEAR(period, (after - before) / LOOP_COUNT, 0.00001);
	}
}

TEST(PeriodicLoopTimerTest, CountsMissedRelesePoints) {
	const double PERIOD = 0.05;
	PeriodicLoopTimer plt(PERIOD);

	for (int i = 0; i < 5; ++i) {
		EXPECT_EQ(i, plt.wait()) << "This test is known to fail under Xenomai.";
		btsleep(PERIOD * (i + 1.5));
	}
}

}
