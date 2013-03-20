/*
 * real_time_writer.cpp
 *
 *  Created on: Dec 30, 2009
 *      Author: dc
 */


#include <cstdio>
#include <stdexcept>

#include <gtest/gtest.h>
#include <barrett/os.h>
#include <barrett/log/real_time_writer.h>
#include "./verify_file_contents.h"


namespace {
using namespace barrett;


struct BigLogTraits {
	typedef double parameter_type;
	typedef double* pointer_type;

	static size_t serializedLength() {
		return 1025;
	}

	static void serialize(parameter_type source, pointer_type dest) {
		*dest = source;
	}
};

// macros don't like the comma outside of parenthesis
typedef log::RealTimeWriter<double, BigLogTraits> big_log_t;


void fillLogVerify(size_t n, size_t period) {
	char tmpFile[] = "/tmp/btXXXXXX";
	ASSERT_TRUE(mkstemp(tmpFile) != -1);

	double* ds = new double[n];
	log::RealTimeWriter<double> lw(tmpFile, 0.01, 100);

	for (size_t i = 0; i < n; ++i) {
		ds[i] = i*1103.58 - 7e6;
		lw.putRecord(ds[i]);

		if (period != 0) {
			btsleep(period*0.000001);
		}
	}
	lw.close();

	verifyFileContents(tmpFile, reinterpret_cast<char*>(ds), sizeof(double[n]));

	delete[] ds;
	std::remove(tmpFile);
}


TEST(RealTimeLogWriterTest, RecordRateCtorThrows) {
	char tmpFile[] = "/tmp/btXXXXXX";
	ASSERT_TRUE(mkstemp(tmpFile) != -1);

	EXPECT_THROW(big_log_t lw1(tmpFile, 500.0), std::logic_error);  // too big
	EXPECT_THROW(log::RealTimeWriter<double> lw2(tmpFile, 3.6e-6), std::logic_error);  // too fast

	std::remove(tmpFile);
}

TEST(RealTimeLogWriterTest, SmallSlow) {
	fillLogVerify(55, 500);
}

TEST(RealTimeLogWriterTest, SmallFast) {
	// the data set size is smaller than the buffer, so no overflow
	fillLogVerify(55, 0);
}

TEST(RealTimeLogWriterTest, NormalSlow) {
	fillLogVerify(555, 500);
}

TEST(RealTimeLogWriterTest, NormalFast) {
	// don't worry too much if this fails to throw something
	EXPECT_THROW(fillLogVerify(555, 0), std::overflow_error);
}

TEST(RealTimeLogWriterTest, BigSlow) {
	// this is faster than the other slow tests because it took too long to run
	fillLogVerify(5555, 250);
}

TEST(RealTimeLogWriterTest, BigFast) {
	// don't worry too much if this fails to throw something
	EXPECT_THROW(fillLogVerify(5555, 0), std::overflow_error);
}


}
