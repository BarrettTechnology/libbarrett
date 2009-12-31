/*
 * reader.cpp
 *
 *  Created on: Dec 31, 2009
 *      Author: dc
 */


#include <stdexcept>
#include <cstdio>

#include <gtest/gtest.h>

#include <barrett/units.h>
#include <barrett/log/reader.h>
#include <barrett/log/writer.h>


namespace {
using namespace barrett;


TEST(LogReaderTest, Double) {
	char tmpFile[L_tmpnam];
	ASSERT_TRUE(std::tmpnam(tmpFile) != NULL);

	double d = -23425.0000001;

	log::Writer<double> lw(tmpFile);
	lw.putRecord(d);
	lw.close();

	log::Reader<double> lr(tmpFile);
	EXPECT_EQ(1, lr.numRecords());
	EXPECT_EQ(d, lr.getRecord());
	EXPECT_THROW(lr.getRecord(), std::underflow_error);
	lr.close();

	std::remove(tmpFile);
}

TEST(LogReaderTest, Array) {
	char tmpFile[L_tmpnam];
	ASSERT_TRUE(std::tmpnam(tmpFile) != NULL);

	units::Array<15> d;
	d <<	23,	54,		34,		4,		25,
			23,	6,		46,		23,		-6,
			11,	868,	12312,	-44.2,	1;

	log::Writer<units::Array<15> > lw(tmpFile);
	lw.putRecord(d);
	lw.close();

	log::Reader<units::Array<15> > lr(tmpFile);
	EXPECT_EQ(1, lr.numRecords());
	EXPECT_EQ(d, lr.getRecord());
	EXPECT_THROW(lr.getRecord(), std::underflow_error);
	lr.close();

	std::remove(tmpFile);
}

TEST(LogReaderTest, SeveralRecords) {
	char tmpFile[L_tmpnam];
	ASSERT_TRUE(std::tmpnam(tmpFile) != NULL);

	double ds[] = {3e7, -12, 432, 8.888};
	size_t n = sizeof(ds)/sizeof(double);

	log::Writer<double> lw(tmpFile);
	for (size_t i = 0; i < n; ++i) {
		lw.putRecord(ds[i]);
	}
	lw.close();

	log::Reader<double> lr(tmpFile);
	EXPECT_EQ(n, lr.numRecords());
	for (size_t i = 0; i < n; ++i) {
		EXPECT_EQ(ds[i], lr.getRecord());
	}
	EXPECT_THROW(lr.getRecord(), std::underflow_error);
	lr.close();

	std::remove(tmpFile);
}


}
