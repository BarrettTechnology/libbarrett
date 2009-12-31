/*
 * writer.cpp
 *
 *  Created on: Dec 29, 2009
 *      Author: dc
 */


#include <cstdio>

#include <gtest/gtest.h>
#include <barrett/units.h>
#include <barrett/log/writer.h>
#include "./verify_file_contents.h"


namespace {
using namespace barrett;


TEST(LogWriterTest, Double) {
	char tmpFile[L_tmpnam];
	ASSERT_TRUE(std::tmpnam(tmpFile) != NULL);

	double d = 345823230823.2222;

	log::Writer<double> lw(tmpFile);
	lw.putRecord(d);
	lw.close();

	verifyFileContents(tmpFile, reinterpret_cast<char*>(&d), sizeof(double));
	std::remove(tmpFile);
}

TEST(LogWriterTest, Array) {
	char tmpFile[L_tmpnam];
	ASSERT_TRUE(std::tmpnam(tmpFile) != NULL);

	units::Array<15> d;
	d <<	23,	54,		34,		4,		25,
			23,	6,		46,		23,		-6,
			11,	868,	12312,	-44.2,	1;

	log::Writer<units::Array<15> > lw(tmpFile);
	lw.putRecord(d);
	lw.close();

	verifyFileContents(tmpFile, reinterpret_cast<char*>(d.c_array()), sizeof(double) * units::Array<15>::SIZE);
	std::remove(tmpFile);
}

TEST(LogWriterTest, SeveralRecords) {
	char tmpFile[L_tmpnam];
	ASSERT_TRUE(std::tmpnam(tmpFile) != NULL);

	double ds[] = {3e7, -12, 432, 8.888};

	log::Writer<double> lw(tmpFile);
	lw.putRecord(ds[0]);
	lw.putRecord(ds[1]);
	lw.putRecord(ds[2]);
	lw.putRecord(ds[3]);
	lw.close();

	verifyFileContents(tmpFile, reinterpret_cast<char*>(ds), sizeof(ds));
	std::remove(tmpFile);
}


}
