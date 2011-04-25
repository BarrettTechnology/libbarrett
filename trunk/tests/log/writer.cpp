/*
 * writer.cpp
 *
 *  Created on: Dec 29, 2009
 *      Author: dc
 */


#include <cstdio>

#include <gtest/gtest.h>
#include <barrett/math/matrix.h>
#include <barrett/log/writer.h>
#include "./verify_file_contents.h"


namespace {
using namespace barrett;


TEST(LogWriterTest, CtorThrows) {
	// TODO(dc): test this!
}

TEST(LogWriterTest, Double) {
	char tmpFile[] = "/tmp/btXXXXXX";
	ASSERT_TRUE(mkstemp(tmpFile) != -1);

	double d = 345823230823.2222;

	log::Writer<double> lw(tmpFile);
	lw.putRecord(d);
	lw.close();

	verifyFileContents(tmpFile, reinterpret_cast<char*>(&d), sizeof(double));
	std::remove(tmpFile);
}

TEST(LogWriterTest, Tuple) {
	typedef boost::tuple<double, double> tuple_type;

	char tmpFile[] = "/tmp/btXXXXXX";
	ASSERT_TRUE(mkstemp(tmpFile) != -1);

	tuple_type d(-287.2, 8e3);

	log::Writer<tuple_type> lw(tmpFile);
	lw.putRecord(d);
	lw.close();

	verifyFileContents(tmpFile, reinterpret_cast<char*>(&d), 2 * sizeof(double));
	std::remove(tmpFile);
}

TEST(LogWriterTest, Array) {
	char tmpFile[] = "/tmp/btXXXXXX";
	ASSERT_TRUE(mkstemp(tmpFile) != -1);

	math::Vector<15>::type d;
	d <<	23,	54,		34,		4,		25,
			23,	6,		46,		23,		-6,
			11,	868,	12312,	-44.2,	1;

	log::Writer<math::Vector<15>::type> lw(tmpFile);
	lw.putRecord(d);
	lw.close();

	verifyFileContents(tmpFile, reinterpret_cast<char*>(d.data()), sizeof(double) * d.size());
	std::remove(tmpFile);
}

TEST(LogWriterTest, SeveralRecords) {
	char tmpFile[] = "/tmp/btXXXXXX";
	ASSERT_TRUE(mkstemp(tmpFile) != -1);

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
