/*
 * reader.cpp
 *
 *  Created on: Dec 31, 2009
 *      Author: dc
 */


#include <stdexcept>
#include <cstdio>

#include <gtest/gtest.h>

#include <boost/tuple/tuple.hpp>
#include <boost/tuple/tuple_comparison.hpp>
#include <boost/tuple/tuple_io.hpp>

#include <barrett/math/matrix.h>
#include <barrett/units.h>
#include <barrett/log/reader.h>
#include <barrett/log/writer.h>

#include "./verify_file_contents.h"


namespace {
using namespace barrett;


TEST(LogReaderTest, CtorThrows) {
	// TODO(dc): test this!
}

TEST(LogReaderTest, Double) {
	char tmpFile[] = "/tmp/btXXXXXX";
	ASSERT_TRUE(mkstemp(tmpFile) != -1);

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

TEST(LogReaderTest, TupleA) {
	typedef boost::tuple<double, double> tuple_type;

	char tmpFile[] = "/tmp/btXXXXXX";
	ASSERT_TRUE(mkstemp(tmpFile) != -1);

	tuple_type d(7.2, 803.0);

	log::Writer<tuple_type> lw(tmpFile);
	lw.putRecord(d);
	lw.close();

	log::Reader<tuple_type> lr(tmpFile);
	EXPECT_EQ(1, lr.numRecords());
	EXPECT_EQ(d, lr.getRecord());
	EXPECT_THROW(lr.getRecord(), std::underflow_error);
	lr.close();

	std::remove(tmpFile);
}

TEST(LogReaderTest, TupleB) {
	typedef boost::tuple<math::Vector<15>::type, double, double, units::JointTorques<3>::type> tuple_type;

	char tmpFile[] = "/tmp/btXXXXXX";
	ASSERT_TRUE(mkstemp(tmpFile) != -1);

	tuple_type d;
	d.get<0>() <<	23,	54,		34,		4,		25,
					23,	6,		46,		23,		-6,
					11,	868,	12312,	-44.2,	1;
	d.get<1>() = 2323823e-12;
	d.get<2>() = -0.5;
	d.get<3>() << 2.23, 867, -34.78e6;


	log::Writer<tuple_type> lw(tmpFile);
	lw.putRecord(d);
	lw.close();

	log::Reader<tuple_type> lr(tmpFile);
	EXPECT_EQ(1, lr.numRecords());
	EXPECT_EQ(d, lr.getRecord());
	EXPECT_THROW(lr.getRecord(), std::underflow_error);
	lr.close();

	std::remove(tmpFile);
}

TEST(LogReaderTest, Array) {
	char tmpFile[] = "/tmp/btXXXXXX";
	ASSERT_TRUE(mkstemp(tmpFile) != -1);

	math::Vector<15>::type d;
	d <<	23,	54,		34,		4,		25,
			23,	6,		46,		23,		-6,
			11,	868,	12312,	-44.2,	1;

	log::Writer<math::Vector<15>::type> lw(tmpFile);
	lw.putRecord(d);
	lw.close();

	log::Reader<math::Vector<15>::type> lr(tmpFile);
	EXPECT_EQ(1, lr.numRecords());
	EXPECT_EQ(d, lr.getRecord());
	EXPECT_THROW(lr.getRecord(), std::underflow_error);
	lr.close();

	std::remove(tmpFile);
}

TEST(LogReaderTest, SeveralRecords) {
	char tmpFile[] = "/tmp/btXXXXXX";
	ASSERT_TRUE(mkstemp(tmpFile) != -1);

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

TEST(LogReaderTest, ExportCSVDouble) {
	char tmpFile[] = "/tmp/btXXXXXX";
	ASSERT_TRUE(mkstemp(tmpFile) != -1);

	char tmpFile2[] = "/tmp/btXXXXXX";
	ASSERT_TRUE(mkstemp(tmpFile2) != -1);

	double ds[] = {3e7, -12, 432, 8.888};
	size_t n = sizeof(ds)/sizeof(double);

	log::Writer<double> lw(tmpFile);
	for (size_t i = 0; i < n; ++i) {
		lw.putRecord(ds[i]);
	}
	lw.close();

	log::Reader<double> lr(tmpFile);
	lr.exportCSV(tmpFile2);
	lr.close();

	char contents[] = "3e+07\n-12\n432\n8.888\n";
	verifyFileContents(tmpFile2, contents, sizeof(contents) - 1 /*ignore the \0 on the end! */);

	std::remove(tmpFile);
	std::remove(tmpFile2);
}

TEST(LogReaderTest, ExportCSVArray) {
	char tmpFile[] = "/tmp/btXXXXXX";
	ASSERT_TRUE(mkstemp(tmpFile) != -1);

	char tmpFile2[] = "/tmp/btXXXXXX";
	ASSERT_TRUE(mkstemp(tmpFile2) != -1);

	math::Vector<15>::type d;

	log::Writer<math::Vector<15>::type> lw(tmpFile);

	d <<	23,	54,		34,		4,		25,
			23,	6,		46,		23,		-6,
			11,	868,	12312,	-44.2,	1;
	lw.putRecord(d);

	d.setConstant(1.0/9.0);
	lw.putRecord(d);

	lw.close();

	log::Reader<math::Vector<15>::type> lr(tmpFile);
	lr.exportCSV(tmpFile2);
	lr.close();

	char contents[] = "23,54,34,4,25,23,6,46,23,-6,11,868,12312,-44.2,1\n0.111111,0.111111,0.111111,0.111111,0.111111,0.111111,0.111111,0.111111,0.111111,0.111111,0.111111,0.111111,0.111111,0.111111,0.111111\n";
	verifyFileContents(tmpFile2, contents, sizeof(contents) - 1 /*ignore the \0 on the end! */);

	std::remove(tmpFile);
	std::remove(tmpFile2);
}

TEST(LogReaderTest, ExportCSVTupleB) {
	typedef boost::tuple<math::Vector<15>::type, double, double, units::JointTorques<3>::type> tuple_type;

	char tmpFile[] = "/tmp/btXXXXXX";
	ASSERT_TRUE(mkstemp(tmpFile) != -1);

	char tmpFile2[] = "/tmp/btXXXXXX";
	ASSERT_TRUE(mkstemp(tmpFile2) != -1);

	tuple_type d;

	log::Writer<tuple_type> lw(tmpFile);

	d.get<0>() <<	23,	54,		34,		4,		25,
					23,	6,		46,		23,		-6,
					11,	868,	12312,	-44.2,	1;
	d.get<1>() = 2323823e-12;
	d.get<2>() = -0.5;
	d.get<3>() << 2.23, 867, -34.78e6;
	lw.putRecord(d);

	d.get<0>() <<	23,	6,		46,		23,		-6,
					11,	868,	12312,	-44.2,	1,
					23,	54,		34,		4,		25;
	d.get<1>() = 77712;
	d.get<2>() = 8.5;
	d.get<3>() << 234.23, 65656, 1;
	lw.putRecord(d);

	lw.close();

	log::Reader<tuple_type> lr(tmpFile);
	lr.exportCSV(tmpFile2);
	lr.close();

	char contents[] = "23,54,34,4,25,23,6,46,23,-6,11,868,12312,-44.2,1,2.32382e-06,-0.5,2.23,867,-3.478e+07\n23,6,46,23,-6,11,868,12312,-44.2,1,23,54,34,4,25,77712,8.5,234.23,65656,1\n";
	verifyFileContents(tmpFile2, contents, sizeof(contents) - 1 /*ignore the \0 on the end! */);

	std::remove(tmpFile);
	std::remove(tmpFile2);
}


}
