/*
 * units.cpp
 *
 *  Created on: Oct 27, 2009
 *      Author: dc
 */


#include <sstream>
#include <stdexcept>
#include <gsl/gsl_vector.h>
#include <libconfig.h++>

#include <gtest/gtest.h>
#include <barrett/math/vector.h>
#include <barrett/units.h>


namespace {
using namespace barrett;


DECLARE_UNITS(LocalUnits);


// template parameters list the types the following tests should be run over
typedef ::testing::Types<
		math::Vector<5>,
		units::JointTorques<5>,
		units::JointPositions<5>,
		LocalUnits<5> > UATypes;

template<typename T>
class VectorTypedTest : public ::testing::Test {
public:
	VectorTypedTest() :
		a() {}
protected:
	T a;
};

TYPED_TEST_CASE(VectorTypedTest, UATypes);


TYPED_TEST(VectorTypedTest, InitialValueCtor) {
	TypeParam a(-487.9);

	for (size_t i = 0; i < TypeParam::SIZE; ++i) {
		EXPECT_EQ(-487.9, a[i]);
	}
}

TYPED_TEST(VectorTypedTest, GslVectorCtor) {
	gsl_vector* gslVec = gsl_vector_calloc(TypeParam::SIZE);
	for (size_t i = 0; i < TypeParam::SIZE; ++i) {
		gsl_vector_set(gslVec, i, i*0.1);
	}

	TypeParam a(gslVec);

	for (size_t i = 0; i < TypeParam::SIZE; ++i) {
		EXPECT_EQ(i*0.1, a[i]);
	}
}

TYPED_TEST(VectorTypedTest, GslVectorCtorThrows) {
	gsl_vector* gslVec = gsl_vector_calloc(TypeParam::SIZE+1);
	EXPECT_THROW(TypeParam a(gslVec), std::logic_error);
}

TYPED_TEST(VectorTypedTest, ConfigCtor) {
	this->a << -20, -.5, 0, 38.2, 2.3e4;

	libconfig::Config config;
	config.readFile("test.config");
	TypeParam b(config.lookup("vector_test.five"));

	EXPECT_EQ(this->a, b);
}

TYPED_TEST(VectorTypedTest, ConfigCtorThrows) {
	libconfig::Config config;
	config.readFile("test.config");

	EXPECT_THROW(TypeParam(config.lookup("vector_test.four")), std::runtime_error);
	EXPECT_THROW(TypeParam(config.lookup("vector_test.six")), std::runtime_error);
}

TYPED_TEST(VectorTypedTest, CopyCtor) {
	TypeParam a(-487.9);
	TypeParam b(a);

	a.setConstant(2.0);

	for (size_t i = 0; i < TypeParam::SIZE; ++i) {
		EXPECT_EQ(2.0, a[i]);
		EXPECT_EQ(-487.9, b[i]);
		EXPECT_EQ(b[i], gsl_vector_get(b.asGslVector(), i));
	}
}

TYPED_TEST(VectorTypedTest, CopyToGslVector) {
	gsl_vector* gslVec = gsl_vector_calloc(TypeParam::SIZE);
	this->a << 5, 42.8, 37, -12, 1.4;

	this->a.copyTo(gslVec);

	for (size_t i = 0; i < TypeParam::SIZE; ++i) {
		EXPECT_EQ(this->a[i], gsl_vector_get(gslVec, i));
	}
}

TYPED_TEST(VectorTypedTest, CopyToGslVectorThrows) {
	gsl_vector* gslVec = gsl_vector_calloc(TypeParam::SIZE+1);
	EXPECT_THROW(this->a.copyTo(gslVec), std::logic_error);
}

TYPED_TEST(VectorTypedTest, CopyFromGslVector) {
	gsl_vector* gslVec = gsl_vector_calloc(TypeParam::SIZE);
	for (size_t i = 0; i < TypeParam::SIZE; ++i) {
		gsl_vector_set(gslVec, i, i*0.1);
	}

	this->a.copyFrom(gslVec);

	for (size_t i = 0; i < TypeParam::SIZE; ++i) {
		EXPECT_EQ(i*0.1, this->a[i]);
	}
}

TYPED_TEST(VectorTypedTest, CopyFromGslVectorThrows) {
	gsl_vector* gslVec = gsl_vector_calloc(TypeParam::SIZE+1);
	EXPECT_THROW(this->a.copyFrom(gslVec), std::logic_error);
}

TYPED_TEST(VectorTypedTest, CopyFromConfig) {
	this->a << -20, -.5, 0, 38.2, 2.3e4;

	libconfig::Config config;
	config.readFile("test.config");
	TypeParam b;
	b.copyFrom(config.lookup("vector_test.five"));

	EXPECT_EQ(this->a, b);
}

TYPED_TEST(VectorTypedTest, CopyFromConfigThrows) {
	libconfig::Config config;
	config.readFile("test.config");

	EXPECT_THROW(this->a.copyFrom(config.lookup("vector_test.four")), std::runtime_error);
	EXPECT_THROW(this->a.copyFrom(config.lookup("vector_test.six")), std::runtime_error);
}

TYPED_TEST(VectorTypedTest, AsGslVector) {
	gsl_vector* gslVec = this->a.asGslVector();

	this->a << 5, 42.8, 37, -12, 1.4;
	for (size_t i = 0; i < TypeParam::SIZE; ++i) {
		EXPECT_EQ(this->a[i], gsl_vector_get(gslVec, i));
	}

	gsl_vector_set(gslVec, 2, 8.9);
	EXPECT_EQ(8.9, this->a[2]);

	this->a[4] = -3.2;
	EXPECT_EQ(-3.2, gsl_vector_get(gslVec, 4));
}

TYPED_TEST(VectorTypedTest, IsZero) {
	this->a.setConstant(0.0);
	EXPECT_TRUE(this->a.isZero());

	this->a[0] = 1.0;
	EXPECT_FALSE(this->a.isZero());

	this->a.setConstant(-5.8);
	EXPECT_FALSE(this->a.isZero());
}

TYPED_TEST(VectorTypedTest, CopyFromTypeParam) {
	this->a << 5, 42.8, 37, -12, 1.4;
	TypeParam a_copy = this->a;  // uses copy constructor

	EXPECT_EQ(this->a, a_copy);

	this->a.setConstant(20.2);
	for (size_t i = 0; i < TypeParam::SIZE; ++i) {
		EXPECT_EQ(a_copy[i], gsl_vector_get(a_copy.asGslVector(), i));
	}
}

TYPED_TEST(VectorTypedTest, AssignFromTypeParam) {
	this->a << 5, 42.8, 37, -12, 1.4;
	TypeParam a_copy;
	a_copy = this->a;  // uses assignment operator

	EXPECT_EQ(this->a, a_copy);

	this->a.setConstant(20.2);
	for (size_t i = 0; i < TypeParam::SIZE; ++i) {
		EXPECT_EQ(a_copy[i], gsl_vector_get(a_copy.asGslVector(), i));
	}
}

TYPED_TEST(VectorTypedTest, CopyFromVector) {
	math::Vector<5> vec;
	vec << 5, 42.8, 37, -12, 1.4;
	TypeParam vec_copy = vec;  // uses copy constructor

	EXPECT_EQ(vec, vec_copy);

	vec.setConstant(20.2);
	for (size_t i = 0; i < TypeParam::SIZE; ++i) {
		EXPECT_EQ(vec_copy[i], gsl_vector_get(vec_copy.asGslVector(), i));
	}
}

TYPED_TEST(VectorTypedTest, AssignFromVector) {
	math::Vector<5> vec;
	vec << 5, 42.8, 37, -12, 1.4;
	TypeParam vec_copy;
	vec_copy = vec;  // uses assignment operator

	EXPECT_EQ(vec, vec_copy);

	vec.setConstant(20.2);
	for (size_t i = 0; i < TypeParam::SIZE; ++i) {
		EXPECT_EQ(vec_copy[i], gsl_vector_get(vec_copy.asGslVector(), i));
	}
}

TYPED_TEST(VectorTypedTest, ExplicitAssignment) {
	this->a << 5, 42.8, 37, -12, 1.4;

	double expected[] = { 5, 42.8, 37, -12, 1.4 };
	for (size_t i = 0; i < TypeParam::SIZE; ++i) {
		EXPECT_EQ(expected[i], this->a[i]);
	}
}

TYPED_TEST(VectorTypedTest, AccessAndModifyMembersByIndex) {
	for (size_t i = 0; i < TypeParam::SIZE; ++i) {
		this->a[i] = i/10.0 - 1;  // test both at()
		EXPECT_EQ(i/10.0 - 1, this->a[i]);  // and []
	}
}

//TYPED_TEST(VectorTypedTest, AccessAndModifyMembersByIterator) {
//	typename TypeParam::iterator i;
//	for (i = this->a.begin(); i != this->a.end(); ++i) {
//		*i = -38.1e7;
//		EXPECT_EQ(-38.1e7, *i);
//	}
//}

//// we just want this to compile
//TYPED_TEST(VectorTypedTest, BoostArrayOperators) {
//	TypeParam a_copy = this->a;
//
//	EXPECT_TRUE(this->a == a_copy);
//	EXPECT_FALSE(this->a != a_copy);
//
//	EXPECT_TRUE(this->a <= a_copy);
//	EXPECT_FALSE(this->a > a_copy);
//
//	EXPECT_TRUE(this->a >= a_copy);
//	EXPECT_FALSE(this->a < a_copy);
//}

//TYPED_TEST(VectorTypedTest, VectorArithmetic) {
//	TypeParam a1, a2, expected, result;
//
//	a1 << 1, 2, 3, 4, 5;
//	a2 << 5, 4, 3, 2, 1;
//
//	result = a1 + a2;
//	expected.setConstant(6);
//	EXPECT_EQ(expected, result);
//
//	result = a1 - a2;
//	expected << -4, -2, 0, 2, 4;
//	EXPECT_EQ(expected, result);
//
//	result = a1 * a2;
//	expected << 5, 8, 9, 8, 5;
//	EXPECT_EQ(expected, result);
//
//	result = a1 / a2;
//	expected << 0.2, 0.5, 1, 2, 5;
//	EXPECT_EQ(expected, result);
//
//	result = -a1;
//	expected << -1, -2, -3, -4, -5;
//	EXPECT_EQ(expected, result);
//
//	result = ((a1/a2) + (a1*a2)) / (-a1);
//	expected << -5.2, -4.25, -10.0/3.0, -2.5, -2;
//	EXPECT_EQ(expected, result);
//}
//
//TYPED_TEST(VectorTypedTest, VectorScalarArithmetic) {
//	TypeParam a, expected, result;
//
//	a << 1, 2, 3, 4, 5;
//
//	result = a + 5;
//	expected << 6, 7, 8, 9, 10;
//	EXPECT_EQ(expected, result);
//
//	result = 5 + a;
//	expected << 6, 7, 8, 9, 10;
//	EXPECT_EQ(expected, result);
//
//	result = a - 5;
//	expected << -4, -3, -2, -1, 0;
//	EXPECT_EQ(expected, result);
//
//	result = 5 - a;
//	expected << 4, 3, 2, 1, 0;
//	EXPECT_EQ(expected, result);
//
//	result = a * 5;
//	expected << 5, 10, 15, 20, 25;
//	EXPECT_EQ(expected, result);
//
//	result = 5 * a;
//	expected << 5, 10, 15, 20, 25;
//	EXPECT_EQ(expected, result);
//
//	result = a / 5;
//	expected << 0.2, 0.4, 0.6, 0.8, 1;
//	EXPECT_EQ(expected, result);
//
//	result = 5 / a;
//	expected << 5, 2.5, 5.0/3.0, 1.25, 1;
//	EXPECT_EQ(expected, result);
//
//	// TODO(dc): need an approxEqual() method
////	result = (0.6*a + 8) / 0.6 - a;
////	expected.setConstant(8.0/0.6);
////	EXPECT_EQ(expected, result);
//}

TYPED_TEST(VectorTypedTest, OstreamOperator) {
	this->a.setConstant(0);
	std::stringstream ss;
	ss << this->a;
	EXPECT_EQ("[0, 0, 0, 0, 0]", ss.str());
}


}
