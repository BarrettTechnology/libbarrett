/*
 * matrix.cpp
 *
 *  Created on: Oct 12, 2010
 *      Author: dc
 */

#include <gtest/gtest.h>
#include <gsl/gsl_matrix.h>

#include <barrett/math/matrix.h>
#include <barrett/units.h>


// TODO(dc): finish writing tests


namespace {
using namespace barrett;


template<int R, int C> struct LocalUnits {
	typedef typename math::Matrix<R,C, LocalUnits<R,C> >::type type;
};

const int ROWS = 3;
const int COLS = 2;


// tests for fixed sized matrices
template<typename T>
class FixedMatrixTypedTest : public ::testing::Test {
public:
	FixedMatrixTypedTest() :
		a() {}
protected:
	T a;
};

typedef ::testing::Types<
			math::Matrix<ROWS,COLS>,
			LocalUnits<ROWS,COLS>::type
		> FixedTypes;
TYPED_TEST_CASE(FixedMatrixTypedTest, FixedTypes);


// tests for dynamically sized matrices
template<typename T>
class DynamicMatrixTypedTest : public ::testing::Test {
public:
	DynamicMatrixTypedTest() :
		a(ROWS,COLS) {}
protected:
	T a;
};

typedef ::testing::Types<
			math::Matrix<Eigen::Dynamic,Eigen::Dynamic>,
			LocalUnits<Eigen::Dynamic,Eigen::Dynamic>::type
		> DynamicTypes;
TYPED_TEST_CASE(DynamicMatrixTypedTest, DynamicTypes);


// common tests
template<typename T>
class MatrixTypedTest : public ::testing::Test {
public:
	MatrixTypedTest() :
		a(ROWS,COLS) {}
protected:
	T a;
};

typedef ::testing::Types<
			math::Matrix<ROWS,COLS>,
			LocalUnits<ROWS,COLS>::type,
			math::Matrix<Eigen::Dynamic,Eigen::Dynamic>,
			LocalUnits<Eigen::Dynamic,Eigen::Dynamic>::type
		> BothTypes;
TYPED_TEST_CASE(MatrixTypedTest, BothTypes);



// uses the actual default ctor
TYPED_TEST(FixedMatrixTypedTest, DefaultCtor) {
	for (int i = 0; i < this->a.rows(); ++i) {
		for (int j = 0; j < this->a.cols(); ++j) {
			EXPECT_EQ(0.0, this->a(i,j));
		}
	}
}

// uses the closest thing to a default ctor that a dynamic vector has
// both types of vectors should be able to be constructed this way
TYPED_TEST(MatrixTypedTest, DefaultCtor) {
	for (int i = 0; i < this->a.rows(); ++i) {
		for (int j = 0; j < this->a.cols(); ++j) {
			EXPECT_EQ(0.0, this->a(i,j));
		}
	}
}

TYPED_TEST(FixedMatrixTypedTest, InitialValueCtor) {
	TypeParam a(-487.9);

	for (int i = 0; i < a.rows(); ++i) {
		for (int j = 0; j < a.cols(); ++j) {
			EXPECT_EQ(-487.9, a(i,j));
		}
	}
}

// both types of vectors should be able to be constructed this way
TYPED_TEST(MatrixTypedTest, InitialValueCtor) {
	TypeParam a(ROWS,COLS, -487.9);

	for (int i = 0; i < a.rows(); ++i) {
		for (int j = 0; j < a.cols(); ++j) {
			EXPECT_EQ(-487.9, a(i,j));
		}
	}
}

TYPED_TEST(MatrixTypedTest, GslMatrixCtor) {
	gsl_matrix* gslMat = gsl_matrix_calloc(this->a.rows(), this->a.cols());
	for (int i = 0; i < this->a.rows(); ++i) {
		for (int j = 0; j < this->a.cols(); ++j) {
			gsl_matrix_set(gslMat, i,j, i*0.1 + i-j);
		}
	}

	TypeParam a(gslMat);

	for (int i = 0; i < a.rows(); ++i) {
		for (int j = 0; j < a.cols(); ++j) {
			EXPECT_EQ(i*0.1 + i-j, a(i,j));
		}
	}

	gsl_matrix_free(gslMat);
}

TYPED_TEST(FixedMatrixTypedTest, GslMatrixCtorThrows) {
	gsl_matrix* gslMat;

	gslMat = gsl_matrix_calloc(this->a.rows()+1, this->a.cols()+1);
	EXPECT_THROW(TypeParam a(gslMat), std::logic_error);
	gsl_matrix_free(gslMat);

	gslMat = gsl_matrix_calloc(this->a.rows()+1, this->a.cols()+0);
	EXPECT_THROW(TypeParam a(gslMat), std::logic_error);
	gsl_matrix_free(gslMat);

	gslMat = gsl_matrix_calloc(this->a.rows()+1, this->a.cols()-1);
	EXPECT_THROW(TypeParam a(gslMat), std::logic_error);
	gsl_matrix_free(gslMat);


	gslMat = gsl_matrix_calloc(this->a.rows()+0, this->a.cols()+1);
	EXPECT_THROW(TypeParam a(gslMat), std::logic_error);
	gsl_matrix_free(gslMat);

	gslMat = gsl_matrix_calloc(this->a.rows()+0, this->a.cols()-1);
	EXPECT_THROW(TypeParam a(gslMat), std::logic_error);
	gsl_matrix_free(gslMat);


	gslMat = gsl_matrix_calloc(this->a.rows()-1, this->a.cols()+1);
	EXPECT_THROW(TypeParam a(gslMat), std::logic_error);
	gsl_matrix_free(gslMat);

	gslMat = gsl_matrix_calloc(this->a.rows()-1, this->a.cols()+0);
	EXPECT_THROW(TypeParam a(gslMat), std::logic_error);
	gsl_matrix_free(gslMat);

	gslMat = gsl_matrix_calloc(this->a.rows()-1, this->a.cols()-1);
	EXPECT_THROW(TypeParam a(gslMat), std::logic_error);
	gsl_matrix_free(gslMat);
}

TYPED_TEST(DynamicMatrixTypedTest, GslMatrixCtorResizes) {
	gsl_matrix* gslMat;

	gslMat = gsl_matrix_calloc(this->a.rows()+1, this->a.cols()+1);
	EXPECT_NO_THROW(TypeParam a(gslMat));
	EXPECT_EQ(gslMat->size1, TypeParam(gslMat).rows());
	EXPECT_EQ(gslMat->size2, TypeParam(gslMat).cols());
	gsl_matrix_free(gslMat);

	gslMat = gsl_matrix_calloc(this->a.rows()+1, this->a.cols()+0);
	EXPECT_NO_THROW(TypeParam a(gslMat));
	EXPECT_EQ(gslMat->size1, TypeParam(gslMat).rows());
	EXPECT_EQ(gslMat->size2, TypeParam(gslMat).cols());
	gsl_matrix_free(gslMat);

	gslMat = gsl_matrix_calloc(this->a.rows()+1, this->a.cols()-1);
	EXPECT_NO_THROW(TypeParam a(gslMat));
	EXPECT_EQ(gslMat->size1, TypeParam(gslMat).rows());
	EXPECT_EQ(gslMat->size2, TypeParam(gslMat).cols());
	gsl_matrix_free(gslMat);


	gslMat = gsl_matrix_calloc(this->a.rows()+0, this->a.cols()+1);
	EXPECT_NO_THROW(TypeParam a(gslMat));
	EXPECT_EQ(gslMat->size1, TypeParam(gslMat).rows());
	EXPECT_EQ(gslMat->size2, TypeParam(gslMat).cols());
	gsl_matrix_free(gslMat);

	gslMat = gsl_matrix_calloc(this->a.rows()+0, this->a.cols()-1);
	EXPECT_NO_THROW(TypeParam a(gslMat));
	EXPECT_EQ(gslMat->size1, TypeParam(gslMat).rows());
	EXPECT_EQ(gslMat->size2, TypeParam(gslMat).cols());
	gsl_matrix_free(gslMat);


	gslMat = gsl_matrix_calloc(this->a.rows()-1, this->a.cols()+1);
	EXPECT_NO_THROW(TypeParam a(gslMat));
	EXPECT_EQ(gslMat->size1, TypeParam(gslMat).rows());
	EXPECT_EQ(gslMat->size2, TypeParam(gslMat).cols());
	gsl_matrix_free(gslMat);

	gslMat = gsl_matrix_calloc(this->a.rows()-1, this->a.cols()+0);
	EXPECT_NO_THROW(TypeParam a(gslMat));
	EXPECT_EQ(gslMat->size1, TypeParam(gslMat).rows());
	EXPECT_EQ(gslMat->size2, TypeParam(gslMat).cols());
	gsl_matrix_free(gslMat);

	gslMat = gsl_matrix_calloc(this->a.rows()-1, this->a.cols()-1);
	EXPECT_NO_THROW(TypeParam a(gslMat));
	EXPECT_EQ(gslMat->size1, TypeParam(gslMat).rows());
	EXPECT_EQ(gslMat->size2, TypeParam(gslMat).cols());
	gsl_matrix_free(gslMat);
}

TYPED_TEST(MatrixTypedTest, ConfigCtor) {
	this->a << -20, -.5, 0, 38.2, 2.3e4, 209;

	libconfig::Config config;
	config.readFile("test.config");
	TypeParam b(config.lookup("matrix_test.three_two"));

	EXPECT_EQ(this->a, b);
}

TYPED_TEST(FixedMatrixTypedTest, ConfigCtorThrows) {
	libconfig::Config config;
	config.readFile("test.config");

	EXPECT_THROW(TypeParam(config.lookup("matrix_test.two_one")), std::runtime_error);
	EXPECT_THROW(TypeParam(config.lookup("matrix_test.two_two")), std::runtime_error);
	EXPECT_THROW(TypeParam(config.lookup("matrix_test.two_three")), std::runtime_error);

	EXPECT_THROW(TypeParam(config.lookup("matrix_test.three_one")), std::runtime_error);
	EXPECT_THROW(TypeParam(config.lookup("matrix_test.three_three")), std::runtime_error);

	EXPECT_THROW(TypeParam(config.lookup("matrix_test.four_one")), std::runtime_error);
	EXPECT_THROW(TypeParam(config.lookup("matrix_test.four_two")), std::runtime_error);
	EXPECT_THROW(TypeParam(config.lookup("matrix_test.four_three")), std::runtime_error);
}

TYPED_TEST(DynamicMatrixTypedTest, ConfigCtorResizes) {
	libconfig::Config config;
	config.readFile("test.config");
	libconfig::Setting* setting;

	setting = &config.lookup("matrix_test.two_one");
	EXPECT_NO_THROW(TypeParam(*setting));
	EXPECT_EQ(2, TypeParam(*setting).rows());
	EXPECT_EQ(1, TypeParam(*setting).cols());

	setting = &config.lookup("matrix_test.two_one");
	EXPECT_NO_THROW(TypeParam(*setting));
	EXPECT_EQ(2, TypeParam(*setting).rows());
	EXPECT_EQ(1, TypeParam(*setting).cols());

	setting = &config.lookup("matrix_test.two_one");
	EXPECT_NO_THROW(TypeParam(*setting));
	EXPECT_EQ(2, TypeParam(*setting).rows());
	EXPECT_EQ(1, TypeParam(*setting).cols());


	setting = &config.lookup("matrix_test.three_two");
	EXPECT_NO_THROW(TypeParam(*setting));
	EXPECT_EQ(3, TypeParam(*setting).rows());
	EXPECT_EQ(2, TypeParam(*setting).cols());

	setting = &config.lookup("matrix_test.three_two");
	EXPECT_NO_THROW(TypeParam(*setting));
	EXPECT_EQ(3, TypeParam(*setting).rows());
	EXPECT_EQ(2, TypeParam(*setting).cols());


	setting = &config.lookup("matrix_test.four_three");
	EXPECT_NO_THROW(TypeParam(*setting));
	EXPECT_EQ(4, TypeParam(*setting).rows());
	EXPECT_EQ(3, TypeParam(*setting).cols());

	setting = &config.lookup("matrix_test.four_three");
	EXPECT_NO_THROW(TypeParam(*setting));
	EXPECT_EQ(4, TypeParam(*setting).rows());
	EXPECT_EQ(3, TypeParam(*setting).cols());

	setting = &config.lookup("matrix_test.four_three");
	EXPECT_NO_THROW(TypeParam(*setting));
	EXPECT_EQ(4, TypeParam(*setting).rows());
	EXPECT_EQ(3, TypeParam(*setting).cols());
}

TYPED_TEST(MatrixTypedTest, CopyCtor) {
	TypeParam a(ROWS,COLS, -487.9);
	TypeParam b(a);

	a.setConstant(2.0);

	for (int i = 0; i < a.rows(); ++i) {
		for (int j = 0; j < a.cols(); ++j) {
			EXPECT_EQ(2.0, a(i,j));
			EXPECT_EQ(-487.9, b(i,j));
			EXPECT_EQ(b(i,j), gsl_matrix_get(b.asGslType(), i,j));
		}
	}
}

TYPED_TEST(MatrixTypedTest, CopyToGslMatrix) {
	gsl_matrix* gslMat = gsl_matrix_calloc(this->a.rows(), this->a.cols());
	this->a << 5, 42.8, 37, -12, 1.4, -3e-3;

	this->a.copyTo(gslMat);

	for (int i = 0; i < this->a.rows(); ++i) {
		for (int j = 0; j < this->a.cols(); ++j) {
			EXPECT_EQ(this->a(i,j), gsl_matrix_get(gslMat, i,j));
		}
	}

	gsl_matrix_free(gslMat);
}

TYPED_TEST(MatrixTypedTest, CopyToGslMatrixThrows) {
	gsl_matrix* gslMat;

	gslMat = gsl_matrix_calloc(this->a.rows()+1, this->a.cols()+1);
	EXPECT_THROW(this->a.copyTo(gslMat), std::logic_error);
	gsl_matrix_free(gslMat);

	gslMat = gsl_matrix_calloc(this->a.rows()+1, this->a.cols()+0);
	EXPECT_THROW(this->a.copyTo(gslMat), std::logic_error);
	gsl_matrix_free(gslMat);

	gslMat = gsl_matrix_calloc(this->a.rows()+1, this->a.cols()-1);
	EXPECT_THROW(this->a.copyTo(gslMat), std::logic_error);
	gsl_matrix_free(gslMat);


	gslMat = gsl_matrix_calloc(this->a.rows()+0, this->a.cols()+1);
	EXPECT_THROW(this->a.copyTo(gslMat), std::logic_error);
	gsl_matrix_free(gslMat);

	gslMat = gsl_matrix_calloc(this->a.rows()+0, this->a.cols()-1);
	EXPECT_THROW(this->a.copyTo(gslMat), std::logic_error);
	gsl_matrix_free(gslMat);


	gslMat = gsl_matrix_calloc(this->a.rows()-1, this->a.cols()+1);
	EXPECT_THROW(this->a.copyTo(gslMat), std::logic_error);
	gsl_matrix_free(gslMat);

	gslMat = gsl_matrix_calloc(this->a.rows()-1, this->a.cols()+0);
	EXPECT_THROW(this->a.copyTo(gslMat), std::logic_error);
	gsl_matrix_free(gslMat);

	gslMat = gsl_matrix_calloc(this->a.rows()-1, this->a.cols()-1);
	EXPECT_THROW(this->a.copyTo(gslMat), std::logic_error);
	gsl_matrix_free(gslMat);
}

TYPED_TEST(MatrixTypedTest, CopyFromGslMatrix) {
	gsl_matrix* gslMat = gsl_matrix_calloc(this->a.rows(), this->a.cols());
	for (int i = 0; i < this->a.rows(); ++i) {
		for (int j = 0; j < this->a.cols(); ++j) {
			gsl_matrix_set(gslMat, i,j, i*0.1 + i-j);
		}
	}

	this->a.copyFrom(gslMat);

	for (int i = 0; i < this->a.rows(); ++i) {
		for (int j = 0; j < this->a.cols(); ++j) {
			EXPECT_EQ(i*0.1 + i-j, this->a(i,j));
		}
	}

	gsl_matrix_free(gslMat);
}

TYPED_TEST(MatrixTypedTest, CopyFromGslMatrixThrows) {
	gsl_matrix* gslMat;

	gslMat = gsl_matrix_calloc(this->a.rows()+1, this->a.cols()+1);
	EXPECT_THROW(this->a.copyFrom(gslMat), std::logic_error);
	gsl_matrix_free(gslMat);

	gslMat = gsl_matrix_calloc(this->a.rows()+1, this->a.cols()+0);
	EXPECT_THROW(this->a.copyFrom(gslMat), std::logic_error);
	gsl_matrix_free(gslMat);

	gslMat = gsl_matrix_calloc(this->a.rows()+1, this->a.cols()-1);
	EXPECT_THROW(this->a.copyFrom(gslMat), std::logic_error);
	gsl_matrix_free(gslMat);


	gslMat = gsl_matrix_calloc(this->a.rows()+0, this->a.cols()+1);
	EXPECT_THROW(this->a.copyFrom(gslMat), std::logic_error);
	gsl_matrix_free(gslMat);

	gslMat = gsl_matrix_calloc(this->a.rows()+0, this->a.cols()-1);
	EXPECT_THROW(this->a.copyFrom(gslMat), std::logic_error);
	gsl_matrix_free(gslMat);


	gslMat = gsl_matrix_calloc(this->a.rows()-1, this->a.cols()+1);
	EXPECT_THROW(this->a.copyFrom(gslMat), std::logic_error);
	gsl_matrix_free(gslMat);

	gslMat = gsl_matrix_calloc(this->a.rows()-1, this->a.cols()+0);
	EXPECT_THROW(this->a.copyFrom(gslMat), std::logic_error);
	gsl_matrix_free(gslMat);

	gslMat = gsl_matrix_calloc(this->a.rows()-1, this->a.cols()-1);
	EXPECT_THROW(this->a.copyFrom(gslMat), std::logic_error);
	gsl_matrix_free(gslMat);
}

TYPED_TEST(MatrixTypedTest, CopyFromConfig) {
	this->a << -20, -.5, 0, 38.2, 2.3e4, 209;

	libconfig::Config config;
	config.readFile("test.config");
	TypeParam b(ROWS,COLS);
	b.copyFrom(config.lookup("matrix_test.three_two"));

	EXPECT_EQ(this->a, b);
}

TYPED_TEST(MatrixTypedTest, CopyFromConfigThrows) {
	libconfig::Config config;
	config.readFile("test.config");

	EXPECT_THROW(this->a.copyFrom(config.lookup("matrix_test.two_one")), std::runtime_error);
	EXPECT_THROW(this->a.copyFrom(config.lookup("matrix_test.two_two")), std::runtime_error);
	EXPECT_THROW(this->a.copyFrom(config.lookup("matrix_test.two_three")), std::runtime_error);

	EXPECT_THROW(this->a.copyFrom(config.lookup("matrix_test.three_one")), std::runtime_error);
	EXPECT_THROW(this->a.copyFrom(config.lookup("matrix_test.three_three")), std::runtime_error);

	EXPECT_THROW(this->a.copyFrom(config.lookup("matrix_test.four_one")), std::runtime_error);
	EXPECT_THROW(this->a.copyFrom(config.lookup("matrix_test.four_two")), std::runtime_error);
	EXPECT_THROW(this->a.copyFrom(config.lookup("matrix_test.four_three")), std::runtime_error);
}

TYPED_TEST(MatrixTypedTest, AsGslMatrix) {
	gsl_matrix* gslMat = this->a.asGslType();

	EXPECT_EQ(this->a.rows(), gslMat->size1);
	EXPECT_EQ(this->a.cols(), gslMat->size2);
	EXPECT_EQ(NULL, gslMat->block);
	EXPECT_EQ(0, gslMat->owner);

	this->a << 5, 42.8, 37, -12, 1.4, -3e-3;
	for (int i = 0; i < this->a.rows(); ++i) {
		for (int j = 0; j < this->a.cols(); ++j) {
			EXPECT_EQ(this->a(i,j), gsl_matrix_get(gslMat, i,j));
		}
	}

	gsl_matrix_set(gslMat, 2,1, 8.9);
	EXPECT_EQ(8.9, this->a(2,1));

	this->a(1,0) = -3.2;
	EXPECT_EQ(-3.2, gsl_matrix_get(gslMat, 1,0));
}

/*
TYPED_TEST(MatrixTypedTest, IsZero) {
	this->a.setConstant(0.0);
	EXPECT_TRUE(this->a.isZero());

	this->a[0] = 1.0;
	EXPECT_FALSE(this->a.isZero());

	this->a.setConstant(-5.8);
	EXPECT_FALSE(this->a.isZero());
}

TYPED_TEST(MatrixTypedTest, CopyFromTypeParam) {
	this->a << 5, 42.8, 37, -12, 1.4;
	TypeParam a_copy = this->a;  // uses copy constructor

	EXPECT_EQ(this->a, a_copy);

	this->a.setConstant(20.2);
	for (int i = 0; i < this->a.size(); ++i) {
		EXPECT_EQ(a_copy[i], gsl_vector_get(a_copy.asGslType(), i));
	}
}

TYPED_TEST(MatrixTypedTest, AssignFromTypeParam) {
	this->a << 5, 42.8, 37, -12, 1.4;
	TypeParam a_copy(ROWS,COLS);
	a_copy = this->a;  // uses assignment operator

	EXPECT_EQ(this->a, a_copy);

	this->a.setConstant(20.2);
	for (int i = 0; i < this->a.size(); ++i) {
		EXPECT_EQ(a_copy[i], gsl_vector_get(a_copy.asGslType(), i));
	}
}

TYPED_TEST(MatrixTypedTest, CopyFromVector) {
	math::Vector<DIM>::type vec;
	vec << 5, 42.8, 37, -12, 1.4;
	TypeParam vec_copy = vec;  // uses copy constructor

	EXPECT_EQ(vec, vec_copy);

	vec.setConstant(20.2);
	for (int i = 0; i < this->a.size(); ++i) {
		EXPECT_EQ(vec_copy[i], gsl_vector_get(vec_copy.asGslType(), i));
	}
}

TYPED_TEST(MatrixTypedTest, AssignFromVector) {
	math::Vector<DIM>::type vec;
	vec << 5, 42.8, 37, -12, 1.4;
	TypeParam vec_copy(ROWS,COLS);
	vec_copy = vec;  // uses assignment operator

	EXPECT_EQ(vec, vec_copy);

	vec.setConstant(20.2);
	for (int i = 0; i < vec_copy.size(); ++i) {
		EXPECT_EQ(vec_copy[i], gsl_vector_get(vec_copy.asGslType(), i));
	}
}

TYPED_TEST(MatrixTypedTest, ExplicitAssignment) {
	this->a << 5, 42.8, 37, -12, 1.4;

	double expected[] = { 5, 42.8, 37, -12, 1.4 };
	for (int i = 0; i < this->a.size(); ++i) {
		EXPECT_EQ(expected[i], this->a[i]);
	}
}

TYPED_TEST(MatrixTypedTest, AccessAndModifyMembersByIndex) {
	for (int i = 0; i < this->a.size(); ++i) {
		this->a[i] = i/10.0 - 1;
		EXPECT_EQ(i/10.0 - 1, this->a[i]);
	}
}

TYPED_TEST(MatrixTypedTest, TraitsAssignZero) {
	typedef math::Traits<TypeParam> T;

	TypeParam result(ROWS,COLS);
	result.setConstant(7);

	T::zero(result);
	EXPECT_EQ(TypeParam::Zero(ROWS,COLS), result);
}

TYPED_TEST(FixedMatrixTypedTest, TraitsInstantiateZero) {
	typedef math::Traits<TypeParam> T;
	EXPECT_EQ(TypeParam::Zero(), T::zero());
}

// both types of vectors should be able to be instantiated this way
TYPED_TEST(MatrixTypedTest, TraitsInstantiateZero) {
	typedef math::Traits<TypeParam> T;
	EXPECT_EQ(TypeParam::Zero(ROWS,COLS), T::zero(ROWS,COLS));
}

TYPED_TEST(MatrixTypedTest, VectorVectorTraitsArithmetic) {
	typedef math::Traits<TypeParam> T;

	TypeParam a1(ROWS,COLS), a2(ROWS,COLS), expected(ROWS,COLS), result(ROWS,COLS);

	a1 << 1, 2, 3, 4, 5;
	a2 << 5, 4, 3, 2, 1;

	result = T::add(a1, a2);
	expected.setConstant(6);
	EXPECT_EQ(expected, result);

	result = T::sub(a1, a2);
	expected << -4, -2, 0, 2, 4;
	EXPECT_EQ(expected, result);

	result = T::mult(a1, a2);
	expected << 5, 8, 9, 8, 5;
	EXPECT_EQ(expected, result);

	result = T::div(a1, a2);
	expected << 0.2, 0.5, 1, 2, 5;
	EXPECT_EQ(expected, result);

	result = T::neg(a1);
	expected << -1, -2, -3, -4, -5;
	EXPECT_EQ(expected, result);

//	result = ((a1/a2) + (a1*a2)) / (-a1);
	result = T::div( T::add(T::div(a1,a2), T::mult(a1,a2)), T::neg(a1) );
	expected << -5.2, -4.25, -10.0/3.0, -2.5, -2;
	EXPECT_EQ(expected, result);
}

TYPED_TEST(MatrixTypedTest, VectorScalarTraitsArithmetic) {
	typedef math::Traits<TypeParam> T;

	TypeParam a(ROWS,COLS), expected(ROWS,COLS), result(ROWS,COLS);

	a << 1, 2, 3, 4, 5;

	result = T::add(a,5);
	expected << 6, 7, 8, 9, 10;
	EXPECT_EQ(expected, result);

	result = T::add(5, a);
	expected << 6, 7, 8, 9, 10;
	EXPECT_EQ(expected, result);

	result = T::sub(a, 5);
	expected << -4, -3, -2, -1, 0;
	EXPECT_EQ(expected, result);

	result = T::sub(5, a);
	expected << 4, 3, 2, 1, 0;
	EXPECT_EQ(expected, result);

	result = T::mult(a, 5);
	expected << 5, 10, 15, 20, 25;
	EXPECT_EQ(expected, result);

	result = T::mult(5, a);
	expected << 5, 10, 15, 20, 25;
	EXPECT_EQ(expected, result);

	result = T::div(a, 5);
	expected << 0.2, 0.4, 0.6, 0.8, 1;
	EXPECT_TRUE(expected.isApprox(result));

	result = T::div(5, a);
	expected << 5, 2.5, 5.0/3.0, 1.25, 1;
	EXPECT_TRUE(expected.isApprox(result));

//	result = (0.6*a + 8) / 0.6 - a;
	result = T::sub(T::div(T::add(T::mult(0.6, a), 8), 0.6), a);
	expected.setConstant(8.0/0.6);
	EXPECT_TRUE(expected.isApprox(result));
}

TYPED_TEST(MatrixTypedTest, OstreamOperator) {
	this->a.setConstant(0);
	std::stringstream ss;
	ss << this->a;
	EXPECT_EQ("[0, 0, 0, 0, 0]", ss.str());
}
*/

}
