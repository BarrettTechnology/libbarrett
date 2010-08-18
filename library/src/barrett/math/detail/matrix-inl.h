/*
 * matrix-inl.h
 *
 *  Created on: Jan 13, 2010
 *      Author: dc
 */

#include <stdexcept>
#include <cstring>
#include <sstream>

#include <boost/type_traits/is_same.hpp>
#include <boost/static_assert.hpp>
#include <libconfig.h++>

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

#include "../../detail/libconfig_utils.h"


namespace barrett {
namespace math {


template<int R, int C, typename Units>
inline Matrix<R,C, Units>::Matrix() :
	Base(), gsl()
{
	initGslType(&gsl);
}

template<int R, int C, typename Units>
inline Matrix<R,C, Units>::Matrix(int dim) :
	Base(dim), gsl()
{
	initGslType(&gsl);
}

template<int R, int C, typename Units>
inline Matrix<R,C, Units>::Matrix(int r, int c) :
	Base(r, c), gsl()
{
	initGslType(&gsl);
}

template<int R, int C, typename Units>
inline Matrix<R,C, Units>::Matrix(double x, double y) :
	Base(x, y), gsl()
{
	initGslType(&gsl);
}

template<int R, int C, typename Units>
inline Matrix<R,C, Units>::Matrix(double x, double y, double z) :
	Base(x, y, z), gsl()
{
	initGslType(&gsl);
}

template<int R, int C, typename Units>
inline Matrix<R,C, Units>::Matrix(double x, double y, double z, double w) :
	Base(x, y, z, w), gsl()
{
	initGslType(&gsl);
}

template<int R, int C, typename Units>
inline Matrix<R,C, Units>::Matrix(const double* data) :
	Base(data), gsl()
{
	initGslType(&gsl);
}

template<int R, int C, typename Units>
template<typename OtherDerived>
inline Matrix<R,C, Units>::Matrix(const Eigen::MatrixBase<OtherDerived>& other) :
	Base(other), gsl()
{
	initGslType(&gsl);
}

template<int R, int C, typename Units>
template<typename OtherDerived>
inline Matrix<R,C, Units>::Matrix(const Eigen::RotationBase<OtherDerived,Base::ColsAtCompileTime>& r) :
	Base(r), gsl()
{
	initGslType(&gsl);
}

//template<int R, int C, typename Units>
//template<typename OtherDerived>
//inline Matrix& Matrix<R,C, Units>::operator=(const Eigen::RotationBase<OtherDerived,Base::ColsAtCompileTime>& r)
//{
//	Base::operator=(r);
//	return *this;
//}


template<int R, int C, typename Units>
inline Matrix<R,C, Units>::Matrix(double d) :
	Base(), gsl()
{
	initGslType(&gsl);
	this->setConstant(d);
}

template<int R, int C, typename Units>
inline Matrix<R,C, Units>::Matrix(int r, double d) :
	Base(r), gsl()
{
	initGslType(&gsl);
	this->setConstant(d);
}

template<int R, int C, typename Units>
inline Matrix<R,C, Units>::Matrix(int r, int c, double d) :
	Base(r,c), gsl()
{
	initGslType(&gsl);
	this->setConstant(d);
}

template<int R, int C, typename Units>
inline Matrix<R,C, Units>::Matrix(const gsl_type* gslType) :
	Base(), gsl()
{
	resizeToMatchIfDynamic(gslType);
	initGslType(&gsl);
	copyFrom(gslType);
}

template<int R, int C, typename Units>
inline Matrix<R,C, Units>::Matrix(const libconfig::Setting& setting) :
	Base(), gsl()
{
	resizeIfDynamic(setting.getLength());  // TODO(dc): add support for Matrix settings
	initGslType(&gsl);
	copyFrom(setting);
}



template<int R, int C, typename Units>
inline Matrix<R,C, Units>::Matrix(const Matrix& a) :
	Base(a), gsl(a.gsl)
{
	initGslType(&gsl);
}

template<int R, int C, typename Units>
inline Matrix<R,C, Units>::~Matrix()
{
}


//template<int R, int C, typename Units>
//inline size_t Matrix<R,C, Units>::serializedLength()
//{
//	return sizeof(double) * R*C;
//}
//
//template<int R, int C, typename Units>
//inline void Matrix<R,C, Units>::serialize(char* dest) const
//{
//	std::memcpy(dest, this->data(), serializedLength());
//}
//
//template<int R, int C, typename Units>
//inline Matrix<R,C, Units> Matrix<R,C, Units>::unserialize(char* source)
//{
//	Matrix<R,C, Units> a;
//	std::memcpy(a.data(), source, serializedLength());
//	return a;
//}


template<int R, int C, typename Units>
inline void Matrix<R,C, Units>::copyTo(gsl_type* gslType) const
throw(std::logic_error)
{
	if (gslType->size != static_cast<size_t>(this->size())) {
		std::stringstream ss;
		ss << "(math::Matrix<>::copyTo(gsl_type*)): The size of the "
				"gsl_type must match the size of the Matrix. Expected size "
				<< this->size() << ", got size " << gslType->size;
		throw std::logic_error(ss.str());
	}

	for (int i = 0; i < this->size(); ++i) {
		gsl_vector_set(gslType, i, (*this)[i]);
	}
}

template<int R, int C, typename Units>
inline void Matrix<R,C, Units>::copyFrom(const gsl_type* gslType)
throw(std::logic_error)
{
	if (gslType->size != static_cast<size_t>(this->size())) {
		std::stringstream ss;
		ss << "(math::Matrix<>::copyFrom(gsl_type*)): The size of the "
				"gsl_type must match the size of the Matrix. Expected size "
				<< this->size() << ", got size " << gslType->size;
		throw std::logic_error(ss.str());
	}

	for (int i = 0; i < this->size(); ++i) {
		(*this)[i] = gsl_vector_get(gslType, i);
	}
}

template<int R, int C, typename Units>
inline void Matrix<R,C, Units>::copyFrom(const libconfig::Setting& setting)
{
	if (setting.getLength() != this->size()) {
		std::stringstream ss;
		ss << "(math::Matrix<>::copyFrom(libconfig::Setting)): The size of "
				"the configuration list must match the size of the Matrix. "
				"Expected size " << this->size() << ", got size " << setting.getLength() <<
				". Path: \"" << setting.getPath() << "\", Line: " <<
				setting.getSourceLine();
		throw std::runtime_error(ss.str());
	}

	for (int i = 0; i < this->size(); ++i) {
		(*this)[i] = detail::numericToDouble(setting[i]);
	}
}

template<int R, int C, typename Units>
inline typename Matrix<R,C, Units>::gsl_type* Matrix<R,C, Units>::asGslType()
{
	return &(this->gsl);
}

template<int R, int C, typename Units>
inline const typename Matrix<R,C, Units>::gsl_type* Matrix<R,C, Units>::asGslType() const
{
	return &(this->gsl);
}

template<int R, int C, typename Units>
inline void Matrix<R,C, Units>::resizeIfDynamic(int r, int c)
{
	if (R == Eigen::Dynamic  ||  C == Eigen::Dynamic) {
		this->resize(r,c);
	}
}

template<int R, int C, typename Units>
inline void Matrix<R,C, Units>::initGslType(gsl_vector* g)
{
	g->size = this->size();
	g->stride = 1;
	g->data = this->data();
	g->block = NULL;
	g->owner = 0;
}
template<int R, int C, typename Units>
inline void Matrix<R,C, Units>::initGslType(gsl_matrix* g)
{
	g->size1 = this->rows();
	g->size2 = this->cols();
	g->tda = this->cols();
	g->data = this->data();
	g->block = NULL;
	g->owner = 0;
}


template<int R, int C, typename Units>
inline void Matrix<R,C, Units>::resizeToMatchIfDynamic(const gsl_vector* g)
{
	resizeIfDynamic(g->size);
}
template<int R, int C, typename Units>
inline void Matrix<R,C, Units>::resizeToMatchIfDynamic(const gsl_matrix* g)
{
	resizeIfDynamic(g->size1, g->size2);
}

template<int R, int C, typename Units>
std::ostream& operator<< (std::ostream& os, const Matrix<R,C, Units>& a) {
	os << "[";

	// TODO(dc): Make separate vector/matrix formats.
	int maxIndex = a.size() - 1;
	for (int i = 0; i <= maxIndex; ++i) {
		os << a[i];
		if (i != maxIndex) {
			os << ", ";
		}
	}

	os << "]";
	return os;
}


}
}
