/*
 * matrix-inl.h
 *
 *  Created on: Jan 13, 2010
 *      Author: dc
 */

#include <stdexcept>
#include <cstring>
#include <sstream>

#include <boost/static_assert.hpp>
#include <libconfig.h++>

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

#include <barrett/detail/libconfig_utils.h>


namespace barrett {
namespace math {


//template<int R, int C, typename Units>
//inline Matrix<R,C, Units>::Matrix() :
//	Base(), gsl()
//{
//	initGslType(&gsl);
//}
//
//template<int R, int C, typename Units>
//inline Matrix<R,C, Units>::Matrix(int dim) :
//	Base(dim), gsl()
//{
//	initGslType(&gsl);
//}
//
//template<int R, int C, typename Units>
//inline Matrix<R,C, Units>::Matrix(int r, int c) :
//	Base(r, c), gsl()
//{
//	initGslType(&gsl);
//}

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
	resizeIfDynamic(setting.getLength(), setting[0].isNumber() ? 1 : setting[0].getLength());
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


template<int R, int C, typename Units>
inline size_t Matrix<R,C, Units>::serializedLength()
{
	// This method assumes a fixed-size Matrix.
	BOOST_STATIC_ASSERT( !Traits<type>::IsDynamic );
	return sizeof(double) * SIZE;
}

template<int R, int C, typename Units>
inline void Matrix<R,C, Units>::serialize(char* dest) const
{
	std::memcpy(dest, this->data(), serializedLength());
}

template<int R, int C, typename Units>
inline Matrix<R,C, Units> Matrix<R,C, Units>::unserialize(char* source)
{
	Matrix<R,C, Units> a;
	std::memcpy(a.data(), source, serializedLength());
	return a;
}


template<int R, int C, typename Units>
inline void Matrix<R,C, Units>::copyTo(gsl_type* gslType) const
throw(std::logic_error)
{
	checkSize(gslType);
	copyToHelper(gslType);
}

template<int R, int C, typename Units>
inline void Matrix<R,C, Units>::copyFrom(const gsl_type* gslType)
throw(std::logic_error)
{
	checkSize(gslType);
	copyFromHelper(gslType);
}

template<int R, int C, typename Units>
inline void Matrix<R,C, Units>::copyFrom(const libconfig::Setting& setting)
{
	// special case: both column and row vectors can be initialized by a config row vector
	if (this->isVector()) {
		if (setting.getLength() != this->size()) {
			std::stringstream ss;
			ss << "(math::Matrix<>::copyFrom(libconfig::Setting)): The size of "
					"the configuration list must match the size of the Matrix. "
					"Expected size " << this->size() << ", got size " << setting.getLength() <<
					". Path: \"" << setting.getPath() << "\", Line: " <<
					setting.getSourceLine();
			throw std::runtime_error(ss.str());
		}

		if (setting[0].isNumber()) {  // if setting is a row vector
			for (int i = 0; i < this->size(); ++i) {
				(*this)[i] = barrett::detail::numericToDouble(setting[i]);
			}
			return;
		}  // else setting is a column vector or a matrix
	}


	// column vectors and matrices
	if (setting.getLength() != this->rows()) {
		std::stringstream ss;
		ss << "(math::Matrix<>::copyFrom(libconfig::Setting)): Wrong number of "
				"rows. Expected " << this->rows() << ", got " <<
				setting.getLength() << ". Path: \"" <<
				setting.getPath() << "\", Line: " << setting.getSourceLine();
		throw std::runtime_error(ss.str());
	}
	for (int i = 0; i < this->rows(); ++i) {
		if (setting[i].getLength() != this->cols()) {
			std::stringstream ss;
			ss << "(math::Matrix<>::copyFrom(libconfig::Setting)): Wrong number of "
					"columns. Expected " << this->cols() << ", got " <<
					setting[i].getLength() << ". Path: \"" <<
					setting[i].getPath() << "\", Line: " << setting[i].getSourceLine();
			throw std::runtime_error(ss.str());
		}
		for (int j = 0; j < this->cols(); ++j) {
			(*this)(i,j) = barrett::detail::numericToDouble(setting[i][j]);
		}
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
	if (Traits<type>::IsDynamic) {
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
void Matrix<R,C, Units>::checkSize(const gsl_vector* g) const
{
	if (g->size != static_cast<size_t>(this->size())) {
		std::stringstream ss;
		ss << "(math::Matrix<>::checkSize(gsl_vector*)): The size of the "
				"gsl_vector must match the size of the Matrix. Expected size "
				<< this->size() << ", got size " << g->size;
		throw std::logic_error(ss.str());
	}
}
template<int R, int C, typename Units>
void Matrix<R,C, Units>::checkSize(const gsl_matrix* g) const
{
	if (g->size1 != static_cast<size_t>(this->rows())
			||  g->size2 != static_cast<size_t>(this->cols())) {
		std::stringstream ss;
		ss << "(math::Matrix<>::checkSize(gsl_matrix*)): The size of the "
				"gsl_matrix must match the size of the Matrix. Expected size "
				<< this->rows() << "x" << this->cols() << ", got size "
				<< g->size1 << "x" << g->size2;
		throw std::logic_error(ss.str());
	}
}

template<int R, int C, typename Units>
void Matrix<R,C, Units>::copyToHelper(gsl_vector* g) const
{
	for (int i = 0; i < this->size(); ++i) {
		gsl_vector_set(g, i, (*this)[i]);
	}
}
template<int R, int C, typename Units>
void Matrix<R,C, Units>::copyToHelper(gsl_matrix* g) const
{
	for (int i = 0; i < this->rows(); ++i) {
		for (int j = 0; j < this->cols(); ++j) {
			gsl_matrix_set(g, i,j, (*this)(i,j));
		}
	}
}

template<int R, int C, typename Units>
void Matrix<R,C, Units>::copyFromHelper(const gsl_vector* g)
{
	for (int i = 0; i < this->size(); ++i) {
		(*this)[i] = gsl_vector_get(g, i);
	}
}
template<int R, int C, typename Units>
void Matrix<R,C, Units>::copyFromHelper(const gsl_matrix* g)
{
	for (int i = 0; i < this->rows(); ++i) {
		for (int j = 0; j < this->cols(); ++j) {
			(*this)(i,j) = gsl_matrix_get(g, i,j);
		}
	}
}


template<int R, int C, typename Units>
std::ostream& operator<< (std::ostream& os, const Matrix<R,C, Units>& a) {
	bool isVector = a.isVector();
	int maxRowIndex = a.rows() - 1;
	int maxColIndex = a.cols() - 1;

	os << "[";
	for (int i = 0; i < a.rows(); ++i) {
		for (int j = 0; j < a.cols(); ++j) {
			os << a(i,j);
			if ((isVector && i != maxRowIndex)  ||  j != maxColIndex) {
				os << ", ";
			}
		}
		if (!isVector  &&  i != maxRowIndex) {
			os << "\n ";
		}
	}
	os << "]";

	return os;
}


}
}
