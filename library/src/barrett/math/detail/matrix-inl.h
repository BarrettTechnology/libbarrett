/*
 * matrix-inl.h
 *
 *  Created on: Jan 13, 2010
 *      Author: dc
 */

#ifndef MATRIXINL_H_
#define MATRIXINL_H_


#include <stdexcept>
#include <sstream>

#include <gsl/gsl_matrix.h>
#include <boost/array.hpp>


namespace barrett {
namespace math {


template<size_t M, size_t N>
Matrix<M,N>::Matrix() :
	gslMatrix()
{
	this->assign(0.0);
	initGslMatrix();
}

template<size_t M, size_t N>
Matrix<M,N>::Matrix(const gsl_matrix* mat) :
	gslMatrix()
{
	initGslMatrix();
	copyFrom(mat);
}

template<size_t M, size_t N>
Matrix<M,N>::Matrix(const Matrix& m) :
	boost::array<double, SIZE>(m), gslMatrix(m.gslMatrix)
{
	initGslMatrix();
}

template<size_t M, size_t N>
Matrix<M,N>::~Matrix()
{
}


template<size_t M, size_t N>
void Matrix<M,N>::copyTo(gsl_matrix* mat) const
throw(std::logic_error)
{
	if (mat->size1 != M  ||  mat->size2 != N) {
		std::stringstream ss;
		ss << "(math::Matrix<>::copyTo(gsl_matrix*)): The size of the "
				"gsl_matrix must match the size of the Matrix. Expected size "
				<< M << "x" << N << ", got size "
				<< mat->size1 << "x" << mat->size2;
		throw std::logic_error(ss.str());
	}

	gsl_matrix_memcpy(mat, asGslMatrix());
}

template<size_t M, size_t N>
void Matrix<M,N>::copyFrom(const gsl_matrix* mat)
throw(std::logic_error)
{
	if (mat->size1 != M  ||  mat->size2 != N) {
		std::stringstream ss;
		ss << "(math::Matrix<>::copyFrom(gsl_matrix*)): The size of the "
				"gsl_matrix must match the size of the Matrix. Expected size "
				<< M << "x" << N << ", got size "
				<< mat->size1 << "x" << mat->size2;
		throw std::logic_error(ss.str());
	}

	gsl_matrix_memcpy(asGslMatrix(), mat);
}

template<size_t M, size_t N>
inline gsl_matrix* Matrix<M,N>::asGslMatrix()
{
	return &(this->gslMatrix);
}

template<size_t M, size_t N>
inline const gsl_matrix* Matrix<M,N>::asGslMatrix() const
{
	return &(this->gslMatrix);
}


template<size_t M, size_t N>
inline double Matrix<M,N>::operator() (size_t r, size_t c) const
{
	return this->at(r*ROWS + c);
}

template<size_t M, size_t N>
inline double& Matrix<M,N>::operator() (size_t r, size_t c)
{
	return this->at(r*ROWS + c);
}

template<size_t M, size_t N>
inline void Matrix<M,N>::initGslMatrix() {
	gslMatrix.size1 = ROWS;
	gslMatrix.size2 = COLUMNS;
	gslMatrix.tda = COLUMNS;
	gslMatrix.data = this->c_array();
	gslMatrix.block = 0;
	gslMatrix.owner = 0;
}


}
}


#endif /* MATRIXINL_H_ */
