/*
 * matrix.h
 *
 *  Created on: Jan 13, 2010
 *      Author: dc
 */

#ifndef MATRIX_H_
#define MATRIX_H_


#include <stdexcept>

#include <gsl/gsl_matrix.h>
#include <boost/array.hpp>


namespace barrett {
namespace math {


template<size_t M, size_t N>
class Matrix : protected boost::array<double, M*N> {  // NOTE: inheritance is not public!
public:
	static const size_t ROWS = M;
	static const size_t COLUMNS = N;
	static const size_t SIZE = ROWS * COLUMNS;

	Matrix();
	Matrix(const gsl_matrix* mat);
	Matrix(const Matrix& m);
	~Matrix();

	void copyTo(gsl_matrix* mat) const throw(std::logic_error);
	void copyFrom(const gsl_matrix* mat) throw(std::logic_error);

	gsl_matrix* asGslMatrix();
	const gsl_matrix* asGslMatrix() const;

	double operator() (size_t r, size_t c) const;
	double& operator() (size_t r, size_t c);

protected:
	void initGslMatrix();
	gsl_matrix gslMatrix;
};


}
}


// include template definitions
#include "./detail/matrix-inl.h"


#endif /* MATRIX_H_ */
