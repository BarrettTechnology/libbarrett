/*
 * first_order_filter.h
 *
 *  Created on: Apr 1, 2010
 *      Author: dc
 */

#ifndef BARRETT_MATH_FIRST_ORDER_FILTER_H_
#define BARRETT_MATH_FIRST_ORDER_FILTER_H_


#include "../detail/ca_macro.h"
#include "./traits.h"


namespace barrett {
namespace  math {


template<typename T, typename MathTraits = math::Traits<T> >
class FirstOrderFilter {
protected:
	typedef MathTraits MT;

public:
	explicit FirstOrderFilter(double timeStep = 0.0);

	void setSamplePeriod(double timeStep);
	void setLowPass(T omega_p, T dcGain = T(1.0));
	void setHighPass(T omega_p, T dcGain = T(1.0));
	void setZPK(T omega_z, T omega_p, T dcGain = T(1.0));
	void setIntegrator(T gain = T(1.0));
	void setParameters(T a, T b, T c);

	T eval(const T& input);

	typedef T result_type;  ///< For use with boost::bind().
	result_type operator() (const T& input) {
		return eval(input);
	}

protected:
	void updateCoefficients();

	T a, b, c;
	double T_s;

	T c1, c2, c3;
	T y_0, y_1, x_0, x_1;

private:
	// TODO(dc): write a real copy constructor and assignment operator?
	DISALLOW_COPY_AND_ASSIGN(FirstOrderFilter);
};


}
}


// include template definitions
#include "./detail/first_order_filter-inl.h"


#endif /* BARRETT_MATH_FIRST_ORDER_FILTER_H_ */
