/*
 * first_order_filter.h
 *
 *  Created on: Apr 1, 2010
 *      Author: dc
 */

#ifndef BARRETT_MATH_FIRST_ORDER_FILTER_H_
#define BARRETT_MATH_FIRST_ORDER_FILTER_H_


#include <Eigen/Core>
#include <libconfig.h++>

#include <barrett/detail/ca_macro.h>
#include <barrett/math/traits.h>


namespace barrett {
namespace  math {


template<typename T, typename MathTraits = math::Traits<T> >
class FirstOrderFilter {
protected:
	typedef MathTraits MT;

public:
	explicit FirstOrderFilter(double timeStep = 0.0);
	explicit FirstOrderFilter(const libconfig::Setting& setting);

	void setSamplePeriod(double timeStep);
	void setFromConfig(const libconfig::Setting& setting);
	void setLowPass(const T& omega_p, const T& dcGain = T(1.0));
	void setHighPass(const T& omega_p, const T& hfGain = T(1.0));
	void setZPK(const T& omega_z, const T& omega_p, const T& dcGain = T(1.0));
	void setIntegrator(const T& gain = T(1.0));
	void setParameters(const T& a, const T& b, const T& c);

	const T& eval(const T& input);

	typedef const T& result_type;  ///< For use with boost::bind().
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

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF(MT::RequiresAlignment)
};


}
}


// include template definitions
#include <barrett/math/detail/first_order_filter-inl.h>


#endif /* BARRETT_MATH_FIRST_ORDER_FILTER_H_ */
