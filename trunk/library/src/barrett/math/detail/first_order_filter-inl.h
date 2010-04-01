/*
 * first_order_filter-inl.h
 *
 *  Created on: Apr 1, 2010
 *      Author: dc
 */


namespace barrett {
namespace math {


template<typename T, typename MathTraits>
FirstOrderFilter<T, MathTraits>::FirstOrderFilter(double timeStep) :
	a(MT::zero()), b(MT::zero()), c(MT::zero()), T_s(timeStep),
	c1(MT::zero()), c2(MT::zero()), c3(MT::zero()),
	y_0(MT::zero()), y_1(MT::zero()), x_0(MT::zero()), x_1(MT::zero())
{
}

template<typename T, typename MathTraits>
inline void FirstOrderFilter<T, MathTraits>::setSamplePeriod(double timeStep)
{
	T_s = timeStep;
	updateCoefficients();
}

template<typename T, typename MathTraits>
void FirstOrderFilter<T, MathTraits>::setLowPass(T omega_p, T dcGain)
{
	a = MT::zero();
	b = MT::mult(dcGain, omega_p);
	c = omega_p;

	updateCoefficients();
}

template<typename T, typename MathTraits>
void FirstOrderFilter<T, MathTraits>::setHighPass(T omega_p, T dcGain)
{
	a = dcGain;
	b = MT::zero();
	c = omega_p;

	updateCoefficients();
}

template<typename T, typename MathTraits>
void FirstOrderFilter<T, MathTraits>::setZPK(T omega_z, T omega_p, T dcGain)
{
	a = MT::div(MT::mult(dcGain, omega_p), omega_z);
	b = MT::mult(dcGain, omega_p);
	c = omega_p;

	updateCoefficients();
}

template<typename T, typename MathTraits>
void FirstOrderFilter<T, MathTraits>::setIntegrator(T gain)
{
	a = MT::zero();
	b = gain;
	c = MT::zero();

	updateCoefficients();
}

template<typename T, typename MathTraits>
void FirstOrderFilter<T, MathTraits>::setParameters(T a_, T b_, T c_)
{
	a = a_;
	b = b_;
	c = c_;

	updateCoefficients();
}


// filter parameters:
//         a*s + b
// H(s) = ---------
//          s + c
//
// difference equation:
//             1                    a + b*T_s                  a
// y[n] = ----------- * y[n-1]  +  ----------- * x[n]  -  ----------- * x[n-1]
//         1 + c*T_s                1 + c*T_s              1 + c*T_s
template<typename T, typename MathTraits>
inline void FirstOrderFilter<T, MathTraits>::updateCoefficients()
{
	T den = MT::add(1.0, MT::mult(c,T_s));

	c1 = MT::div(1.0, den);
	c2 = MT::div(MT::add(a, MT::mult(b, T_s)), den);
	c3 = MT::div(a, den);
}

template<typename T, typename MathTraits>
inline T FirstOrderFilter<T, MathTraits>::eval(const T& x_0)
{
	// y_0 = c1*y_1 + c2*x_0 - c3*x_1;
	y_0 = MT::sub(MT::add(MT::mult(c1,y_1), MT::mult(c2,x_0)), MT::mult(c3,x_1));

	y_1 = y_0;
	x_1 = x_0;

	return y_0;
}


}
}
