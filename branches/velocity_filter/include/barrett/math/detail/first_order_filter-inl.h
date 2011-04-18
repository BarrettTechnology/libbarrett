/*
 * first_order_filter-inl.h
 *
 *  Created on: Apr 1, 2010
 *      Author: dc
 */


#include <string>


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
FirstOrderFilter<T, MathTraits>::FirstOrderFilter(const libconfig::Setting& setting) :
	a(MT::zero()), b(MT::zero()), c(MT::zero()), T_s(0.0),
	c1(MT::zero()), c2(MT::zero()), c3(MT::zero()),
	y_0(MT::zero()), y_1(MT::zero()), x_0(MT::zero()), x_1(MT::zero())
{
	setFromConfig(setting);
}

template<typename T, typename MathTraits>
inline void FirstOrderFilter<T, MathTraits>::setSamplePeriod(double timeStep)
{
	T_s = timeStep;
	updateCoefficients();
}

template<typename T, typename MathTraits>
void FirstOrderFilter<T, MathTraits>::setFromConfig(const libconfig::Setting& setting)
{
	// TODO(dc): test!
	T omega_p(MT::zero());
	if (setting.exists("omega_p")) {
		omega_p = setting["omega_p"];
	}
	T omega_z(MT::zero());
	if (setting.exists("omega_z")) {
		omega_z = setting["omega_z"];
	}
	T a(MT::zero());
	if (setting.exists("a")) {
		a = setting["a"];
	}
	T b(MT::zero());
	if (setting.exists("b")) {
		b = setting["b"];
	}
	T c(MT::zero());
	if (setting.exists("c")) {
		c = setting["c"];
	}

	if (setting.exists("type")) {
		std::string type = setting["type"];
		switch (type[0]) {
		case 'l':  // low_pass
			if (setting.exists("dc_gain")) {
				setLowPass(omega_p, T(setting["dc_gain"]));
			} else {
				setLowPass(omega_p);
			}
			break;
		case 'h':  // high_pass
			if (setting.exists("hf_gain")) {
				setHighPass(omega_p, T(setting["hf_gain"]));
			} else {
				setHighPass(omega_p);
			}
			break;
		case 'z':  // zpk
			if (setting.exists("dc_gain")) {
				setZPK(omega_z, omega_p, T(setting["dc_gain"]));
			} else {
				setZPK(omega_z, omega_p);
			}
			break;
		case 'i':  // integrator
			if (setting.exists("gain")) {
				setIntegrator(T(setting["gain"]));
			} else {
				setIntegrator();
			}
			break;
		case 'p':  // parameters
			setParameters(a, b, c);
			break;
		}
	}
}

template<typename T, typename MathTraits>
void FirstOrderFilter<T, MathTraits>::setLowPass(const T& omega_p, const T& dcGain)
{
	a = MT::zero();
	b = MT::mult(dcGain, omega_p);
	c = omega_p;

	updateCoefficients();
}

template<typename T, typename MathTraits>
void FirstOrderFilter<T, MathTraits>::setHighPass(const T& omega_p, const T& hfGain)
{
	a = hfGain;
	b = MT::zero();
	c = omega_p;

	updateCoefficients();
}

template<typename T, typename MathTraits>
void FirstOrderFilter<T, MathTraits>::setZPK(const T& omega_z, const T& omega_p, const T& dcGain)
{
	a = MT::div(MT::mult(dcGain, omega_p), omega_z);
	b = MT::mult(dcGain, omega_p);
	c = omega_p;

	updateCoefficients();
}

template<typename T, typename MathTraits>
void FirstOrderFilter<T, MathTraits>::setIntegrator(const T& gain)
{
	a = MT::zero();
	b = gain;
	c = MT::zero();

	updateCoefficients();
}

template<typename T, typename MathTraits>
void FirstOrderFilter<T, MathTraits>::setParameters(const T& a_, const T& b_, const T& c_)
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
inline const T& FirstOrderFilter<T, MathTraits>::eval(const T& x_0)
{
	// y_0 = c1*y_1 + c2*x_0 - c3*x_1;
	y_0 = MT::sub(MT::add(MT::mult(c1,y_1), MT::mult(c2,x_0)), MT::mult(c3,x_1));

	y_1 = y_0;
	x_1 = x_0;

	return y_0;
}


}
}
