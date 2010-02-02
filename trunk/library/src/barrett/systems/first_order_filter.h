/*
 * first_order_filter.h
 *
 *  Created on: Nov 18, 2009
 *      Author: dc
 */

#ifndef FIRST_ORDER_FILTER_H_
#define FIRST_ORDER_FILTER_H_


#include "../detail/ca_macro.h"
#include "./abstract/single_io.h"


namespace barrett {
namespace systems {


// TODO(dc): test!
// TODO(dc): add a configuration file interface

template<typename T>
class FirstOrderFilter : public SingleIO<T, T> {
public:
	explicit FirstOrderFilter(bool updateEveryExecutionCycle = false);

	void setSamplePeriod(double timeStep);
	void setLowPass(T omega_p, T dcGain);
	void setHighPass(T omega_p, T dcGain);
	void setZPK(T omega_z, T omega_p, T dcGain);
	void setIntegrator(T gain);
	void setParameters(T a, T b, T c);

	virtual void setExecutionManager(ExecutionManager* newEm);

protected:
	void updateCoefficients();
	virtual void operate();

	T a, b, c;
	double T_s;

	T c1, c2, c3;
	T y_0, y_1, x_0, x_1;

private:
	void getSamplePeriodFromEM();

	DISALLOW_COPY_AND_ASSIGN(FirstOrderFilter);
};


}
}


// include template definitions
#include "./detail/first_order_filter-inl.h"


#endif /* FIRST_ORDER_FILTER_H_ */
