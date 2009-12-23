/*
 * trapezoidal_velocity_profile_evaluator.h
 *
 *  Created on: Dec 23, 2009
 *      Author: dc
 */

#ifndef TRAPEZOIDAL_VELOCITY_PROFILE_EVALUATOR_H_
#define TRAPEZOIDAL_VELOCITY_PROFILE_EVALUATOR_H_


#include "../detail/ca_macro.h"
#include "./abstract/single_io.h"
#include "../math/trapezoidal_velocity_profile.h"


namespace barrett {
namespace systems {


class TrapezoidalVelocityProfileEvaluator : public SingleIO<double, double> {
public:
	explicit TrapezoidalVelocityProfileEvaluator(
			const math::TrapezoidalVelocityProfile& profile) :
		SingleIO<double, double>(), p(profile) {}
	virtual ~TrapezoidalVelocityProfileEvaluator() {}

protected:
	virtual void operate() {
		this->outputValue->setValue(p.eval(this->input.getValue()));
	}

	const math::TrapezoidalVelocityProfile& p;

private:
	DISALLOW_COPY_AND_ASSIGN(TrapezoidalVelocityProfileEvaluator);
};


}
}


#endif /* TRAPEZOIDAL_VELOCITY_PROFILE_EVALUATOR_H_ */
