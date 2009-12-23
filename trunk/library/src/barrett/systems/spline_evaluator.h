/*
 * spline_evaluator.h
 *
 *  Created on: Dec 17, 2009
 *      Author: dc
 */

#ifndef SPLINE_EVALUATOR_H_
#define SPLINE_EVALUATOR_H_


#include "../detail/ca_macro.h"
#include "./abstract/single_io.h"
#include "../math/spline.h"


namespace barrett {
namespace systems {


template<typename OutputType>
class SplineEvaluator : public SingleIO<double, OutputType> {
public:
	explicit SplineEvaluator(const math::Spline<OutputType>& spline) :
		SingleIO<double, OutputType>(), s(spline) {}

protected:
	virtual void operate() {
		this->outputValue->setValue(s.eval(this->input.getValue()));
	}

	const math::Spline<OutputType>& s;

private:
	DISALLOW_COPY_AND_ASSIGN(SplineEvaluator);
};


}
}


#endif /* SPLINE_EVALUATOR_H_ */
