/*
 * spline_evaluator.h
 *
 *  Created on: Dec 17, 2009
 *      Author: dc
 */

#ifndef SPLINE_EVALUATOR_H_
#define SPLINE_EVALUATOR_H_


#include "abstract/single_io.h"


namespace barrett {
namespace systems {


template<typename OutputType>
class SplineEvaluator : public SingleIO<double, OutputType> {

};


}
}


#endif /* SPLINE_EVALUATOR_H_ */
