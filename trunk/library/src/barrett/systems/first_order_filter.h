/*
 * first_order_filter.h
 *
 *  Created on: Nov 18, 2009
 *      Author: dc
 */

#ifndef BARRETT_SYSTEMS_FIRST_ORDER_FILTER_H_
#define BARRETT_SYSTEMS_FIRST_ORDER_FILTER_H_


#include "../detail/ca_macro.h"
#include "../math/first_order_filter.h"
#include "./abstract/single_io.h"


namespace barrett {
namespace systems {


// TODO(dc): test!
// TODO(dc): add a configuration file interface

template<typename T>
class FirstOrderFilter : public SingleIO<T, T>, public math::FirstOrderFilter<T> {
public:
	explicit FirstOrderFilter(bool updateEveryExecutionCycle = false);
	virtual ~FirstOrderFilter() {}

	virtual void setExecutionManager(ExecutionManager* newEm);

protected:
	virtual void operate();

private:
	void getSamplePeriodFromEM();

	DISALLOW_COPY_AND_ASSIGN(FirstOrderFilter);
};


}
}


// include template definitions
#include "./detail/first_order_filter-inl.h"


#endif /* BARRETT_SYSTEMS_FIRST_ORDER_FILTER_H_ */
