/*
 * callback.h
 *
 *  Created on: Nov 16, 2009
 *      Author: dc
 */

#ifndef BARRETT_SYSTEMS_CALLBACK_H_
#define BARRETT_SYSTEMS_CALLBACK_H_


#include <boost/function.hpp>
#include <barrett/detail/ca_macro.h>
#include <barrett/systems/abstract/single_io.h>


namespace barrett {
namespace systems {


template<typename InputType, typename OutputType = InputType>
class Callback : public SingleIO<InputType, OutputType> {
public:
	typedef boost::function<OutputType (const InputType&)> callback_type;

	explicit Callback(callback_type operateCallback, bool updateEveryExecutionCycle = false) :
		SingleIO<InputType, OutputType>(updateEveryExecutionCycle), callback(operateCallback) {}
	virtual ~Callback() {}

protected:
	virtual void operate() {
		this->outputValue->setValue(callback(this->input.getValue()));
	}

	callback_type callback;

private:
	DISALLOW_COPY_AND_ASSIGN(Callback);
};


}
}


#endif /* BARRETT_SYSTEMS_CALLBACK_H_ */
