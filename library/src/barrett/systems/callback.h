/*
 * callback.h
 *
 *  Created on: Nov 16, 2009
 *      Author: dc
 */

#ifndef CALLBACK_H_
#define CALLBACK_H_


#include "./abstract/single_io.h"


namespace barrett {
namespace systems {


// TODO(dc): expand to take functors

template<typename InputType, typename OutputType = InputType>
class Callback : public SingleIO<InputType, OutputType> {
public:
	typedef OutputType (*callback_type)(const InputType&);

	Callback(callback_type operateCallback) :
		SingleIO<InputType, OutputType>(), callback(operateCallback) {}
	explicit Callback(callback_type operateCallback,
			const OutputType& initialOutputValue) :
		SingleIO<InputType, OutputType>(initialOutputValue),
		callback(operateCallback) {}
	virtual ~Callback() {}

protected:
	virtual void operate() {
		if (this->input.valueDefined()) {
			this->outputValue->setValue(callback(this->input.getValue()));
		} else {
			this->outputValue->setValueUndefined();
		}
	}

	callback_type callback;
};


}
}


#endif /* CALLBACK_H_ */
