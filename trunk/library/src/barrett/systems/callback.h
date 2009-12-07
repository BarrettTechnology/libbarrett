/*
 * callback.h
 *
 *  Created on: Nov 16, 2009
 *      Author: dc
 */

#ifndef CALLBACK_H_
#define CALLBACK_H_


#include "../detail/ca_macro.h"
#include "./abstract/single_io.h"


namespace barrett {
namespace systems {


// TODO(dc): generalize to take functors

template<typename InputType, typename OutputType = InputType>
class Callback : public SingleIO<InputType, OutputType> {
public:
	typedef OutputType (*callback_type)(const InputType&);

	explicit Callback(callback_type operateCallback) :
		SingleIO<InputType, OutputType>(), callback(operateCallback) {}
	Callback(callback_type operateCallback,
			const OutputType& initialOutputValue) :
		SingleIO<InputType, OutputType>(initialOutputValue),
		callback(operateCallback) {}
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


#endif /* CALLBACK_H_ */
