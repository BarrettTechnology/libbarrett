/*
 * exposed_output.h
 *
 *  Created on: Nov 13, 2009
 *      Author: dc
 */

#ifndef EXPOSED_OUTPUT_H_
#define EXPOSED_OUTPUT_H_


#include "../detail/ca_macro.h"
#include "./abstract/system.h"


namespace barrett {
namespace systems {


template <typename T>
class ExposedOutput {
// IO
public:		System::Output<T> output;
protected:	typename System::Output<T>::Value* outputValue;


public:
	ExposedOutput() :
		output(&outputValue) {}
	explicit ExposedOutput(const T& initialValue) :
		output(initialValue, &outputValue) {}

	void setValue(const T& value) {
		outputValue->setValue(value);
	}
	void setValueUndefined() {
		outputValue->setValueUndefined();
	}
	void delegateTo(const System::Output<T>& delegate) {
		outputValue->delegateTo(delegate);
	}

private:
	DISALLOW_COPY_AND_ASSIGN(ExposedOutput);
};


}
}
#endif /* EXPOSED_OUTPUT_H_ */
