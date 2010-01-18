/*
 * exposed_output.h
 *
 *  Created on: Nov 13, 2009
 *      Author: dc
 */

#ifndef EXPOSED_OUTPUT_H_
#define EXPOSED_OUTPUT_H_


#include "../detail/ca_macro.h"
#include "../thread/abstract/mutex.h"
#include "./abstract/system.h"


namespace barrett {
namespace systems {


template <typename T>
class ExposedOutput : public System {
// IO
public:		System::Output<T> output;
protected:	typename System::Output<T>::Value* outputValue;


public:
	ExposedOutput() :
		output(this, &outputValue) {}
	explicit ExposedOutput(const T& initialValue) :
		output(&outputValue)
	{
		outputValue->setValue(initialValue);
	}

	void setValue(const T& value) {
		SCOPED_LOCK(getEmMutex());
		outputValue->setValue(value);
	}
	void setValueUndefined() {
		SCOPED_LOCK(getEmMutex());
		outputValue->setValueUndefined();
	}
	void delegateTo(const System::Output<T>& delegate) {
		SCOPED_LOCK(getEmMutex());
		outputValue->delegateTo(delegate);
	}

protected:
	virtual void operate() {  /* do nothing */  }

private:
	DISALLOW_COPY_AND_ASSIGN(ExposedOutput);
};


}
}
#endif /* EXPOSED_OUTPUT_H_ */
