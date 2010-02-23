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
#include "./abstract/single_io.h"


namespace barrett {
namespace systems {


template <typename T>
class ExposedOutput : public System, public SingleOutput<T> {
public:
	ExposedOutput() :
		SingleOutput<T>(this) {}
	explicit ExposedOutput(const T& initialValue) :
		SingleOutput<T>(this)
	{
		this->outputValue->setValue(initialValue);
	}

	void setValue(const T& value) {
		SCOPED_LOCK(getEmMutex());
		this->outputValue->setValue(value);
	}
	void setValueUndefined() {
		SCOPED_LOCK(getEmMutex());
		this->outputValue->setValueUndefined();
	}
	void delegateTo(const System::Output<T>& delegate) {
		SCOPED_LOCK(getEmMutex());
		this->outputValue->delegateTo(delegate);
	}

protected:
	virtual void operate() {  /* do nothing */  }

private:
	DISALLOW_COPY_AND_ASSIGN(ExposedOutput);
};


}
}
#endif /* EXPOSED_OUTPUT_H_ */
