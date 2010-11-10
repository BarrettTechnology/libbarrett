/*
 * exposed_output.h
 *
 *  Created on: Nov 13, 2009
 *      Author: dc
 */

#ifndef BARRETT_SYSTEMS_EXPOSED_OUTPUT_H_
#define BARRETT_SYSTEMS_EXPOSED_OUTPUT_H_


#include <barrett/detail/ca_macro.h>
#include <barrett/thread/abstract/mutex.h>
#include <barrett/systems/abstract/system.h>
#include <barrett/systems/abstract/single_io.h>


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
		BARRETT_SCOPED_LOCK(getEmMutex());
		this->outputValue->setValue(value);
	}
	void setValueUndefined() {
		BARRETT_SCOPED_LOCK(getEmMutex());
		this->outputValue->setValueUndefined();
	}
	void delegateTo(const System::Output<T>& delegate) {
		BARRETT_SCOPED_LOCK(getEmMutex());
		this->outputValue->delegateTo(delegate);
	}

protected:
	virtual void operate() {  /* do nothing */  }

private:
	DISALLOW_COPY_AND_ASSIGN(ExposedOutput);
};


}
}


#endif /* BARRETT_SYSTEMS_EXPOSED_OUTPUT_H_ */
