/*
 * constant.h
 *
 *  Created on: Sep 4, 2009
 *      Author: dc
 */

#ifndef BARRETT_SYSTEMS_CONSTANT_H_
#define BARRETT_SYSTEMS_CONSTANT_H_


#include <barrett/detail/ca_macro.h>
#include <barrett/systems/abstract/system.h>
#include <barrett/systems/abstract/single_io.h>


namespace barrett {
namespace systems {

// TODO(dc): add a configuration file interface

template<typename T>
class Constant : public System, public SingleOutput<T> {
public:
	explicit Constant(const T& value) :
		SingleOutput<T>(this)
	{
		this->outputValue->setValue(value);
	}
	virtual ~Constant() {}

protected:
	virtual void operate() {  /* do nothing */  }

private:
	DISALLOW_COPY_AND_ASSIGN(Constant);
};


}
}


#endif /* BARRETT_SYSTEMS_CONSTANT_H_ */
