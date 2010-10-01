/*
 * constant.h
 *
 *  Created on: Sep 4, 2009
 *      Author: dc
 */

#ifndef CONSTANT_H_
#define CONSTANT_H_


#include "../detail/ca_macro.h"
#include "./abstract/system.h"
#include "./abstract/single_io.h"


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


#endif /* CONSTANT_H_ */
