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


namespace barrett {
namespace systems {


template<typename T>
class Constant : public System {
// IO
public:		Output<T> output;
protected:	typename Output<T>::Value* outputValue;


public:
	explicit Constant(const T& value) :
		output(this, &outputValue)
	{
		outputValue->setValue(value);
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
