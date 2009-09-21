/*
 * constant.h
 *
 *  Created on: Sep 4, 2009
 *      Author: dc
 */

#ifndef CONSTANT_H_
#define CONSTANT_H_


#include "abstract/system.h"
#include "../ca_macro.h"


namespace Systems {


template<typename T>
class Constant : public System {
// IO
public:		Output<T> output;
protected:	typename System::Output<T>::Value* outputValue;


public:
	explicit Constant(T value) :
		output(value, &outputValue) {}
	virtual ~Constant() {}

protected:
	virtual void operate() {}  // no operation, only output!

private:
	DISALLOW_COPY_AND_ASSIGN(Constant);
};


}


#endif /* CONSTANT_H_ */
