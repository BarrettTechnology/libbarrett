/*
 * constant.h
 *
 *  Created on: Sep 4, 2009
 *      Author: dc
 */

#ifndef CONSTANT_H_
#define CONSTANT_H_


#include "abstract/system.h"
#include "../detail/ca_macro.h"


namespace barrett {
namespace systems {


template<typename T>
class Constant {
// IO
public:		System::Output<T> output;
protected:	typename System::Output<T>::Value* outputValue;


public:
	explicit Constant(T value) :
		output(value, &outputValue) {}
	virtual ~Constant() {}

private:
	DISALLOW_COPY_AND_ASSIGN(Constant);
};


}
}


#endif /* CONSTANT_H_ */
