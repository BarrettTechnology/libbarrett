/*
 * conversion.h
 *
 *  Created on: Oct 29, 2009
 *      Author: dc
 */

#ifndef BARRETT_SYSTEMS_ABSTRACT_CONVERSION_H_
#define BARRETT_SYSTEMS_ABSTRACT_CONVERSION_H_


#include <barrett/detail/ca_macro.h>
#include <barrett/systems/abstract/system.h>


namespace barrett {
namespace systems {


// objects implementing this interface can be manipulated by a
// Converter
template<typename OutputType>
class Conversion {
public:
	Conversion() {}
	virtual ~Conversion() {}

	virtual System::AbstractInput* getConversionInput() = 0;
	virtual System::Output<OutputType>& getConversionOutput() = 0;

private:
	DISALLOW_COPY_AND_ASSIGN(Conversion);
};


}
}


#endif /* BARRETT_SYSTEMS_ABSTRACT_CONVERSION_H_ */
