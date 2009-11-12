/*
 * converter.h
 *
 *  Created on: Oct 29, 2009
 *      Author: dc
 */

#ifndef CONVERTER_H_
#define CONVERTER_H_


#include <list>
#include <utility>

#include "../detail/ca_macro.h"
#include "./abstract/system.h"
#include "./abstract/conversion.h"


namespace barrett {
namespace systems {


template<typename OutputType>
class Converter {
//IO
public:		System::Output<OutputType> output;
protected:	typename System::Output<OutputType>::Value* outputValue;


public:
	Converter() :
		output(&outputValue), conversions() {}
	explicit Converter(const OutputType& initialOutputValue) :
		output(initialOutputValue, &outputValue), conversions() {}
	virtual ~Converter();

	void registerConversion(
			Conversion<OutputType>* conversion);

	template<typename T>
	void connectInputTo(System::Output<T>& referenceOutput)  //NOLINT: non-const reference for syntax
	throw(std::invalid_argument);

	template<typename T>
	bool connectInputTo(System::Output<T>& referenceOutput,  //NOLINT: non-const reference for syntax
			Conversion<OutputType>* conversion);

protected:
	std::list<Conversion<OutputType>*> conversions;

private:
	DISALLOW_COPY_AND_ASSIGN(Converter);
};


}
}


// include template definitions
#include "./detail/converter-inl.h"


#endif /* CONVERTER_H_ */
