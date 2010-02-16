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
#include "./abstract/single_io.h"
#include "./abstract/conversion.h"


namespace barrett {
namespace systems {


template<typename OutputType>
class Converter : public System, public SingleOutput<OutputType> {
public:
	Converter() :
		SingleOutput<OutputType>(this), conversions() {}
	virtual ~Converter();

	void registerConversion(
			Conversion<OutputType>* conversion);

	template<typename T>
	void connectInputTo(System::Output<T>& output)  //NOLINT: non-const reference for syntax
	throw(std::invalid_argument);

	template<typename T>
	bool connectInputTo(System::Output<T>& output,  //NOLINT: non-const reference for syntax
			Conversion<OutputType>* conversion);

	void disconnectInput();

protected:
	virtual void operate() {  /* TODO(dc): invalidate outputs */  }

	std::list<Conversion<OutputType>*> conversions;

private:
	DISALLOW_COPY_AND_ASSIGN(Converter);
};


}
}


// include template definitions
#include "./detail/converter-inl.h"


#endif /* CONVERTER_H_ */
