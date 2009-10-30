/*
 * converter-inl.h
 *
 *  Created on: Oct 29, 2009
 *      Author: dc
 */


#include <utility>
#include <list>
#include <stdexcept>

#include "../../detail/purge.h"
#include "../abstract/system.h"
#include "../helpers.h"
#include "../abstract/conversion.h"
#include "../converter.h"


// I think RTTI is useful here to keep syntax clean and support flexibility and
// extensibility. Also, Visitor and multiple-dispatch don't really fit in this
// usage.
// FIXME(dc): Is there a clean way to get this behavior without RTTI?

namespace barrett {
namespace systems {

template<typename OutputType>
inline Converter<OutputType>::~Converter()
{
	purge(conversions);
}

template<typename OutputType>
inline void Converter<OutputType>::registerConversion(
		Conversion<OutputType>* conversion)
{
	conversions.push_front(conversion);
}

template<typename OutputType>
template<typename T>
void Converter<OutputType>::connectInputTo(
		System::Output<T>& referenceOutput)
throw(std::invalid_argument)
{
	typename std::list<Conversion<OutputType>*>::iterator i;
	for (i = conversions.begin(); i != conversions.end(); ++i) {
		if (connectInputTo(referenceOutput, *i)) {
			return;
		}
	}

	throw std::invalid_argument(
			"(systems::Converter::connectInputTo): "
			"No Convertable is registered that matches "
			"referenceOutput's type");
}

template<typename OutputType>
template<typename T>
bool Converter<OutputType>::connectInputTo(
		System::Output<T>& referenceOutput,
		Conversion<OutputType>* conversion)
{
	System::Input<T>* referenceInput = dynamic_cast<Input<T>*>(  //NOLINT: see RTTI note above
			conversion->getConversionInput() );

	if (referenceInput == NULL) {
		return false;
	}

	forceConnect(referenceOutput, *referenceInput);
	forceConnect(conversion->getConversionOutput(),
			shaddowInput);

	return true;
}


template<typename OutputType>
void Converter<OutputType>::operate()
{
	outputValue->setValue(shaddowInput.getValue());
}


}
}
