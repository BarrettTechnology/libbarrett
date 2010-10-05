/*
 * converter-inl.h
 *
 *  Created on: Oct 29, 2009
 *      Author: dc
 */


#include <utility>
#include <list>
#include <stdexcept>

#include <barrett/detail/stl_utils.h>
#include <barrett/systems/abstract/system.h>
#include <barrett/systems/helpers.h>
#include <barrett/systems/abstract/conversion.h>


// I think RTTI is useful here to keep syntax clean and support flexibility and
// extensibility. Also, Visitor and multiple-dispatch don't really fit in this
// usage.
// FIXME(dc): Is there a clean way to get this behavior without RTTI?

namespace barrett {
namespace systems {


template<typename OutputType>
inline Converter<OutputType>::~Converter()
{
	barrett::detail::purge(conversions);
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
		System::Output<T>& output)
throw(std::invalid_argument)
{
	typename std::list<Conversion<OutputType>*>::iterator i;
	for (i = conversions.begin(); i != conversions.end(); ++i) {
		if (connectInputTo(output, *i)) {
			return;
		}
	}

	throw std::invalid_argument(
			"(systems::Converter::connectInputTo): No systems::Conversion is "
			"registered that matches the output's type.");
}

template<typename OutputType>
template<typename T>
bool Converter<OutputType>::connectInputTo(
		System::Output<T>& output,
		Conversion<OutputType>* conversion)
{
	System::Input<T>* input = dynamic_cast<System::Input<T>*>(  //NOLINT: see RTTI note above
			conversion->getConversionInput() );

	if (input == NULL) {
		return false;
	}

	forceConnect(output, *input);
	this->outputValue->delegateTo(conversion->getConversionOutput());

	return true;
}

template<typename OutputType>
void Converter<OutputType>::disconnectInput()
{
	this->outputValue->undelegate();
}


}
}
