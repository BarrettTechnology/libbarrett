/*
 * io_conversion.h
 *
 *  Created on: Oct 30, 2009
 *      Author: dc
 */

#ifndef IO_CONVERSION_H_
#define IO_CONVERSION_H_


#include "./abstract/conversion.h"
#include "./abstract/system.h"


namespace barrett {
namespace systems {


template<typename InputType, typename OutputType>
class IOConversion : public Conversion<OutputType> {
public:
	IOConversion(System::Input<InputType>& conversionInput,  //NOLINT: non-const reference
			System::Output<OutputType>& conversionOutput) :
		input(conversionInput), output(conversionOutput) {}
	virtual ~IOConversion() {}

	virtual System::Input<InputType>* getConversionInput() {
		return &input;
	}
	virtual System::Output<OutputType>& getConversionOutput() {
		return output;
	}

protected:
	System::Input<InputType>& input;
	System::Output<OutputType>& output;

private:
	DISALLOW_COPY_AND_ASSIGN(IOConversion);
};


template<typename InputType, typename OutputType>
IOConversion<InputType, OutputType>* makeIOConversion(
		System::Input<InputType>& input, System::Output<OutputType>& output)
{
	return new IOConversion<InputType, OutputType>(input, output);
}


}
}


#endif /* IO_CONVERSION_H_ */
