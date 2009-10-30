/*
 * conversion_impl.h
 *
 *  Created on: Oct 29, 2009
 *      Author: dc
 */

#ifndef CONVERSION_IMPL_H_
#define CONVERSION_IMPL_H_


#include <barrett/detail/ca_macro.h>
#include <barrett/systems/abstract/system.h>
#include <barrett/systems/abstract/single_io.h>
#include <barrett/systems/abstract/conversion.h>


template<typename InputType, typename OutputType>
class ConversionImpl :
		public barrett::systems::SingleIO<InputType, OutputType>,
		public barrett::systems::Conversion<OutputType> {
public:
	ConversionImpl() {}
	virtual ~ConversionImpl() {}

	virtual barrett::systems::System::AbstractInput*
	getConversionInput()
	{
		return &(this->input);
	}

	virtual barrett::systems::System::Output<OutputType>&
	getConversionOutput()
	{
		return this->output;
	}

protected:
	virtual void operate() {
		this->outputValue->setValue(this->input.getValue());
	}

private:
	DISALLOW_COPY_AND_ASSIGN(ConversionImpl);
};


#endif /* CONVERSION_IMPL_H_ */
