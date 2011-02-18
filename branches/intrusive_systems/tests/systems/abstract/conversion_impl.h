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


// SingleIO<> implements Conversion
template<typename InputType, typename OutputType>
class ConversionImpl :
		public barrett::systems::SingleIO<InputType, OutputType> {
public:
	ConversionImpl() {}
	virtual ~ConversionImpl() {}

protected:
	virtual void operate() {
		this->outputValue->setValue(this->input.getValue());
	}

private:
	DISALLOW_COPY_AND_ASSIGN(ConversionImpl);
};


#endif /* CONVERSION_IMPL_H_ */
