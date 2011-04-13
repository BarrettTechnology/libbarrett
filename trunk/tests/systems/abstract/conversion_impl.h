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
	ConversionImpl(const std::string& sysName = "ConversionImpl") :
		barrett::systems::SingleIO<InputType, OutputType>(sysName) {}
	virtual ~ConversionImpl() { this->mandatoryCleanUp(); }

protected:
	virtual void operate() {
		data = static_cast<typename OutputType::Base>(this->input.getValue());
		this->outputValue->setData(&data);
	}

	OutputType data;

private:
	DISALLOW_COPY_AND_ASSIGN(ConversionImpl);
};


#endif /* CONVERSION_IMPL_H_ */
