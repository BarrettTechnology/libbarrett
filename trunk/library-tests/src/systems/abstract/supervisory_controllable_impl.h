/*
 * supervisory_controllable_impl.h
 *
 *  Created on: Oct 29, 2009
 *      Author: dc
 */

#ifndef SUPERVISORY_CONTROLLABLE_IMPL_H_
#define SUPERVISORY_CONTROLLABLE_IMPL_H_


#include <barrett/detail/ca_macro.h>
#include <barrett/systems/abstract/system.h>
#include <barrett/systems/abstract/single_io.h>
#include <barrett/systems/abstract/supervisory_controllable.h>


template<typename InputType, typename OutputType>
class SupervisoryControllableImpl :
		public barrett::systems::SingleIO<InputType, OutputType>,
		public barrett::systems::SupervisoryControllable<OutputType> {
public:
	SupervisoryControllableImpl() {}
	virtual ~SupervisoryControllableImpl() {}

	virtual barrett::systems::System::AbstractInput*
	getSupervisoryControllableInput()
	{
		return &(this->input);
	}

	virtual barrett::systems::System::Output<OutputType>&
	getSupervisoryControllableOutput()
	{
		return this->output;
	}

protected:
	virtual void operate() {
		this->outputValue->setValue(this->input.getValue());
	}

private:
	DISALLOW_COPY_AND_ASSIGN(SupervisoryControllableImpl);
};


#endif /* SUPERVISORY_CONTROLLABLE_IMPL_H_ */
