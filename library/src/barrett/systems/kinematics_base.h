/*
 * kinematics_base.h
 *
 *  Created on: Jan 15, 2010
 *      Author: dc
 */

#ifndef KINEMATICS_BASE_H_
#define KINEMATICS_BASE_H_


#include <libconfig.h>

#include "../detail/ca_macro.h"
#include "../units.h"
#include "../math/kinematics.h"
#include "./abstract/system.h"


namespace barrett {
namespace systems {


template<size_t DOF>
class KinematicsBase : public System {
// IO
public:		Input<units::JointPositions<DOF> > jpInput;
public:		Input<units::JointVelocities<DOF> > jvInput;
public:		Output<const math::Kinematics<DOF>*> output;
protected:	typename Output<const math::Kinematics<DOF>*>::Value* outputValue;


public:
	explicit KinematicsBase(config_setting_t * config) :
		jpInput(this), jvInput(this), output(this, &outputValue), kin(config) {}
	virtual ~KinematicsBase() {}

protected:
	virtual void operate() {
		kin.eval(jpInput.getValue(), jvInput.getValue());
		outputValue->setValue(&kin);
	}

	math::Kinematics<DOF> kin;

private:
	DISALLOW_COPY_AND_ASSIGN(KinematicsBase);
};


}
}


#endif /* KINEMATICS_BASE_H_ */
