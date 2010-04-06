/*
 * kinematics_base.h
 *
 *  Created on: Jan 15, 2010
 *      Author: dc
 */

#ifndef KINEMATICS_BASE_H_
#define KINEMATICS_BASE_H_


#include <libconfig.h++>

#include "../detail/ca_macro.h"
#include "../units.h"
#include "../math/kinematics.h"
#include "./abstract/system.h"


namespace barrett {
namespace systems {


template<size_t DOF>
class KinematicsInput {  // not a System in order to avoid diamond inheritance
// IO
public:	System::Input<const math::Kinematics<DOF>*> kinInput;


public:
	explicit KinematicsInput(System* parentSys) :
		kinInput(parentSys) {}

private:
	DISALLOW_COPY_AND_ASSIGN(KinematicsInput);
};


template<size_t DOF>
class KinematicsBase : public System {
// IO
public:		Input<typename units::JointPositions<DOF>::type> jpInput;
public:		Input<typename units::JointVelocities<DOF>::type> jvInput;
public:		Output<const math::Kinematics<DOF>*> kinOutput;
protected:	typename Output<const math::Kinematics<DOF>*>::Value* kinOutputValue;


public:
	explicit KinematicsBase(const libconfig::Setting& setting) :
		jpInput(this), jvInput(this),
		kinOutput(this, &kinOutputValue), kin(setting) {}
	virtual ~KinematicsBase() {}

protected:
	virtual void operate() {
		kin.eval(jpInput.getValue(), jvInput.getValue());
		kinOutputValue->setValue(&kin);
	}

	math::Kinematics<DOF> kin;

private:
	DISALLOW_COPY_AND_ASSIGN(KinematicsBase);
};


}
}


#endif /* KINEMATICS_BASE_H_ */
