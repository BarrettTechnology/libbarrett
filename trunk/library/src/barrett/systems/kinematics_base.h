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


class KinematicsInput {  // not a System in order to avoid diamond inheritance
// IO
public:	System::Input<const math::Kinematics*> kinInput;


public:
	explicit KinematicsInput(System* parentSys) :
		kinInput(parentSys) {}

private:
	DISALLOW_COPY_AND_ASSIGN(KinematicsInput);
};


class KinematicsBase : public System {
// IO
public:		Input<units::jp_type> jpInput;
public:		Input<units::jv_type> jvInput;
public:		Output<const math::Kinematics*> kinOutput;
protected:	Output<const math::Kinematics*>::Value* kinOutputValue;


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

	math::Kinematics kin;

private:
	DISALLOW_COPY_AND_ASSIGN(KinematicsBase);
};


}
}


#endif /* KINEMATICS_BASE_H_ */
