/*
 * gravity_compensator.h
 *
 *  Created on: Feb 9, 2010
 *      Author: dc
 */

#ifndef GRAVITY_COMPENSATOR_H_
#define GRAVITY_COMPENSATOR_H_


#include <libconfig.h++>


#include "../detail/ca_macro.h"
#include "../units.h"
#include "./abstract/system.h"
#include "./abstract/single_io.h"
#include "./kinematics_base.h"
#include "../cdlbt/calgrav/calgrav.h"


namespace barrett {
namespace systems {


class GravityCompensator : public System,
						   public KinematicsInput,
						   public SingleOutput<units::jt_type> {
	BARRETT_UNITS_TYPEDEFS;

public:
	explicit GravityCompensator(const libconfig::Setting& setting) :
		KinematicsInput(this), SingleOutput<jt_type>(this),
		dof(setting.getLength()), impl(NULL) {
		bt_calgrav_create(&impl, setting.getCSetting(), dof);
	}

	virtual ~GravityCompensator() {
		bt_calgrav_destroy(impl);
		impl = NULL;
	}

protected:
	virtual void operate() {
		jt_type jt;
		bt_calgrav_eval(impl, this->kinInput.getValue()->impl, jt.asGslType());
		this->outputValue->setValue(jt);
	}

	const int dof;
	struct bt_calgrav* impl;

private:
	DISALLOW_COPY_AND_ASSIGN(GravityCompensator);
};


}
}


#endif /* GRAVITY_COMPENSATOR_H_ */
