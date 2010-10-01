/*
 * gravity_compensator.h
 *
 *  Created on: Feb 9, 2010
 *      Author: dc
 */

#ifndef BARRETT_SYSTEMS_GRAVITY_COMPENSATOR_H_
#define BARRETT_SYSTEMS_GRAVITY_COMPENSATOR_H_


#include <libconfig.h++>


#include <barrett/detail/ca_macro.h>
#include <barrett/units.h>
#include <barrett/cdlbt/calgrav.h>

#include <barrett/systems/abstract/system.h>
#include <barrett/systems/abstract/single_io.h>
#include <barrett/systems/kinematics_base.h>


namespace barrett {
namespace systems {


template<size_t DOF>
class GravityCompensator : public System,
						   public KinematicsInput<DOF>,
						   public SingleOutput<typename units::JointTorques<DOF>::type> {

	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

public:
	explicit GravityCompensator(const libconfig::Setting& setting) :
		KinematicsInput<DOF>(this), SingleOutput<jt_type>(this),
		impl(NULL) {
		bt_calgrav_create(&impl, setting.getCSetting(), DOF);
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

	struct bt_calgrav* impl;

private:
	DISALLOW_COPY_AND_ASSIGN(GravityCompensator);
};


}
}


#endif /* BARRETT_SYSTEMS_GRAVITY_COMPENSATOR_H_ */
