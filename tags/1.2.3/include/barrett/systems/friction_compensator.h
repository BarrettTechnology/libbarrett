/*
	Copyright 2012 Barrett Technology <support@barrett.com>

	This file is part of libbarrett.

	This version of libbarrett is free software: you can redistribute it
	and/or modify it under the terms of the GNU General Public License as
	published by the Free Software Foundation, either version 3 of the
	License, or (at your option) any later version.

	This version of libbarrett is distributed in the hope that it will be
	useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License along
	with this version of libbarrett.  If not, see
	<http://www.gnu.org/licenses/>.

	Further, non-binding information about licensing is available at:
	<http://wiki.barrett.com/libbarrett/wiki/LicenseNotes>
*/

/*
 * friction_compensator.h
 *
 *  Created on: Jan 27, 2012
 *      Author: dc
 */

#ifndef BARRETT_SYSTEMS_FRICTION_COMPENSATOR_H_
#define BARRETT_SYSTEMS_FRICTION_COMPENSATOR_H_


#include <string>

#include <Eigen/Core>
#include <libconfig.h++>

#include <barrett/units.h>
#include <barrett/systems/abstract/single_io.h>


namespace barrett {
namespace systems {


template<size_t DOF>
class FrictionCompensator : public SingleIO<
									typename units::JointVelocities<DOF>::type,
									typename units::JointTorques<DOF>::type>
{
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

public:
	FrictionCompensator(const libconfig::Setting& setting,
			const std::string& sysName = "FrictionCompensator") :
		SingleIO<jv_type,jt_type>(sysName),
		coulomb(setting["coulomb"]), viscous(setting["viscous"])
	{}
	virtual ~FrictionCompensator() {
		this->mandatoryCleanUp();
	}

protected:
	v_type coulomb;
	v_type viscous;
	jt_type jt;

	virtual void operate() {
		const jv_type& jv = this->input.getValue();
		for (size_t i = 0; i < DOF; ++i) {
			if (jv[i] > 0.0) {
				jt[i] = jv[i]*viscous[i] + coulomb[i];
			} else if (jv[i] < 0.0) {
				jt[i] = jv[i]*viscous[i] - coulomb[i];
			} else {
				jt[i] = 0.0;
			}
		}

		this->outputValue->setData(&jt);
	}

private:
	DISALLOW_COPY_AND_ASSIGN(FrictionCompensator);

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


}
}


#endif /* BARRETT_SYSTEMS_FRICTION_COMPENSATOR_H_ */
