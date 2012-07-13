/*
 * tensioner.h
 *
 *  Created on: Jun 14, 2011
 *      Author: CJ Valle
 */

#ifndef TENSIONER_H_
#define TENSIONER_H_

#include <stdexcept>
#include <cmath>

#include <syslog.h>
#include <unistd.h> /* for close() */

#include <barrett/units.h>
#include <barrett/systems/abstract/single_io.h>

namespace barrett {
namespace systems {

template <size_t DOF>
class Tensioner : public barrett::systems::SingleIO<typename barrett::units::JointTorques<DOF>::type, typename barrett::units::JointTorques<DOF>::type> {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

public:
	explicit Tensioner(ExecutionManager* em, const std::string& sysName = "Tensioner") :
		barrett::systems::SingleIO<jt_type,jt_type>(sysName)
	{
		watching = false;
		if (em != NULL) em->startManaging(*this);
	}

	void activate(int m, double l)
	{
		motor = m;
		limit = l;
		watching = true;

	}

	void deactivate()
	{
		watching = false;
	}

protected:

	virtual void operate()
	{
		if (watching) 
		{
			torques = this->input.getValue();
			if (fabs(torques[motor]) > limit)
			{
				watching = false;
			}
		}
	}

public:
	bool watching;
	int motor;
	double limit;
	jt_type torques;
};
}}
#endif /* TENSIONER_H_ */
