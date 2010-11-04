/*
 * safety_module.h
 *
 *  Created on: Nov 4, 2010
 *      Author: dc
 */

#ifndef BARRETT_PRODUCTS_SAFETY_MODULE_H_
#define BARRETT_PRODUCTS_SAFETY_MODULE_H_


#include <stdexcept>

#include <syslog.h>

#include <barrett/products/puck.h>
#include <barrett/products/abstract/special_puck.h>


namespace barrett {


class SafetyModule : public SpecialPuck {
public:
	enum SafetyMode {
		ESTOP, IDLE, ACTIVE
	};


	SafetyModule(Puck* puck = NULL) : SpecialPuck(Puck::PT_Safety) { setPuck(puck); }
	virtual ~SafetyModule() {}

	virtual void update() {}

	enum SafetyMode getMode() const {
		int s = getProperty(Puck::MODE);
		if (s < 0  ||  s > 2) {
			syslog(LOG_ERR, "SafetyModule::getMode(): Expected MODE value of 0, 1, or 2. Got value of %d.", s);
			throw std::runtime_error("SafetyModule::getMode(): Bad MODE value. Check /var/log/syslog for details.");
		}
		return static_cast<enum SafetyMode>(s);
	}

	bool wamIsZeroed() const { return getProperty(Puck::ZERO) == 1; }
	void setWamZeroed(bool zeroed = true) const { setProperty(Puck::ZERO, zeroed); }
};


}


#endif /* BARRETT_PRODUCTS_SAFETY_MODULE_H_ */
