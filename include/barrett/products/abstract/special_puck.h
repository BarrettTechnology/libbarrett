/*
 * special_puck.h
 *
 *  Created on: Nov 4, 2010
 *      Author: dc
 */

#ifndef BARRETT_PRODUCTS_ABSTRACT_SPECIAL_PUCK_H_
#define BARRETT_PRODUCTS_ABSTRACT_SPECIAL_PUCK_H_


#include <stdexcept>
#include <syslog.h>

#include <barrett/products/puck.h>


namespace barrett {


class SpecialPuck {
public:
	SpecialPuck(enum Puck::PuckType _type = Puck::PT_Unknown) :
		type(_type), p(NULL) {}
	~SpecialPuck() {}

	Puck* getPuck() const { return p; }
	void setPuck(Puck* puck) {
		if (puck != NULL  &&  type != Puck::PT_Unknown  &&  puck->getType() != type) {
			syslog(LOG_ERR, "SpecialPuck::setPuck(): Expected Puck with type %s, got Puck with type %s.",
					Puck::getPuckTypeStr(type), Puck::getPuckTypeStr(puck->getType()));
			throw std::logic_error("SpecialPuck::setPuck(): Bad PuckType. Check /var/log/syslog for details.");
		}
		p = puck;
	}

protected:
	enum Puck::PuckType type;
	Puck* p;
};


}


#endif /* BARRETT_PRODUCTS_ABSTRACT_SPECIAL_PUCK_H_ */
