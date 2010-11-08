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
		type(_type) {}
	virtual ~SpecialPuck() {}

	virtual void setPuck(Puck* puck, bool autoUpdate = true) {
		if (puck != NULL  &&  type != Puck::PT_Unknown  &&  puck->getType() != type) {
			syslog(LOG_ERR, "SpecialPuck::setPuck(): Expected Puck with type %s, got Puck with type %s.",
					Puck::getPuckTypeStr(type), Puck::getPuckTypeStr(puck->getType()));
			throw std::logic_error("SpecialPuck::setPuck(): Bad PuckType. Check /var/log/syslog for details.");
		}
		p = puck;
		if (p != NULL  &&  autoUpdate) {
			update();
		}
	}
	Puck* getPuck() const { return p; }
	virtual void update() {}

	int getProperty(enum Puck::Property prop) const {
		return p->getProperty(prop);
	}
	void setProperty(enum Puck::Property prop, int value) const {
		p->setProperty(prop, value);
	}

	int getId() const { return p->getId(); }
	int getVers() const { return p->getRole(); }
	int getRole() const { return p->getRole(); }

protected:
	enum Puck::PuckType type;
	Puck* p;
};


}


#endif /* BARRETT_PRODUCTS_ABSTRACT_SPECIAL_PUCK_H_ */
