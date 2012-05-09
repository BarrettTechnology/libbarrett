/*
	Copyright 2010, 2011, 2012 Barrett Technology <support@barrett.com>

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
 * special_puck.h
 *
 *  Created on: Nov 4, 2010
 *      Author: dc
 */

#ifndef BARRETT_PRODUCTS_ABSTRACT_SPECIAL_PUCK_H_
#define BARRETT_PRODUCTS_ABSTRACT_SPECIAL_PUCK_H_


#include <stdexcept>

#include <barrett/os.h>
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
			(logMessage("SpecialPuck::%s(): Bad PuckType. "
					"Expected Puck with type %s, got Puck with type %s.")
					% __func__ % Puck::getPuckTypeStr(type) % Puck::getPuckTypeStr(puck->getType())
			).raise<std::logic_error>();
		}
		p = puck;
	}

protected:
	enum Puck::PuckType type;
	Puck* p;
};


}


#endif /* BARRETT_PRODUCTS_ABSTRACT_SPECIAL_PUCK_H_ */
