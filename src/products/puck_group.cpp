/*
 * puck_group.cpp
 *
 *  Created on: Oct 7, 2010
 *      Author: dc
 */

#include <stdexcept>
#include <vector>

#include <barrett/products/puck.h>
#include <barrett/products/puck_group.h>


namespace barrett {


PuckGroup::PuckGroup(int _id, const std::vector<Puck*>& _pucks) :
	id(_id), pucks(_pucks), bus(pucks[0]->getBus())
{
	if ( !(id & Puck::GROUP_MASK)  ||  (id & Puck::TO_MASK) != id) {
		throw std::invalid_argument("PuckGroup::PuckGroup(): Invalid Group ID.");
	}
}

PuckGroup::~PuckGroup()
{
}

bool PuckGroup::verifyProperty(enum Puck::Property prop) const
{
	int propId = getPropertyIdNoThrow(prop);  // Checks the first Puck

	if (propId == -1) {
		return false;
	}
	for (size_t i = 1; i < numPucks(); ++i) {  // Checks the rest of the Pucks
		if (pucks[i]->getPropertyIdNoThrow(prop) != propId) {
			return false;
		}
	}

	return true;
}


}
