/*
 * puck_group.cpp
 *
 *  Created on: Oct 7, 2010
 *      Author: dc
 */

#include <stdexcept>
#include <vector>

#include <barrett/puck.h>
#include <barrett/puck_group.h>


namespace barrett {


PuckGroup::PuckGroup(int _id, const std::vector<Puck*>& _pucks) :
	id(_id), pucks(_pucks)
{
	if ( !(id & Puck::GROUP_MASK)  ||  (id & Puck::TO_MASK) != id) {
		throw std::invalid_argument("PuckGroup::PuckGroup(): Invalid Group ID.");
	}
}

PuckGroup::~PuckGroup()
{
}

bool PuckGroup::verifyProperty(enum Puck::Property prop)
{
	int propId = pucks[0]->getPropertyIdNoThrow(prop);

	if (propId == -1) {
		return false;
	}
	for (size_t i = 1; i < pucks.size(); ++i) {
		if (pucks[i]->getPropertyIdNoThrow(prop) != propId) {
			return false;
		}
	}

	return true;
}


}
