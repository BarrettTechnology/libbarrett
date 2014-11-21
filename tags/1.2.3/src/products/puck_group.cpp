/**
 *	Copyright 2009-2014 Barrett Technology <support@barrett.com>
 *
 *	This file is part of libbarrett.
 *
 *	This version of libbarrett is free software: you can redistribute it
 *	and/or modify it under the terms of the GNU General Public License as
 *	published by the Free Software Foundation, either version 3 of the
 *	License, or (at your option) any later version.
 *
 *	This version of libbarrett is distributed in the hope that it will be
 *	useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License along
 *	with this version of libbarrett.  If not, see
 *	<http://www.gnu.org/licenses/>.
 *
 *
 *	Barrett Technology Inc.
 *	73 Chapel Street
 *	Newton, MA 02458
 *
 */
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
