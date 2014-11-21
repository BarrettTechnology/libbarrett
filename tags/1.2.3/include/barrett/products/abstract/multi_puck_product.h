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
 */
/**
 * @file multi_puck_product.h
 * @date 11/11/2010
 * @author Dan Cody
 * 
 */

#ifndef MULTI_PUCK_PRODUCT_H_
#define MULTI_PUCK_PRODUCT_H_


#include <vector>

#include <barrett/detail/ca_macro.h>
#include <barrett/bus/abstract/communications_bus.h>
#include <barrett/products/puck.h>
#include <barrett/products/puck_group.h>
#include <barrett/products/motor_puck.h>


namespace barrett {


class MultiPuckProduct {
public:
	/** MultiPuckProduct Constructor */
	MultiPuckProduct(size_t DOF, const std::vector<Puck*>& pucks, int groupId, const enum Puck::Property props[], const size_t numProps, const char* syslogStr = NULL);
	~MultiPuckProduct();
	/** getPucks Method generates a vector of Points to each puck */
	const std::vector<Puck*>& getPucks() const { return pucks; }
	/** getMotorPucks Method generates a vector */
	const std::vector<MotorPuck>& getMotorPucks() const { return motorPucks; }
	/** getPuckGroup Method gets point to pucks with appropriate role */
	const PuckGroup& getPuckGroup() const { return group; }

protected:
	const bus::CommunicationsBus& bus;
	std::vector<Puck*> pucks;
	std::vector<MotorPuck> motorPucks;
	PuckGroup group;

private:
	DISALLOW_COPY_AND_ASSIGN(MultiPuckProduct);
};


}


#endif /* MULTI_PUCK_PRODUCT_H_ */
