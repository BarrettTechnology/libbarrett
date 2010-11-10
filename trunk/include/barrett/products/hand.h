/*
 * bhand.h
 *
 *  Created on: Nov 9, 2010
 *      Author: dc
 */

#ifndef BARRETT_PRODUCTS_BHAND_H_
#define BARRETT_PRODUCTS_BHAND_H_


#include <vector>

#include <barrett/detail/ca_macro.h>
#include <barrett/units.h>
#include <barrett/products/puck.h>
#include <barrett/products/motor_puck.h>
#include <barrett/products/puck_group.h>


namespace barrett {


class Hand {
public:
	static const size_t DOF = 4;
private:
	BARRETT_UNITS_TYPEDEFS(DOF);

public:
	Hand(const std::vector<Puck*>& genericPucks);
	~Hand();

	const std::vector<MotorPuck>& getPucks() const { return pucks; }
	const PuckGroup& getGroup() const { return handGroup; }

protected:
	std::vector<MotorPuck> pucks;
	PuckGroup handGroup;

private:
	DISALLOW_COPY_AND_ASSIGN(Hand);
};


}


#endif /* BARRETT_PRODUCTS_BHAND_H_ */
