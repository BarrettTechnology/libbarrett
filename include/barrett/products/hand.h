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
#include <barrett/products/abstract/multi_puck_product.h>


namespace barrett {


class Hand : public MultiPuckProduct {
public:
	static const size_t DOF = 4;
private:
	BARRETT_UNITS_TYPEDEFS(DOF);

public:
	Hand(const std::vector<Puck*>& pucks);
	~Hand();

private:
	static const enum Puck::Property props[];

	DISALLOW_COPY_AND_ASSIGN(Hand);
};


}


#endif /* BARRETT_PRODUCTS_BHAND_H_ */
