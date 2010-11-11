/*
 * hand.cpp
 *
 *  Created on: Nov 9, 2010
 *      Author: dc
 */

#include <vector>

#include <syslog.h>

#include <barrett/products/puck.h>
#include <barrett/products/motor_puck.h>
#include <barrett/products/puck_group.h>
#include <barrett/products/hand.h>
#include <barrett/products/abstract/multi_puck_product.h>


namespace barrett {


const enum Puck::Property Hand::props[] = { Puck::P, Puck::T };


Hand::Hand(const std::vector<Puck*>& _pucks) :
	MultiPuckProduct(DOF, _pucks, PuckGroup::BGRP_HAND, props, sizeof(props)/sizeof(props[0]), "Hand::Hand()")
{
}
Hand::~Hand()
{
}


}
