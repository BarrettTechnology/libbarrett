/*
 * multi_puck_product.h
 *
 *  Created on: Nov 11, 2010
 *      Author: dc
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
	MultiPuckProduct(size_t DOF, const std::vector<Puck*>& pucks, int groupId, const enum Puck::Property props[], const size_t numProps, const char* syslogStr = NULL);
	~MultiPuckProduct();

	const std::vector<Puck*>& getPucks() const { return pucks; }
	const std::vector<MotorPuck>& getMotorPucks() const { return motorPucks; }
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
