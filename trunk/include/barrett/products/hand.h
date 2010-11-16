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
#include <barrett/products/abstract/multi_puck_product.h>
#include <barrett/products/puck.h>
#include <barrett/products/motor_puck.h>
#include <barrett/products/tactile_puck.h>


namespace barrett {


class Hand : public MultiPuckProduct {
public:
	static const size_t DOF = 4;
	BARRETT_UNITS_TYPEDEFS(DOF);

public:
	Hand(const std::vector<Puck*>& pucks);
	~Hand();

	void initialize() const { group.setProperty(Puck::CMD, CMD_HI); }
	void idle() const { group.setProperty(Puck::MODE, MotorPuck::MODE_IDLE); }


	// preferred: low control-rate moves
	void trapezoidalMove(const jp_type& jp) const;
	void setVelocity(const jv_type& jv) const;

	// advanced: high control-rate moves
	void setPositionMode() const { group.setProperty(Puck::MODE, MotorPuck::MODE_PID); }
	void setPositionCommand(const jp_type& jp) const;
	void setTorqueMode() const { group.setProperty(Puck::MODE, MotorPuck::MODE_TORQUE); }
	void setTorqueCommand(const jt_type& jt) const;


	void updateTactFull(bool realtime = false) const;


	const std::vector<TactilePuck*>& getTactilePucks() const { return tactilePucks; }

protected:
	std::vector<TactilePuck*> tactilePucks;

private:
	static const int CMD_HI = 13;
	static const enum Puck::Property props[];

	DISALLOW_COPY_AND_ASSIGN(Hand);
};


}


#endif /* BARRETT_PRODUCTS_BHAND_H_ */
