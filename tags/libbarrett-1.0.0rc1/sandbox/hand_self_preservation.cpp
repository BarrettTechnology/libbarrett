/*
 * hand_self_preservation.cpp
 *
 *  Created on: Jun 17, 2011
 *      Author: dc
 */

#include <iostream>
#include <algorithm>

#include <unistd.h>
#include <native/task.h>
#include <native/timer.h>

#include <barrett/products/product_manager.h>


using namespace barrett;


int main() {
	ProductManager pm;
	if ( !pm.foundHand() ) {
		printf("ERROR: No Hand found on bus!\n");
		return 1;
	}
	Hand& hand = *pm.getHand();


	hand.initialize();

	Hand::jp_type cur(0.0), next(0.0);
	next[0] = next[1] = next[2] = 2.6;

//	rt_task_shadow(new RT_TASK, NULL, 10, 0);
	rt_task_set_mode(0, T_PRIMARY | T_WARNSW, NULL);
	rt_task_set_periodic(NULL, TM_NOW, 2000);

	RTIME now = rt_timer_read();
	RTIME lastMove = 0;

	while (true) {
		rt_task_wait_period(NULL);
		now = rt_timer_read();

		hand.update(Hand::S_POSITION | Hand::S_FINGERTIP_TORQUE, true);
		const std::vector<int>& ftt = hand.getFingertipTorque();
		for (size_t i = 0; i < Hand::DOF - 1; ++i) {
			int offset = 0;
			if (ftt[i] > 2800) {
				offset = -30000;
			} else if (ftt[i] < 1000) {
				offset = 30000;
			}

			if (offset != 0  &&  hand.getPucks()[i]->getProperty(Puck::MODE) != MotorPuck::MODE_PID) {
				lastMove = now;
				hand.getPucks()[i]->setProperty(Puck::MODE, MotorPuck::MODE_PID);
				hand.getPucks()[i]->setProperty(Puck::P, hand.getPrimaryEncoderPosition()[i] + offset);
			}
		}

		if (now - lastMove > 4000000000UL) {
			hand.trapezoidalMove(next, false);
			std::swap(cur, next);
			lastMove = now;
		}
	}

	hand.idle();

	return 0;
}
