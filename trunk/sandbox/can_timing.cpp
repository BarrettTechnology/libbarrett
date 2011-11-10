/*
 * can_timing.cpp
 *
 *  Created on: May 4, 2011
 *      Author: dc
 */

#include <iostream>
#include <limits>

#include <native/task.h>
#include <native/timer.h>

#include <barrett/products/product_manager.h>


using namespace barrett;

const int NUM_MESSAGES = 1000;


#define PRE_MEASURE  \
min = std::numeric_limits<unsigned int>::max();  \
max = 0;  \
sum = 0;  \
sumSq = 0;  \
rt_task_set_mode(0, T_PRIMARY | T_WARNSW, NULL);  \
for (int messageNum = 0; messageNum < NUM_MESSAGES; ++messageNum) {  \
	begin = rt_timer_read()


#define POST_MEASURE  \
	end = rt_timer_read();  \
	  \
	duration = (end - begin) / 1000;  \
	  \
	if (duration < min) {  \
		min = duration;  \
	}  \
	if (duration > max) {  \
		max = duration;  \
	}  \
	sum += duration;  \
	sumSq += duration * duration;  \
}  \
rt_task_set_mode(T_PRIMARY | T_WARNSW, 0, NULL);  \
  \
mean = (double)sum / NUM_MESSAGES;  \
stdev = std::sqrt( ((double)sumSq/NUM_MESSAGES) - mean*mean );  \
printf("%d,%d,%u,%.3f,%u,%.3f\n", puckId, propId, min, mean, max, stdev)


#define MEASURE(puck, prop)  \
puckId = puck->getId();  \
propId = puck->getPropertyId(prop);  \
PRE_MEASURE;  \
puck->getProperty(prop, true);  \
POST_MEASURE;


int main() {
	ProductManager pm;
	if ( !pm.foundHand() ) {
		printf("ERROR: No Hand found on bus!\n");
		return 1;
	}

	Hand& hand = *pm.getHand();
	hand.initialize();


	int puckId, propId;
	RTIME begin, end;
    uint32_t duration;
	uint32_t min = std::numeric_limits<unsigned int>::max();
	uint32_t max = 0;
	uint64_t sum = 0;
	uint64_t sumSq = 0;
	double mean, stdev;

	rt_task_shadow(new RT_TASK, NULL, 95, 0);

	Puck* puck = NULL;
	for (size_t i = 0; i < hand.getPucks().size(); ++i) {
		puck = hand.getPucks()[i];

		MEASURE(puck, Puck::STAT);
		MEASURE(puck, Puck::MODE);

		int p;
		puckId = puck->getId();
		propId = puck->getPropertyId(Puck::P);
		PRE_MEASURE;
		puck->getProperty<MotorPuck::MotorPositionParser<int> >(Puck::P, &p, true);
		POST_MEASURE;

		MEASURE(puck, Puck::SG);

//		TactilePuck* tp = hand.getTactilePucks()[i];
//		puckId = puck->getId();
//		propId = puck->getPropertyId(Puck::TACT);
//		PRE_MEASURE;
//		tp->updateFull(true);
//		POST_MEASURE;
	}


	hand.idle();

	return 0;
}
