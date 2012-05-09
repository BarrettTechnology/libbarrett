/*
 * detect_vibration.cpp
 *
 *  Created on: Jul 25, 2011
 *      Author: dc
 */

#include <iostream>

#include <unistd.h>
#include <native/task.h>
#include <native/timer.h>

#include <boost/tuple/tuple.hpp>

#include <barrett/log.h>
#include <barrett/products/product_manager.h>


using namespace barrett;


int main() {
	typedef TactilePuck::v_type v_type;
	typedef boost::tuple<RTIME, v_type, v_type, v_type>  tuple_type;


	ProductManager pm;
	if ( !pm.foundHand() ) {
		printf("ERROR: No Hand found on bus!\n");
		return 1;
	}
	Hand& hand = *pm.getHand();

	log::RealTimeWriter<tuple_type> writer("tmp.bin", 0.01);

	rt_task_set_mode(0, T_PRIMARY | T_WARNSW, NULL);
	rt_task_set_periodic(NULL, TM_NOW, 10000000);
	RTIME last = rt_timer_read();

	tuple_type t;
	for (int i = 0; i < 5000; ++i) {
		rt_task_wait_period(NULL);

		hand.update(Hand::S_TACT_FULL, true);
		t.get<0>() = rt_timer_read() - last;
		t.get<1>() = hand.getTactilePucks()[0]->getFullData();
		t.get<2>() = hand.getTactilePucks()[1]->getFullData();
		t.get<3>() = hand.getTactilePucks()[2]->getFullData();
		writer.putRecord(t);
	}

	writer.close();
	log::Reader<tuple_type> reader("tmp.bin");
	reader.exportCSV("pressure.csv");

	return 0;
}
