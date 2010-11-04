/*
 * bus_manager.cpp
 *
 *  Created on: Aug 24, 2010
 *      Author: dc
 */

#include <iostream>
#include <cstdio>
#include <vector>

#include <barrett/exception.h>
#include <barrett/bus/bus_manager.h>
#include <barrett/products/puck.h>
#include <barrett/products/puck_group.h>

#include <barrett/products/low_level_wam.h>
#include <signal.h>
#include <native/task.h>
#include <boost/thread.hpp>
#include <barrett/detail/stacktrace.h>


const size_t DOF = 4;

using namespace barrett;
BARRETT_UNITS_TYPEDEFS(DOF);


void warnOnSwitchToSecondaryMode(int)
{
	syslog(LOG_ERR, "WARNING: Switched out of RealTime. Stack-trace:");
	detail::syslog_stacktrace();
	std::cerr << "WARNING: Switched out of RealTime. Stack-trace in syslog.\n";
}


void wamControl(LowLevelWam<DOF>* wam) {
	jt_type jt(0.0);

	rt_task_shadow(new RT_TASK, NULL, 10, 0);
	rt_task_set_mode(0, T_PRIMARY | T_WARNSW, NULL);
	rt_task_set_periodic(NULL, TM_NOW, 2000000);

	while (true) {
		rt_task_wait_period(NULL);

		wam->update();
		wam->setTorques(jt);

//		std::cout << jt << "\n";
	}
}

int main() {
	installExceptionHandler();
	signal(SIGXCPU, &warnOnSwitchToSecondaryMode);

	BusManager bm;

//	printf("Pucks found on bus:\n");
//	const std::vector<Puck*>& pucks = bm.getPucks();
//	for (size_t i = 0; i < pucks.size(); ++i) {
//		if (i > 0  &&  pucks[i]->getId() - 1 != pucks[i-1]->getId()) {
//			printf("  --\n");
//		}
//		printf("  ID=%2d,VERS=%3d,ROLE=0x%04x,TYPE=%s%s\n",
//				pucks[i]->getId(), pucks[i]->getVers(), pucks[i]->getRole(),
//				Puck::getPuckTypeStr(pucks[i]->getType()),
//				(pucks[i]->getEffectiveType() == Puck::PT_Monitor) ? " (Monitor)" : "");
//		printf("  %d, %d, %d, %d\n", pucks[i]->getProperty(Puck::GRPA), pucks[i]->getProperty(Puck::GRPB), pucks[i]->getProperty(Puck::GRPC), pucks[i]->getProperty(Puck::TSTOP));
//	}

//	Puck::wake(pucks);

//	PuckGroup pg(PuckGroup::BGRP_BHAND, pucks);
//	printf("%d\n", pg.verifyProperty(Puck::TSTOP));
//	pg.setProperty(Puck::TSTOP, pucks[0]->getProperty(Puck::TSTOP) + 1);
//	usleep(1000);
//	pg.getProperty(Puck::TSTOP);
////	Puck::sendGetPropertyRequest(bm, PuckGroup::BGRP_BHAND, pucks[0]->getPropertyId(Puck::TSTOP));
//	sleep(2);


	std::vector<Puck*> wamPucks = bm.getWamPucks();
	wamPucks.resize(DOF);
	LowLevelWam<DOF> wam(wamPucks, bm.getPuck(10), bm.getConfig().lookup("wam4.low_level"));
	boost::thread t(wamControl, &wam);

	while (true) {
//		usleep(10000);

		wamPucks[0]->getProperty(Puck::TEMP);
		wamPucks[1]->getProperty(Puck::TEMP);
		wamPucks[2]->getProperty(Puck::TEMP);
		wamPucks[3]->getProperty(Puck::TEMP);

//		printf("%d %d %d %d\n", wamPucks[0]->getProperty(Puck::TEMP), wamPucks[1]->getProperty(Puck::TEMP), wamPucks[2]->getProperty(Puck::TEMP), wamPucks[3]->getProperty(Puck::TEMP));
	}

	return 0;
}
