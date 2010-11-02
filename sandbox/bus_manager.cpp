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
#include <barrett/puck.h>
#include <barrett/puck_group.h>

#include "llw.h"


const size_t DOF = 4;

using namespace barrett;
BARRETT_UNITS_TYPEDEFS(DOF);


int main() {
	installExceptionHandler();

	BusManager bm;
	bm.enumerate();

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


	std::vector<Puck*> wamPucks;
	wamPucks.push_back(bm.getPuck(1));
	wamPucks.push_back(bm.getPuck(2));
	wamPucks.push_back(bm.getPuck(3));
	wamPucks.push_back(bm.getPuck(4));

	LLW<DOF> wam(wamPucks, bm.getPuck(10), bm.getConfig().lookup("wam.low_level"));

	jt_type jt;

	jt << 1, 0, 0, 0;
	wam.update();
	wam.setTorques(jt);
	sleep(1);
	jt << 0, 0, 0, 0;
	wam.update();
	wam.setTorques(jt);

	return 0;
}
