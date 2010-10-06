/*
 * bus_manager.cpp
 *
 *  Created on: Aug 24, 2010
 *      Author: dc
 */

#include <cstdio>
#include <vector>

#include <barrett/bus/bus_manager.h>


using namespace barrett;


int main() {
	BusManager bm;
	bm.enumerate();

	printf("Pucks found on bus:\n");
	const std::vector<Puck*>& pucks = bm.getPucks();
	for (size_t i = 0; i < pucks.size(); ++i) {
		if (i > 0  &&  pucks[i]->getId() - 1 != pucks[i-1]->getId()) {
			printf("  --\n");
		}
		printf("  ID=%2d,VERS=%3d,ROLE=0x%04x,TYPE=%s%s\n",
				pucks[i]->getId(), pucks[i]->getVers(), pucks[i]->getRole(),
				Puck::getPuckTypeStr(pucks[i]->getType()),
				(pucks[i]->getEffectiveType() == Puck::PT_Monitor) ? " (Monitor)" : "");
	}

//	pucks[0]->wake();
	Puck::wake(pucks);

	return 0;
}
