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
		printf("  ID=%2d,VERS=%3d,TYPE=%d\n", pucks[i]->getId(), pucks[i]->getVers(), pucks[i]->getEffectiveType());
	}

//	pucks[0]->wake();
	Puck::wake(pucks);

	return 0;
}
