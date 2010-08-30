/*
 * bus_manager.cpp
 *
 *  Created on: Aug 24, 2010
 *      Author: dc
 */

#include <cstdio>

#include <barrett/bus/bus_manager.h>


using namespace barrett;


int main() {
	BusManager bm;
	bm.enumerate();

	return 0;
}
