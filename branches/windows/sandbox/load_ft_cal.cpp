/*
 * load_ft_cal.cpp
 *
 *  Created on: Jul 9, 2010
 *      Author: dc
 */

#include <iostream>
#include <fstream>
#include <cstdlib>
#include <cstdio>

#include <unistd.h>

#include <barrett/products/product_manager.h>


using namespace std;
using namespace barrett;

const int GM_SIZE = 36;  // The gain matrix is a 6x6 matrix with 36 elements


int main(int argc, char** argv) {
	if (argc != 2) {
		printf("Usage: %s <calibrationFile>\n", argv[0]);
		printf("    <calibrationFile>    File containing the gain-matrix for the attached Force-Torque Sensor\n");
		return 1;
	}


	ProductManager pm;
	if ( !pm.foundForceTorqueSensor() ) {
		printf("ERROR: No Force-Torque Sensor found!\n");
		return 1;
	}
	ForceTorqueSensor& fts = *pm.getForceTorqueSensor();


	int calValue = 0;
	ifstream cal(argv[1]);
	for (int i = 0; i < GM_SIZE; ++i) {
		if (cal.good()) {
			cal >> calValue;
			if (calValue < -32768  ||  calValue > 32767) {
				printf("Calibration file is poorly formated: value out of range: %d.\n", calValue);
				return 1;
			}

			// TODO(dc): Fix once F/T has working ROLE
			//fts.setProperty(Puck::GM, calValue);
			Puck::setProperty(pm.getBus(), fts.getPuck()->getId(), Puck::getPropertyId(Puck::GM, Puck::PT_ForceTorque, 152), calValue);

			printf(".");
			fflush(stdout);
			usleep(1000000);
		} else {
			printf("Calibration file is poorly formated: not enough values.\n");
			return 1;
		}
	}

	printf(" Done.\n");

	return 0;
}
