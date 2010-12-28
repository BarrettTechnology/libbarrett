/*
 * ft_persistent_tare.cpp
 *
 *  Created on: Dec 28, 2010
 *      Author: dc
 */

#include <iostream>
#include <fstream>
#include <cstdlib>
#include <cstdio>

#include <unistd.h>

#include <barrett/bus/bus_manager.h>
#include <barrett/products/force_torque_sensor.h>


using namespace std;
using namespace barrett;

const size_t NUM_SG = 6;


int main(int argc, char** argv) {
	BusManager bm;
	if ( !bm.foundForceTorqueSensor() ) {
		printf("ERROR: No Force-Torque Sensor found!\n");
		return 1;
	}
	ForceTorqueSensor& fts = *bm.getForceTorqueSensor();


	printf("Reading strain gauges: ");
	math::Vector<NUM_SG>::type ov;  // offset vector
	for (size_t i = 0; i < NUM_SG; ++i) {
		// TODO(dc): Fix once F/T has working ROLE
		//ov[i] = fts.getProperty(static_cast<Puck::Property>(Puck::SG1 + i));
		ov[i] = Puck::getProperty(bm, fts.getPuck()->getId(), Puck::getPropertyId(static_cast<Puck::Property>(Puck::SG1 + i), Puck::PT_ForceTorque, 152));
	}
	cout << ov << "\n";

	printf("Writing offset vector");
	for (size_t i = 0; i < NUM_SG; ++i) {
		// TODO(dc): Fix once F/T has working ROLE
		//fts.setProperty(Puck::OV, calValue);
		Puck::setProperty(bm, fts.getPuck()->getId(), Puck::getPropertyId(Puck::OV, Puck::PT_ForceTorque, 152), ov[i]);
		printf(".");
		fflush(stdout);
		usleep(1000000);
	}

	printf(" Done.\n");

	return 0;
}
