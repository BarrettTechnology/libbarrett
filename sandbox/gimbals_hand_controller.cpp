/*
 * gimbals_hand_controller.cpp
 *
 *  Created on: Oct 15, 2010
 *      Author: dc
 */

#include <cstdio>
#include <iostream>
#include <string>

#include <unistd.h>

#include <barrett/bus/bus_manager.h>
#include <barrett/puck.h>


using namespace barrett;


void waitForEnter() {
	static std::string line;
	std::getline(std::cin, line);
}

int main() {
	BusManager bm;
	bm.enumerate();

	Puck& p6 = *bm.getPuck(6);
	Puck& p7 = *bm.getPuck(7);

	p6.wake();
	p7.wake();

	// wire colors as labeled on Gimbals documentation
	int brown, dGreen, orange, yellow, lGreen, blue, violet, white;
	int tmp;
	while (true) {
//		waitForEnter();
		usleep(250000);

		// fetch data
		brown = p7.getProperty(Puck::ANA0);
		dGreen = p7.getProperty(Puck::ANA1);

		orange = p6.getProperty(Puck::ANA0);
		yellow = p6.getProperty(Puck::ANA1);

		tmp = p6.getProperty(Puck::HALLS);
		lGreen = tmp & 0x1;
		blue = tmp & 0x2;

		tmp = p7.getProperty(Puck::HALLS);
		violet = tmp & 0x1;
		white = tmp & 0x2;


		printf("%d, %d, %d, %d, %d, %d, %d, %d\n", brown, dGreen, orange, yellow, lGreen, blue, violet, white);
	}
}
