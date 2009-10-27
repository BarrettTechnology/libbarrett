/*
 * main.cpp
 *
 *  Created on: Sep 29, 2009
 *      Author: dc
 */

#include <iostream>
#include <string>

#include <unistd.h>  // usleep

#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/wam.h>

using namespace barrett;


void waitForEnter() {
	static std::string line;
	std::getline(std::cin, line);
}

int main() {
	Wam wam;
	systems::SupervisoryController sc;

	double setPointArray[] =
		{0.000, -1.57, 0.0, 1.57, 0.0, 1.605, 0.0};
	units::JointAngles setPoint = setPointArray;
	systems::Constant<units::JointAngles> point(setPoint);

	// TODO(dc): this should be done by the library
	systems::System::Input<units::JointAngles>* feedbackInput =
			dynamic_cast<systems::System::Input<units::JointAngles>*>(  //NOLINT: see RTTI note above
			sc.selectController(wam.output).getFeedbackInput() );
	systems::connect(wam.output, *feedbackInput);

	systems::connect(sc.output, wam.input);
	sc.trackReferenceSignal(wam.output);

//	systems::PrintToStream<units::JointAngles> pts("JA: ");
//	systems::connect(wam.output, pts.input);

	std::cout << wam.operateCount << std::endl;

	std::cout << "Enter to gravity compensate.\n";
	waitForEnter();
	wam.gravityCompensate();

	std::cout << wam.operateCount << std::endl;
	usleep(1000000);
	std::cout << wam.operateCount << std::endl;

	std::cout << "Enter to move to set point.\n";
	waitForEnter();
	sc.trackReferenceSignal(point.output);

	std::cout << "Enter to move home.\n";
	waitForEnter();
	sc.trackReferenceSignal(wam.output);
	wam.moveHome();

	std::cout << "Enter to idle.\n";
	waitForEnter();
	wam.gravityCompensate(false);
	wam.idle();

	std::cout << "Shift-idle, then press enter.\n";
	waitForEnter();

	return 0;
}
