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
	Wam<7> wam;

	systems::PIDController<Wam<7>::ja_type>* pid =
			new systems::PIDController<Wam<7>::ja_type>();
	systems::connect(wam.output, pid->feedbackInput);

	systems::Converter<Wam<7>::jt_type> supervisoryController;
	supervisoryController.registerConversion(pid);
	systems::connect(supervisoryController.output, wam.input);

	// tie inputs together for zero torque
	supervisoryController.connectInputTo(wam.output);


	Wam<7>::ja_type setPoint;
	setPoint << 0.000, -1.57, 0.0, 1.57, 0.0, 1.605, 0.0;
	systems::Constant<Wam<7>::ja_type> point(setPoint);

//	// TODO(dc): this should be done by the library
//	systems::System::Input<Wam<7>::ja_type>* feedbackInput =
//			dynamic_cast<systems::System::Input<Wam<7>::ja_type>*>(  //NOLINT: see RTTI note above
//			sc.selectController(wam.output).getFeedbackInput() );
//	systems::connect(wam.output, *feedbackInput);
//
//	systems::connect(sc.output, wam.input);
//	sc.connectInputTo(wam.output);

//	systems::PrintToStream<Wam<7>::jt_type> pts("JT: ");
//	systems::connect(pid->controlOutput, pts.input);

	std::cout << wam.operateCount << std::endl;

	std::cout << "Enter to gravity compensate.\n";
	waitForEnter();
	wam.gravityCompensate();

	std::cout << wam.operateCount << std::endl;
	usleep(1000000);
	std::cout << wam.operateCount << std::endl;

	std::cout << "Enter to move to set point.\n";
	waitForEnter();
//	systems::reconnect(point.output, pid.referenceInput);
	supervisoryController.connectInputTo(point.output);

	std::cout << "Enter to move home.\n";
	waitForEnter();
//	systems::reconnect(wam.output, pid.referenceInput);
	supervisoryController.connectInputTo(wam.output);
	wam.moveHome();

	std::cout << "Enter to idle.\n";
	waitForEnter();
	wam.gravityCompensate(false);
	wam.idle();

	std::cout << "Shift-idle, then press enter.\n";
	waitForEnter();

	return 0;
}
