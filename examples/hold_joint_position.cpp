#include <iostream>
#include <string>

#include <unistd.h>  // usleep()

#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/wam.h>


using namespace barrett;
using systems::connect;
using systems::reconnect;

const size_t DOF = 4;


void waitForEnter() {
	static std::string line;
	std::getline(std::cin, line);
}


int main() {
	Wam<DOF>::jp_type setPoint;
	setPoint << 0, -1.57, 0, 1.57;
	
	units::Array<DOF> tmp;


    // instantiate Systems
	Wam<DOF> wam;  // warning: lots of work is done in this constructor:
	               // reading config files, initializing CAN the bus, starting
	               // the real-time thread, etc.
	systems::PIDController<Wam<DOF>::jp_type> jpController;
	systems::Constant<Wam<DOF>::jp_type> point(setPoint);


    // configure Systems
	tmp << 3e3, 1e3, 1e2, 1e2;
	jpController.setKp(tmp);
	tmp << 25.0, 20.0, 15.0, 15.0;
	jpController.setControlSignalLimit(tmp);


    // make connections between Systems
	connect(wam.jpOutput, jpController.feedbackInput);
	connect(jpController.controlOutput, wam.input);

	// initially, tie inputs together for zero torque
	connect(wam.jpOutput, jpController.referenceInput);


    // interact with the user...
	std::cout << "Shift-activate the WAM, "
	        "then press [Enter] to gravity compensate.\n";
	waitForEnter();
	wam.gravityCompensate();

	std::cout << "Press [Enter] to move to set point.\n";
	waitForEnter();
	reconnect(point.output, jpController.referenceInput);

	std::cout << "Press [Enter] to move home.\n";
	waitForEnter();
	reconnect(wam.jpOutput, jpController.referenceInput);
	wam.moveHome();
	while ( !wam.moveIsDone() ) {
		usleep(100000);
	}

	std::cout << "Press [Enter] to idle.\n";
	waitForEnter();
	wam.gravityCompensate(false);
	wam.idle();

	std::cout << "Shift-idle the WAM, then press [Enter] to exit.\n";
	waitForEnter();

	return 0;
}
