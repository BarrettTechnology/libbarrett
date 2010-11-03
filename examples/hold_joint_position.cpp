#include <iostream>
#include <string>

#include <barrett/systems.h>


using namespace barrett;

const size_t DOF = 4;
const double T_s = 0.002;
BARRETT_UNITS_TYPEDEFS(DOF);  // declares jp_type


void waitForEnter() {
	static std::string line;
	std::getline(std::cin, line);
}


int main() {
	libconfig::Config config;
	config.readFile("/etc/barrett/wam4.conf");

	systems::RealTimeExecutionManager rtem(T_s);
	systems::System::defaultExecutionManager = &rtem;


	// instantiate Systems
	systems::Wam<DOF> wam(config.lookup("wam"));
	systems::ExposedOutput<jp_type> setPoint;


	// start the main loop!
	rtem.start();

	std::cout << "Shift-activate the WAM, "
	        "then press [Enter] to compensate for gravity.\n";
	waitForEnter();
	wam.gravityCompensate();

	std::string line;
	bool going = true, holding = false;
	std::cout << "Press 'h' to toggle holding position. Press 'q' to exit." << std::endl;
	while (going) {
		std::cout << ">>> ";
		std::getline(std::cin, line);

		switch (line[0]) {
		case 'h':
			holding = !holding;
			if (holding) {
				SCOPED_LOCK(rtem.getMutex());

				setPoint.setValue(wam.getJointPositions());
				wam.trackReferenceSignal(setPoint.output);
			} else {
				wam.idle();
			}
			break;

		case 'q':
		case 'x':
			going = false;
			break;

		default:
			break;
		}
	}

	std::cout << "Shift-idle the WAM, then press [Enter] to exit.\n";
	waitForEnter();
	rtem.stop();

	return 0;
}
