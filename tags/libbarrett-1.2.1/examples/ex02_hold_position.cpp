/* ex02_hold_position.cpp
 *
 * This example turns on gravity compensation and then allows the user to
 * constrain the position of the arm using various controllers.
 *
 * Gravity compensation uses cumulative first-moment of mass data (stored in the
 * configuration files in /etc/barrett/) to calculate in real time what torques
 * should be applied to the motors in order to support the arm's weight in
 * gravity. Gravity compensation is typically used in combination with other
 * control strategies; gravity compensation keeps the arm from falling, while
 * the other control strategies make it do something interesting. Unless you
 * have a specific reason not to, you should turn gravity compensation on at the
 * beginning of a WAM program and leave it on for the duration. (Feel free to
 * experiment with how turning gravity compensation on and off affects the WAM's
 * behavior!)
 *
 * If you change the WAM's mass distribution (for instance, by attaching a tool
 * to the end), you'll need to recalibrate before gravity compensation will
 * behave correctly. To do this, run the bt-wam-gravitycal program. This will
 * update the information in the appropriate configuration file for you. The
 * gravity compensation and calibration routines rely on having an accurately
 * zeroed robot, so consider performing zero calibration (bt-wam-zerocal) first.
 * These routines also rely on knowing the orientation of the WAM relative to
 * gravity. The default configuration files assume the WAM is in its standard
 * orientation (the plane containing the four mounting holes is horizontal).
 * Make sure to adjust the "world_to_base" homogeneous transform in the
 * configuration files if your WAM is mounted in a different orientation.
 */


#include <iostream>
#include <string>

// Eigen is the C++ math library that libbarrett uses. See:
//     http://eigen.tuxfamily.org/dox-2.0/
#include <Eigen/Geometry>

#include <barrett/systems.h>
#include <barrett/products/product_manager.h>

#include <barrett/standard_main_function.h>


using namespace barrett;


void printMenu() {
	printf("Commands:\n");
	printf("  j  Hold joint positions\n");
	printf("  p  Hold tool position (in Cartesian space)\n");
	printf("  o  Hold tool orientation\n");
	printf("  b  Hold both tool position and orientation\n");
	printf("  i  Idle (release position/orientation constraints)\n");
	printf("  q  Quit\n");
}

template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam) {
	wam.gravityCompensate();
	printMenu();

	std::string line;
	bool going = true;
	while (going) {
		printf(">>> ");
		std::getline(std::cin, line);

		switch (line[0]) {
		case 'j':
			printf("Holding joint positions.\n");
			wam.moveTo(wam.getJointPositions());
			break;

		case 'p':
			printf("Holding tool position.\n");
			wam.moveTo(wam.getToolPosition());
			break;

		case 'o':
			printf("Holding tool orientation.\n");
			wam.moveTo(wam.getToolOrientation());
			break;

		case 'b':
			printf("Holding both tool position and orientation.\n");
			wam.moveTo(wam.getToolPose());
			break;

		case 'i':
			printf("WAM idled.\n");

			// Note that this use of the word "idle" does not mean "Shift-idle".
			// Calling Wam::idle() will disable any of the controllers that may
			// be connected (joint position, tool position, tool orientation,
			// etc.) leaving only gravity compensation. (More specifically,
			// Wam::idle() disconnects any inputs that were connected using
			// Wam::trackReferenceSignal().)
			wam.idle();
			break;

		case 'q':
		case 'x':
			printf("Quitting.\n");
			going = false;
			break;

		default:
			if (line.size() != 0) {
				printf("Unrecognized option.\n");
				printMenu();
			}
			break;
		}
	}

	// Release the WAM if we're holding. This is convenient because it allows
	// users to move the WAM back to some collapsed position before exiting, if
	// they want.
	wam.idle();

	// Wait for the user to press Shift-idle
	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
	return 0;
}
