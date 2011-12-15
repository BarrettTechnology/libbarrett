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
 * behave correctly. To do this, run the bt-wam-gravitycal program and then
 * update the information in the appropriate configuration file. The gravity
 * compensation and calibration routines rely on having an accurately zeroed
 * robot, so consider performing zero calibration (bt-wam-zerocal) first. These
 * routines also rely on knowing the orientation of the WAM relative to gravity.
 * The default configuration files assume the WAM is in its standard orientation
 * (the plane containing the four mounting holes is horizontal). Make sure to
 * adjust the "world_to_base" homogeneous transform in the configuration files
 * if your WAM is mounted in a different orientation.
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
	// See note on "Holding tool orientation" below
	systems::ExposedOutput<Eigen::Quaterniond> orientationSetPoint;
	systems::ExposedOutput<typename systems::Wam<DOF>::pose_type> poseSetPoint;

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

			// Note that we can't use the systems::Wam<DOF>::moveTo() function
			// to hold tool orientation like we do in the other cases. We should
			// be able to, but the ability to smoothly transition from one
			// orientation to an other has not been implemented in libbarrett
			// yet. Stay tuned! (If you look at the implementation of the
			// Wam::moveTo() function, you'll see that it makes use of the same
			// Wam::trackReferenceSignal() function that we use below.)
			orientationSetPoint.setValue(wam.getToolOrientation());
			wam.trackReferenceSignal(orientationSetPoint.output);
			break;

		case 'b':
			printf("Holding both tool position and orientation.\n");
			poseSetPoint.setValue(wam.getToolPose());
			wam.trackReferenceSignal(poseSetPoint.output);
			break;

		case 'i':
			printf("WAM idled.\n");

			// Note that this use of the word "idle" does not mean "Shift-idle".
			// Calling Wam::idle() will remove any of the controllers that may
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
