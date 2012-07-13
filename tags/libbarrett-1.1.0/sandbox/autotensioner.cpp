/*
 * autotensioner.cpp
 *
 *  Created on: Jun 28, 2011
 *      Author: CJ Valle
 *
 * >>> INSTRUCTIONS <<<
 *  The utility may be run with one of three "types" of arguments:
 *    1 - No arguments.  In this case the WAM will tension all of its joints
 *    2 - With the argument 'w'.  In this case the WAM will tension the wrist. (./autotensioner w)
 *    3 - With a list of numeric arguments representing motor ID's.  In this case the WAM
 *        will tension the joints given as arguments. (./autotensioner 2 4)
 */

#include <iostream>
#include <string>
#include <signal.h>

#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>
#include <barrett/detail/stl_utils.h>
#include <barrett/standard_main_function.h>

#include <Eigen/LU>

#include "tensioner.h"

using namespace barrett;
using systems::connect;
using systems::disconnect;

const double SPEED = 1.0; // How fast we'll move the arm while we're going
const double WIGGLE = 5.0; // Max allowable difference between iterations (micrometers)
const double TENSION_TORQUES[7] = {0.85, 0.85, 0.85, 0.5, 0.23, 0.23, 0.0};
		// Torques we'll apply to tension (newton-meters)
const double HIGH_STOP[7] = {2.6, 2.0, 2.8, 3.1, 1.24, 1.6, 3.0};
const double LOW_STOP[7] = {-2.6, -2.0, -2.8, -0.9, -4.76, -1.6, -3.0};

bool notCanceled = true; // to handle CTRL-C interrupts
void sigint_handler(int steve) {
	notCanceled = false;
	std::cout << "\nCancel Signal received. Further operations on this motor have been cancelled.\n";
	signal(SIGINT, &sigint_handler);
}

template<size_t DOF>
class Autotensioner
{	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

private:
	ProductManager& pm; // Product Manager
	systems::Wam<DOF>& wam; // The WAM!!!
	std::vector<Puck*> pucks; // Pucks
	sqm_type m2jp, j2mp, m2jt, j2mt; // Transforms
	jp_type setPoint, tangPositions; // to keep track of positions

	// Systems
	systems::Gain<jt_type, sqm_type, jt_type> j2mtSys;
	systems::Gain<jt_type, sqm_type, jt_type> m2jtSys;
	systems::Gain<jp_type, sqm_type, jp_type> m2jpSys;
	systems::Gain<jp_type, sqm_type, jp_type> j2mpSys;
	systems::ArrayEditor<jt_type> torqueSetter;
	systems::ArrayEditor<jp_type> positionSetter;
	systems::ExposedOutput<double> torqueSetPoint;
	systems::ExposedOutput<jp_type> holdSetPoint;
	systems::Tensioner<DOF> watcher;
	systems::Ramp motorPos;


	// Set the tang for the given motor to the given state
	void setTang(int motor, int state)
	{
		pucks[motor]->setProperty(Puck::TENSION, state);
	}

	// Wraps usleep to guarantee it returns on time, and to provide SECONDS as an input
	void sleep(double duration) {
		unsigned long total = duration*1000000;
		unsigned long passed = 0;
		unsigned long remaining = total;
		RTIME start = rt_timer_read();
		while (true) {
			usleep(remaining);
			passed = (rt_timer_read() - start) / 1000;
			if (passed >= total) break;
			remaining = total - passed;
		}
	}

	// Finds the a point about 3 motor revolutions away from the end of the spindle of the given motor
	void findPosition(int motor)
	{	
		jp_type temp(0.0);
		switch(motor)
		{
			case 0:
				temp[0] = LOW_STOP[0];
				break;
			case 1:
				temp[0] = LOW_STOP[0]; temp[1] = HIGH_STOP[1]; temp[2] = LOW_STOP[2]; temp[3] = HIGH_STOP[3];
				break;
			case 2:
				temp[1] = LOW_STOP[1]; temp[2] = LOW_STOP[2]; temp[3] = LOW_STOP[3];
				break;
			case 3:
				temp[3] = LOW_STOP[3];
				break;
			case 4:
				temp[4] = HIGH_STOP[4]; temp[5] = LOW_STOP[5];
				break;
			case 5:
				temp[4] = HIGH_STOP[4]; temp[5] = HIGH_STOP[5];
				break;
		}
		setPoint = wam.getJointPositions();
		temp = j2mp*temp;
		if (motor>3) {
			temp[motor] -= 6*M_PI;
		}
		else {
			temp[motor] -= 8*M_PI;
		}
		for (size_t i=0; i<DOF; i++) {
			temp[i] = temp[i] - fmod(temp[i], 2*M_PI) + tangPositions[i] - 0.4;
					// Puts all tang notches just behind the tangs
		}
		temp = m2jp * temp;
		moveTheWAM(temp);
	}

	// Increments the given motor forward
	void increment(int motor, jp_type temp) {
		temp = j2mp*temp;
		temp[motor] += .3;
		temp = m2jp*temp;
		moveTheWAM(temp);
	}

	// Moves the WAM to the given position, handling high torques against stops smoothly
	void moveTheWAM(jp_type position, bool blocking = true) {
		wam.moveTo(setPoint, /*jv_type(0.0),*/ position, blocking, SPEED, SPEED);
		for(size_t i=0; i<DOF; i++) setPoint[i] = position[i];
	}

	// Distributes the slack for the given motor through the WAM.
	void distribute(int motor) {
		setPoint = wam.getJointPositions();
		for(int rep=0; rep<5; rep++) {
			jp_type stop1(0.0), stop2(0.0);
			switch (motor)
			{
				case 0:
					stop1[0] = LOW_STOP[0];
					stop2[0] = HIGH_STOP[0];
					break;
				case 1:
					stop1[0] = LOW_STOP[0]; stop1[1] = HIGH_STOP[1]; stop1[2] = LOW_STOP[2]; stop1[3] = (rand()%300)/100;
					stop2[1] = LOW_STOP[1]; stop2[2] = HIGH_STOP[2]; stop2[3] = -((rand()%60)+20.0)/100.0;
					break;
				case 2:
					stop2[1] = LOW_STOP[1]; stop2[2] = LOW_STOP[2]; stop2[3] = -((rand()%60)+20.0)/100.0;
					stop1[0] = HIGH_STOP[0]; stop1[1] = HIGH_STOP[1]; stop1[2] = HIGH_STOP[2]; stop1[3] = (rand()%300)/100.0;
					break;
				case 3:
					stop1[1] = -0.7; stop1[3] = LOW_STOP[3];
					stop2[1] = -0.7; stop2[3] = HIGH_STOP[3];
					break;
				case 4:
					stop1[4] = HIGH_STOP[4]; stop1[5] = LOW_STOP[5];
					stop2[4] = LOW_STOP[4]; stop2[5] = HIGH_STOP[5];
					break;
				case 5:
					stop1[4] = HIGH_STOP[4]; stop1[5] = HIGH_STOP[5];
					stop2[4] = LOW_STOP[4]; stop2[5] = LOW_STOP[5];
					break;
			}
		moveTheWAM(stop1);
		moveTheWAM(stop2);
		}
	}

	// Rolls the given motor forward until it encounters a stop (hopefully the tang) or until it times out
	void rollForward(int motor, double timeout = 14.0) { // Timeout of 14 seconds is roughly 1.25 revolutions
		holdSetPoint.setValue(setPoint); // Tell it to hold position
		watcher.activate(motor, 0.5); // Start watching for the torque spike
		motorPos.setOutput((j2mp*setPoint)[motor]); //Set the ramp to the proper start position
		wam.trackReferenceSignal(m2jpSys.output); // Start following the controller
		connect(motorPos.output, positionSetter.getElementInput(motor)); // connect the ramp to the controller
		motorPos.smoothStart(10); // start the ramp
		double time = 0.0;
		while(watcher.watching && time < timeout) {
			sleep(0.1); // wait for the torque spike
			time += 0.1;
		}
		motorPos.stop(); // stop the ramp
		disconnect(motorPos.output); // disconnect the ramp
		setPoint = wam.getJointPositions();
	}

	// finds the tang locations for the wrist
	void findTangs() {
		if (DOF > 4) {
			moveTheWAM(jp_type(0.0)); // Move to zero position
			setTang(4, 1); // Open the tang
			while (true) {
				jp_type start = wam.getJointPositions(); // Remember where we start
				rollForward(4); // Roll forward to hit the tang
				if (!watcher.watching) { // If we actually hit the tang go on
					break;
				}
				increment(5, start); // If we timed out increment the other motor from our starting position
			}
			std::cout << "Found the M5 tang notch!\n";
			setPoint=wam.getJointPositions(); // Update setPoint to account for motion
			rollForward(5); // Roll forward on to the other tang
			std::cout << "Found the M6 tang notch!\n";
			tangPositions[4] = fmod((j2mp*wam.getJointPositions())[4], 2*M_PI); // Record the tang positions relative to the motor's arbitrary "zero"
			tangPositions[5] = fmod((j2mp*wam.getJointPositions())[5], 2*M_PI);

			setPoint=wam.getJointPositions(); // Update to account for motion
			setTang(4, 0); // Close the tang
			jp_type temp = j2mp*setPoint;
			temp[4] -= 1.0;
			temp[5] -= 1.0;
			temp = m2jp*temp;
			moveTheWAM(temp); // Back off the tang so it will close
		}
	}

	// Performs a single iteration on the given motor and returns the amount of slack taken up in Rads
	double iterate(int motor) {
		int tangMotor = motor; // declares that the tang we want is attached to the motor we are using
		if (motor == 5) tangMotor = 4; // handles the wrist case

		findPosition(motor); // Get to the ~3 rev away position
		holdSetPoint.setValue(setPoint); // Tell it to hold position
		setTang(tangMotor, 1); // Open the tang
		rollForward(motor); // Rolls forward into the tang
		setTang(tangMotor, 0); // Disengane the tang to save the Puck

		double startPos = (j2mp * wam.getJointPositions())[motor]; // record our starting point
		wam.moveTo(wam.getJointPositions()); // tell it to stay still
		torqueSetPoint.setValue(TENSION_TORQUES[motor]); // set the torque
		connect(torqueSetPoint.output, torqueSetter.getElementInput(motor)); // apply the torque
		sleep(5.0); // wait a few seconds
		double endPos = (j2mp*wam.getJointPositions())[motor]; //record our ending point
		disconnect(torqueSetPoint.output); //stop applying the torque
		
		setPoint = wam.getJointPositions(); // Update setPoint to account for movement
		jp_type temp = j2mp*setPoint;
		temp[motor] -= 1.0;
		temp = m2jp*temp;
		moveTheWAM(temp); // Back off the tang so it will close

		distribute(motor); // Run the WAM between extremes for the motor spindle to distribute tension
		tangPositions[motor] = fmod(endPos, 2*M_PI); // Update the tang position
		return fabs(startPos-endPos); // Return how much slack we've taken up
	}

public:

	// Run the tensioning procedure
	int tension(int argc, char** argv) {
		bool toDo[DOF] = {false}; // Array that determines which motors we'll do
		if (argc > 1) { // If we got an argument
			if (argv[1][0] == 'w') { // If the 1st argument is 'w', do the wrist only
				if (DOF > 4) {
					toDo[4] = true;
					toDo[5] = true;
					std::cout << "Tensioning wrist ONLY\n";
				}
				else {
					std::cout << "No wrist detected.  Exiting...\n";
				}
			}
			else { // If there is a list of motors for arguments, do those
				std::cout << "Tensioning motors: ";
				for (int i=1; i<argc; i++) {
					size_t num = strtol(argv[i], 0, 10);
					if (num <= DOF && num > 0) {
						toDo[num-1] = true;
						std::cout << num << " ";
					}
				}
				std::cout << "\n";
			}
		}
		else { // If there are no arguments just do the entire WAM
			for (size_t i=0; i<DOF-1; i++) {
				toDo[i] = true;
			}
			std::cout << "Tensioning entire WAM";
		}

		if (DOF>4) { // If we have a wrist (This could be expanded to find all joint tangs to make the overall process faster)
			if (toDo[4] || toDo[5]) { // If we are actually tensioning the wrist
				std::cout << "Finding the wrist tang locations now\n";
				findTangs(); // find the tangs for the wrist if it's there
			}
		}
		for (size_t j = 0; j<DOF; j++) // For each motor in the system
		{
			int motor; // The motor we'll tension
			if (toDo[j]) { // If we're tensioning this motor set it as the motor to do
				motor = j;
			}
			else { // Else check the next motor
				continue;
			}
			notCanceled = true;
			double multiplier = 9000; // This value represents the number of micrometers of cable per radian on the motor shaft
			if (motor>4) multiplier = 6600;  // This case handles the (smaller) wrist

			std::cout << "Doing motor " << motor+1 << " now...\n";			
			double slackTaken = 1.0, last = 2.0; // Represents the amount of slack taken up this iteration and the time before
			int totalRuns = 0;
			/* There are three exit cases for each motor's tensioning process:
			 *  1 - The same amount of "slack" is pulled twice AND it's had at least 3 iterations
			 *  2 - The total number of iterations is greater than or equal to 10
			 *  3 - The CTRL-C interrupt takes place
			 * In all cases, the program progresses on to the next motor in the sequence
			*/
			while((fabs(slackTaken-last) > WIGGLE/multiplier || totalRuns < 3) && totalRuns < 10 && notCanceled) 
			{
				last = slackTaken;
				slackTaken = iterate(motor); // Do an iteration
				totalRuns++;
				std::cout << slackTaken*multiplier << " micrometers slack removed\n";
			}
		}

		moveTheWAM(wam.getHomePosition()); // go home
		pm.getSafetyModule()->waitForMode(SafetyModule::IDLE); // Wait for idle
		return 0; // Exit
	}

	// Constructor
	Autotensioner(ProductManager& p, systems::Wam<DOF>& w) :
		pm(p), wam(w),
		j2mtSys(w.getLowLevelWam().getJointToMotorTorqueTransform()),
		m2jtSys(w.getLowLevelWam().getJointToMotorPositionTransform().transpose()),
		m2jpSys(w.getLowLevelWam().getMotorToJointPositionTransform()),
		j2mpSys(w.getLowLevelWam().getJointToMotorPositionTransform()),
		watcher(p.getExecutionManager()),
		motorPos(p.getExecutionManager(), 0.8)
	{	
		// Get information from the WAM
		pucks = wam.getLowLevelWam().getPucks();
		m2jp = wam.getLowLevelWam().getMotorToJointPositionTransform();
		j2mp = wam.getLowLevelWam().getJointToMotorPositionTransform();
		m2jt = j2mp.transpose();
		j2mt = wam.getLowLevelWam().getJointToMotorTorqueTransform();
		setPoint = wam.getJointPositions();

		// Set up systems
		connect(holdSetPoint.output, j2mpSys.input);
		connect(j2mpSys.output, positionSetter.input);
		connect(positionSetter.output, m2jpSys.input);

		connect(wam.jpController.controlOutput, j2mtSys.input);
		connect(j2mtSys.output, torqueSetter.input);
		connect(j2mtSys.output, watcher.input);
		connect(torqueSetter.output, m2jtSys.input);
		wam.supervisoryController.registerConversion(
				makeIOConversion(wam.jpController.referenceInput, m2jtSys.output));

		notCanceled = true;
	}
};

template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& p, systems::Wam<DOF>& w)
{
	Autotensioner<DOF> myTensioner(p, w); // Create tensioner object
	signal(SIGINT, &sigint_handler); // Register sigint handler
	return myTensioner.tension(argc, argv); // Start the process!!
}
