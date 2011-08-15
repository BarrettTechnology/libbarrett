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
				temp[0] = -2.6;
				break;
			case 1:
				temp[0] = -2.6; temp[1] = 2.0; temp[2] = -2.8; temp[3] = 3.0;
				break;
			case 2:
				temp[1] = -2.0; temp[2] = -2.8; temp[3] = -0.8;
				break;
			case 3:
				temp[1] = -1.0; temp[3] = -0.9;
				break;
			case 4:
				temp[4] = 1.24; temp[5] = -1.6;
				break;
			case 5:
				temp[4] = 1.24; temp[5] = 1.6;
				break;
		}
		setPoint = wam.getJointPositions();
		temp = j2mp*temp;
		temp[motor] -= 20.0;
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
		wam.moveTo(setPoint, jv_type(0.0), position, blocking, SPEED, SPEED);
		for(size_t i=0; i<DOF; i++) setPoint[i] = position[i];
	}

	// Distributes the slack for the given motor through the WAM.
	void distribute(int motor) {
		setPoint = wam.getJointPositions();
		jp_type stop1(0.0), stop2(0.0);
		switch (motor)
		{
			case 0:
				stop1[0] = -2.6;
				stop2[0] = 2.6;
				break;
			case 1:
				stop1[0] = -2.6; stop1[1] = 2.0; stop1[2] = -2.8; stop1[3] = 3.0;
				stop2[1] = -2.0; stop2[2] = 2.8; stop2[3] = -0.8;
				break;
			case 2:
				stop2[1] = -2.0; stop2[2] = -2.8; stop2[3] = -0.8;
				stop1[0] = 2.6; stop1[1] = 2.0; stop1[2] = 2.8; stop1[3] = 3.0;
				break;
			case 3:
				stop1[1] = -1.0; stop2[3] = -0.9;
				stop2[1] = -2.0; stop2[3] = 3.0;
				break;
			case 4:
				stop1[4] = 1.24; stop1[5] = -1.6;
				stop2[4] = -4.76; stop2[5] = 1.6;
				break;
			case 5:
				stop1[4] = 1.24; stop1[5] = 1.6;
				stop1[4] = -4.76; stop1[5] = -1.6;
				break;
		}
		for(int rep=0; rep<5; rep++)
		{
			moveTheWAM(stop1);
			moveTheWAM(stop2);
		}
	}

	// Rolls the given motor forward until it encounters a stop (hopefully the tang) or until it times out
	void rollForward(int motor, double timeout = 30.0) {
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
			moveTheWAM(jp_type(0.0));
			setTang(4, 1); // Open the tang
			while (true) {
				jp_type start = wam.getJointPositions();
				rollForward(4, 14);
				if (!watcher.watching) {
					break;
				}
				increment(5, start);
			}
			std::cout << "Found the M5 tang notch!\n";
			setPoint=wam.getJointPositions();
			rollForward(5);
			std::cout << "Found the M6 tang notch!\n";
			tangPositions[4] = fmod((j2mp*wam.getJointPositions())[4], 2*M_PI);
			tangPositions[5] = fmod((j2mp*wam.getJointPositions())[5], 2*M_PI);

			setPoint=wam.getJointPositions();
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
		
		double startPos = (j2mp * wam.getJointPositions())[motor]; // record our starting point
		double tangLoc = wam.getJointPositions()[motor];
		wam.moveTo(wam.getJointPositions()); // tell it to stay still
		torqueSetPoint.setValue(TENSION_TORQUES[motor]); // set the torque
		connect(torqueSetPoint.output, torqueSetter.getElementInput(motor)); // apply the torque
		//std::cout << "Applying torque now!\n";
		sleep(5.0); // wait a few seconds
		double endPos = (j2mp*wam.getJointPositions())[motor]; //record our ending point
		tangLoc = fabs(tangLoc - wam.getJointPositions()[motor]);
		disconnect(torqueSetPoint.output); //stop applying the torque

		setTang(tangMotor, 0); //Close the tang
		setPoint = wam.getJointPositions();
		jp_type temp = j2mp*setPoint;
		temp[motor] -= 1.0;
		temp = m2jp*temp;
		moveTheWAM(temp); // Back off the tang so it will close

		distribute(motor); // Run the WAM between extremes for the motor spindle to distribute tension
		tangPositions[motor] += tangLoc;
		return fabs(startPos-endPos); // Return how much slack we've taken up
	}

public:

	// Run the tensioning procedure
	int tension(int argc, char** argv)
	{	//std::cout << argc << " " << argv[1][0] << " " << argv[1][1] << "\n";
		bool toDo[DOF] = {false}; // = {false, false, false, false, false, false, false};
		if (argc > 1) {
			if (argv[1][0] == 'w') { // If the 1st argument is 'w', do the wrist only
				toDo[4] = true;
				toDo[5] = true;
				std::cout << "Tensioning wrist ONLY\n";
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

		if (DOF>4) {
			std::cout << "Finding the Wrist tang locations now\n";
			findTangs(); // find the tangs for the wrist if it's there
		}
		for (size_t j = 0; j<DOF; j++)
		{
			int motor;
			if (toDo[j]) {
				motor = j;
			}
			else {
				continue;
			}
			notCanceled = true;
			double multiplier = 9000;
			if (motor>4) multiplier = 6600;

			std::cout << "Doing motor " << motor+1 << " now...\n";			
			double slackTaken = 1.0, last = 2.0;
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
				slackTaken = iterate(motor);
				totalRuns++;
				std::cout << slackTaken*multiplier << " micrometers slack removed\n";
			}
		}

		moveTheWAM(wam.getHomePosition()); // go home
		pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
		return 0;
	}

	// Constructor
	Autotensioner(ProductManager& p, systems::Wam<DOF>& w) :
		pm(p), wam(w),
		j2mtSys(w.llww.getLowLevelWam().getJointToMotorTorqueTransform()),
		m2jtSys(w.llww.getLowLevelWam().getJointToMotorPositionTransform().transpose()),
		m2jpSys(w.llww.getLowLevelWam().getMotorToJointPositionTransform()),
		j2mpSys(w.llww.getLowLevelWam().getJointToMotorPositionTransform()),
		watcher(p.getExecutionManager()),
		motorPos(p.getExecutionManager(), 0.8)
	{	
		// Get information from the WAM
		pucks = wam.llww.getLowLevelWam().getPucks();
		m2jp = wam.llww.getLowLevelWam().getMotorToJointPositionTransform();
		j2mp = wam.llww.getLowLevelWam().getJointToMotorPositionTransform();
		m2jt = j2mp.transpose();
		j2mt = wam.llww.getLowLevelWam().getJointToMotorTorqueTransform();
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
