/*
 * autotensioner.cpp
 *
 *  Created on: Jun 28, 2011
 *      Author: CJ Valle
 */

#include <iostream>
#include <string>
#include <cmath>

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

template<size_t DOF>
class Autotensioner
{	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

private:
	ProductManager& pm; // Product Manager
	systems::Wam<DOF>& wam; // The WAM!!!
	std::vector<Puck*> pucks; // Pucks
	sqm_type m2jp, j2mp, m2jt, j2mt; // Transforms
	jp_type setPoint, tangPositions;

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
		std::cout << "Applying torque now!\n";
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
	int tension()
	{
		if (DOF>4) findTangs(); // find the tangs for the wrist if it's there
		for (size_t i=0; i<DOF; i++)
		{
			double multiplier = 9000;
			if (i>4) multiplier = 6600;

			std::cout << "Doing motor " << i+1 << " now...\n";			
			double slackTaken = 1.0, last = 2.0;
			int totalRuns = 0;
			while((fabs(slackTaken-last) > WIGGLE/multiplier || totalRuns < 3) && totalRuns < 10) 
			{
				last = slackTaken;
				slackTaken = iterate(i);
				totalRuns++;
				std::cout << "Slack removed this iteration: " << slackTaken*multiplier << " micrometers\n";
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
	}
};

template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& p, systems::Wam<DOF>& w)
{
	Autotensioner<DOF> myTensioner(p, w);
	return myTensioner.tension();
}
