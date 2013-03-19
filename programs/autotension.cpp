/*
 File: autotension.cpp
 Date: 9 January, 2012
 Author: Kyle Maroney
 */

#include <unistd.h>
#include <math.h>
#include <vector>
#include <algorithm>

#include <libconfig.h++>

#include <barrett/math.h>
#include <barrett/units.h>
#include <barrett/exception.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>
#include <barrett/products/puck.h>

using namespace barrett;

const char CAL_CONFIG_FILE[] = "/etc/barrett/autotension.conf";

template<size_t DOF>
std::vector<int> validate_args(int argc, char** argv) {
	// Some DOF dependent items
	int jnts2Tens = 4;
	std::vector<int> args;
	if (DOF == 7)
		jnts2Tens = 6;

	// Check our arguments and create vector accordingly.
	if (argc == 1) {
		printf("\nNo arguments provided - Default\n");
		printf("Program will autotension joints 1-%d\n\n", (DOF == 4) ? 4 : 6);
		args.resize(jnts2Tens);
		for (int i = 0; i < jnts2Tens; i++)
			args[i] = i + 1;
	} else {
		args.resize(argc - 1);
		for (int i = 0; i < argc - 1; i++) {
			if (atoi(argv[i + 1]) < 1 || atoi(argv[i + 1]) > jnts2Tens) {
				printf("\nEXITING: Arguments out of range - Must provide valid joints 1-%d (ex. 1 2 4)\n\n", (DOF == 4) ? 4 : 6);
				args.resize(0);
				return args;
			}
		}
		printf("\nProgram will autotension joints ");
		for (int j = 0; j < argc - 1; j++) {
			printf("%d", atoi(argv[j + 1]));
			args[j] = atoi(argv[j + 1]);
			if (j < argc - 2)
				printf(", ");
			else
				printf("\n\n");
		}
	}

	return args;
}

// TorqueWatchdog monitoring system
template<size_t DOF>
class TorqueWatchdog: public systems::SingleIO<typename barrett::units::JointTorques<DOF>::type, typename barrett::units::JointTorques<DOF>::type> {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

public:
	explicit TorqueWatchdog(const std::string& sysName = "TorqueWatchdog") :
			barrett::systems::SingleIO<jt_type, jt_type>(sysName) {
		watching = false, min_torque = 0.0, max_torque = 0.0;
	}

	virtual ~TorqueWatchdog() {
		this->mandatoryCleanUp();
	}

public:
	void activate(int m, double l) {
		motor = m;
		limit = l;
		watching = true;
	}

	void deactivate() {
		watching = false;
	}

protected:

	virtual void operate() {
		if (watching) {
			torques = this->input.getValue();
			if (fabs(torques[motor]) > limit)
				watching = false;
			if (torques[motor] > max_torque)
				max_torque = torques[motor];
			if (torques[motor] < min_torque)
				min_torque = torques[motor];
		}
	}

public:
	bool watching;
	int motor;
	double limit;
	double max_torque, min_torque;
	jt_type torques;

private:
	DISALLOW_COPY_AND_ASSIGN(TorqueWatchdog);
};

// ExposedInput monitoring system
template<typename T>
class ExposedInput: public systems::System, public systems::SingleInput<T> {
public:
	explicit ExposedInput(const std::string& sysName = "ExposedInput") :
			systems::System(sysName), systems::SingleInput<T>(this) {
	}

	virtual ~ExposedInput() {
		this->mandatoryCleanUp();
	}

	T getValue() {
		return value;
	}

protected:

	virtual void operate() {
		value = this->input.getValue();
	}

public:
	T value;

private:
	DISALLOW_COPY_AND_ASSIGN(ExposedInput);
};

//AutoTension Class
template<size_t DOF>
class AutoTension {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
protected:
	systems::Wam<DOF>& wam;
	const libconfig::Setting& setting;
	std::vector<Puck*> puck;

	TorqueWatchdog<DOF> watchdog;
	ExposedInput<jt_type> exposedGravity;
	systems::Gain<jt_type, sqm_type, jt_type> j2mtSys, gravtor2mtSys;
	systems::Gain<jt_type, sqm_type, jt_type> m2jtSys;
	systems::Gain<jp_type, sqm_type, jp_type> m2jpSys;
	systems::Gain<jp_type, sqm_type, jp_type> j2mpSys;
	systems::ArrayEditor<jp_type> mpCommand;
	systems::ArrayEditor<jt_type> mtCommand;
	systems::ExposedOutput<jp_type> holdJP;
	systems::ExposedOutput<double> tensionValue;
	systems::Ramp motorRamp;

	std::vector<jp_type> jpInitial, jpStart, jpSlack1, jpSlack2;
	jp_type tensionDefaults, jpStopHigh, jpStopLow, tangBuffer, tangMiss, stopBuffer, slackThreshold;
	sqm_type j2mp;
	double motorSlackPulled, j1SlackPulled, j5TangPos, j6TangPos;
	int motor;

public:
	Hand* hand;
	AutoTension(systems::Wam<DOF>& wam_, const libconfig::Setting& setting_) :
			wam(wam_), setting(setting_), puck(wam.getLowLevelWam().getPucks()), j2mtSys(wam.getLowLevelWam().getJointToMotorTorqueTransform()), gravtor2mtSys(
					wam.getLowLevelWam().getJointToMotorTorqueTransform()), m2jtSys(wam.getLowLevelWam().getJointToMotorPositionTransform().transpose()), m2jpSys(
					wam.getLowLevelWam().getMotorToJointPositionTransform()), j2mpSys(wam.getLowLevelWam().getJointToMotorPositionTransform()), motorRamp(NULL,
					0.8), tensionDefaults(setting["tens_default"]), jpStopHigh(setting["jp_stop_high"]), jpStopLow(setting["jp_stop_low"]), tangBuffer(
					setting["tang_buffer"]), tangMiss(setting["tang_miss"]), stopBuffer(setting["stop_buffer"]), slackThreshold(setting["slack_thresh"]), j2mp(
					wam.getLowLevelWam().getJointToMotorPositionTransform()), j5TangPos(0.0), j6TangPos(0.0), hand(NULL) {
	}
	void
	init(ProductManager& pm, std::vector<int> args);

	void updateJ1Moves(int tensionMotor, bool addJ1) {
		if (addJ1) {
			jpStart[tensionMotor][0] = jpStopLow[0] + tangBuffer[0];
			jpSlack1[tensionMotor][0] = jpStart[tensionMotor][0] - tangBuffer[0] / 2.0;
			jpSlack2[tensionMotor][0] = jpStopHigh[0] - tangBuffer[0] / 2.0;
		} else {
			jpStart[tensionMotor][0] = 0.0;
			jpSlack1[tensionMotor][0] = 0.0;
			jpSlack2[tensionMotor][0] = 0.0;
		}
	}

	void updateJ5(double jpInc, bool reset = false) {
		if (!reset)
			jpStart[5][4] -= jpInc * M_PI / 180.0;
		else
			jpStart[5][4] = jpStopHigh[4] - tangBuffer[4];
	}

	void updateJ6(double jpInc, bool reset = false) {
		if (!reset)
			jpStart[4][5] += jpInc * M_PI / 180.0;
		else
			jpStart[4][5] = jpStopLow[5] + tangBuffer[5];
	}

	~AutoTension() {
	}

	std::vector<int> tensionJoint(std::vector<int> joint_list);
	bool engage(int motor, double timeout);
	double pullTension(int motor);
	void connectSystems();
};

template<size_t DOF>
void AutoTension<DOF>::init(ProductManager& pm, std::vector<int> args) {
	wam.gravityCompensate(true); // Turning on Gravity Compenstation

	// Make sure all tangs are released correctly
	for (size_t m = 0; m < args.size(); m++) {
		if (args[m] == 6) // Motor 6 tensions using the single tang for 5 & 6
			puck[4]->setProperty(Puck::TENSION, false);
		else
			puck[args[m] - 1]->setProperty(Puck::TENSION, false);
	}

	wam.moveHome(); // Move the WAM to home position

	// Tell our EM to start managing
	pm.getExecutionManager()->startManaging(motorRamp); //starting ramp manager
	pm.getExecutionManager()->startManaging(watchdog);
	pm.getExecutionManager()->startManaging(exposedGravity);

	// Some DOF dependent items
	jpInitial.resize(DOF);
	jpStart.resize(DOF);
	jpSlack1.resize(DOF);
	jpSlack2.resize(DOF);

	// Initialize hand if present
	if (pm.foundHand()) {
		jp_type handBufJT = wam.getJointPositions();
		if (DOF == 4)
			handBufJT[3] -= 0.35; // Lift the elbow to give room for BHand HI
		else
			handBufJT[5] = jpStopLow[5]; // Set J6 on its positive stop
		wam.moveTo(handBufJT);
		hand = pm.getHand();
		hand->initialize();
		hand->close(Hand::GRASP);
	}

	// Joint 1 Tensioning will be added in and run simultaneously if present.
	jpInitial[0] = wam.getJointPositions();

	// Joint 2
	jpInitial[1] = wam.getJointPositions();
	jpStart[1] = jpInitial[1];
	jpStart[1][1] = jpStopHigh[1] - tangBuffer[1];
	jpStart[1][2] = jpStopLow[2] + tangBuffer[2];
	jpStart[1][3] = jpStopHigh[3] - stopBuffer[3];
	jpSlack1[1] = jpStart[1];
	jpSlack1[1][1] = jpStopHigh[1] - stopBuffer[1]; // This is so we dont drive into and wear the joint stops
	jpSlack1[1][2] = jpStopLow[2] + stopBuffer[2];
	jpSlack2[1] = jpSlack1[1];
	jpSlack2[1][1] = jpStopLow[1] + stopBuffer[1];
	jpSlack2[1][2] = jpStopHigh[2] - stopBuffer[2];
	jpSlack2[1][3] = jpStopLow[3] + stopBuffer[3];

	// Joint 3
	jpInitial[2] = wam.getJointPositions();
	jpInitial[2][1] = 0.0;
	jpStart[2] = jpInitial[2];
	jpStart[2][1] = jpStopLow[1] + tangBuffer[1];
	jpStart[2][2] = jpStopLow[2] + tangBuffer[2];
	jpStart[2][3] = jpStopLow[3] + stopBuffer[3];
	jpSlack1[2] = jpStart[2];
	jpSlack1[2][1] = jpStopLow[1] + stopBuffer[1];
	jpSlack1[2][2] = jpStopLow[2] + stopBuffer[1];
	jpSlack2[2] = jpSlack1[2];
	jpSlack2[2][1] = jpStopHigh[1] - stopBuffer[1];
	jpSlack2[2][2] = jpStopHigh[2] - stopBuffer[2];
	jpSlack2[2][3] = jpStopHigh[3] - stopBuffer[3];

	// Joint 4
	jpInitial[3] = wam.getJointPositions();
	jpStart[3] = jpInitial[3];
	jpStart[3][1] = 0.0;
	jpStart[3][3] = jpStopLow[3] + tangBuffer[3];
	jpSlack1[3] = jpStart[3];
	jpSlack1[3][3] = jpStopLow[3] + stopBuffer[3];
	jpSlack2[3] = jpSlack1[3];
	jpSlack2[3][3] = jpStopHigh[3] - stopBuffer[3];

	if (DOF == 7) {
		// Joint 5
		jpInitial[4] = wam.getJointPositions();
		jpStart[4] = jpInitial[4];
		jpStart[4][3] = M_PI / 2.0;
		jpStart[4][4] = jpStopHigh[4] - tangBuffer[4];
		jpStart[4][5] = jpStopLow[5] + tangBuffer[5];
		jpSlack1[4] = jpStart[4];
		jpSlack1[4][4] = jpStopHigh[4] - stopBuffer[4];
		jpSlack1[4][5] = jpStopLow[5] + stopBuffer[5];
		jpSlack2[4] = jpSlack1[4];
		jpSlack2[4][4] = jpStopLow[4] + stopBuffer[4];
		jpSlack2[4][5] = jpStopHigh[5] - stopBuffer[5];

		// Joint 6
		jpInitial[5] = wam.getJointPositions();
		jpStart[5] = jpInitial[5];
		jpStart[5][3] = M_PI / 2.0;
		jpStart[5][4] = jpStopHigh[4] - tangBuffer[4];
		jpStart[5][5] = jpStopHigh[5] - tangBuffer[5];
		jpSlack1[5] = jpStart[5];
		jpSlack1[5][4] = jpStopHigh[4] - stopBuffer[4];
		jpSlack1[5][5] = jpStopHigh[5] - stopBuffer[5];
		jpSlack2[5] = jpSlack1[5];
		jpSlack2[5][4] = jpStopLow[4] + stopBuffer[4];
		jpSlack2[5][5] = jpStopLow[5] + stopBuffer[5];
	}
	// Connect our systems and tell the supervisory control to control jpController in motor space.
	connectSystems();
}

template<size_t DOF>
std::vector<int> AutoTension<DOF>::tensionJoint(std::vector<int> joint_list) {
	int joint = joint_list[joint_list.size() - 1];
	bool j1tens = false;
	bool diff_tens = false;
	if (std::find(joint_list.begin(), joint_list.end(), 1) != joint_list.end())
		j1tens = true;

	motor = joint - 1; // Joint indexing

	while (motorSlackPulled > slackThreshold[motor] || !diff_tens) // Check to see if we have met the slack thresholds
	{
		motorSlackPulled = 1.0; // Large initial slack value for comparison against threshold
		j1SlackPulled = 1.0;
		if (std::find(joint_list.begin(), joint_list.end(), 1) != joint_list.end())
			j1tens = true;
		wam.moveTo(jpInitial[motor], 1.2, 0.75);
		printf("\n**************************\n");
		switch (joint) {
		case 2:
		case 3:
			diff_tens = false;
			joint = 3;
			motor = 2;
			updateJ1Moves(1, j1tens);
			updateJ1Moves(2, j1tens);
			printf("Tensioning Joint %s\n", (j1tens && joint != 1) ? "1, 2, and 3" : "2 and 3");
			// Pull tension from J2
			wam.moveTo(jpStart[1], 1.2, 0.75);
			puck[1]->setProperty(Puck::TENSION, true);
			btsleep(2.0);
			if (!engage(1)) {
				puck[1]->setProperty(Puck::TENSION, false);
				joint_list.resize(0);
				printf("Error - Failed to engage autotensioner for joint 2\n");
				return joint_list;
			} else
				printf("Successfully engaged joint 2 autotensioner.\n");
			btsleep(0.5);
			if (pullTension(1) < slackThreshold[1])
				diff_tens = true;
			puck[1]->setProperty(Puck::TENSION, false);
			break;
		case 5:
		case 6:
			diff_tens = false;
			joint = 6;
			motor = 5;
			updateJ1Moves(4, j1tens);
			updateJ1Moves(5, j1tens);
			printf("Tensioning Joint %s\n", (j1tens && joint != 1) ? "1, 5, and 6" : "5 and 6");
			// Pull tension from J5
			wam.moveTo(jpStart[4], 1.2, 0.75);
			puck[4]->setProperty(Puck::TENSION, true);
			if (j6TangPos == 0.0) {
				while (!engage(5, 10.0)) {
					updateJ6(3.5);
					wam.moveTo(jpStart[4]);
					if (wam.getJointPositions()[5] < jpStopLow[5] + tangMiss[5]) {
						puck[4]->setProperty(Puck::TENSION, false);
						joint_list.resize(0);
						printf("Error - Failed to engage autotensioner for joint 6\n");
						return joint_list;
					}
					wam.moveTo(jpStart[4]);
				}
				j6TangPos = wam.getJointPositions()[5];
			} else {
				jpStart[4][5] = j6TangPos + tangBuffer[5];
				wam.moveTo(jpStart[4]);
				if (!engage(5)) { // Engage joint 6
					puck[4]->setProperty(Puck::TENSION, false);
					joint_list.resize(0);
					printf("Error - Failed to engage autotensioner for joint 6\n");
					return joint_list;
				}
			}
			printf("Successfully engaged joint 6 autotensioner.\n");
			if (!engage(4)) { // Engage joint 5
				puck[4]->setProperty(Puck::TENSION, false);
				joint_list.resize(0);
				printf("Error - Failed to engage autotensioner for joint 5\n");
				return joint_list;
			}
			printf("Successfully engaged joint 5 autotensioner.\n");
			updateJ6(0.0, true);
			btsleep(0.5);
			if (pullTension(4) < slackThreshold[4])
				diff_tens = true;
			puck[4]->setProperty(Puck::TENSION, false); // Release Tang
			break;
		default:
			updateJ1Moves(motor, j1tens);
			printf("Tensioning Joint%s%d\n", (j1tens && joint != 1) ? " 1 and Joint " : " ", joint);
			diff_tens = true;
			break;
		}

		wam.moveTo(jpStart[motor], 1.2, 0.75);

		if (motor != 0 && j1tens) { // Engage tang and pull tension from J1
			puck[0]->setProperty(Puck::TENSION, true);
			if (!engage(0)) {
				puck[0]->setProperty(Puck::TENSION, false);
				joint_list.resize(0);
				printf("Error - Failed to engage autotensioner for joint 1\n");
				return joint_list;
			} else
				printf("Successfully engaged joint 1 autotensioner.\n");
			btsleep(0.5);
			j1SlackPulled = pullTension(0);
			puck[0]->setProperty(Puck::TENSION, false);
		}

		//Engage tang for specified motor
		if (joint != 6)
			puck[motor]->setProperty(Puck::TENSION, true);
		btsleep(2.0);

		if (joint == 6) {
			puck[4]->setProperty(Puck::TENSION, true);
			btsleep(2.0);
			if (j5TangPos == 0.0) {
				while (!engage(4, 10.0)) {
					updateJ5(3.5);
					wam.moveTo(jpStart[motor]);
					if (wam.getJointPositions()[4] < jpStopLow[4] + tangMiss[4]) {
						puck[4]->setProperty(Puck::TENSION, false);
						joint_list.resize(0);
						printf("Error - Failed to engage autotensioner for joint 5\n");
						return joint_list;
					}
					wam.moveTo(jpStart[motor]);
				}
				j5TangPos = wam.getJointPositions()[4];
			} else {
				jpStart[motor][4] = j5TangPos + tangBuffer[4];
				wam.moveTo(jpStart[motor]);
				if (!engage(4)) { // Engage joint 5
					puck[4]->setProperty(Puck::TENSION, false);
					joint_list.resize(0);
					printf("Error - Failed to engage autotensioner for joint 5\n");
					return joint_list;
				}
			}
			printf("Successfully engaged joint 5 autotensioner.\n");
			if (!engage(motor)) { // Engage joint 6
				puck[4]->setProperty(Puck::TENSION, false);
				joint_list.resize(0);
				printf("Error - Failed to engage autotensioner for joint 6\n");
				return joint_list;
			}
			printf("Successfully engaged joint 6 autotensioner.\n");
			updateJ5(0.0, true); // Reset J6 Start values
			btsleep(0.5);
			motorSlackPulled = pullTension(5);
			puck[4]->setProperty(Puck::TENSION, false); // Release Tang
		} else if (engage(motor)) {
			printf("Successfully engaged joint %d autotensioner.\n", joint);
			fflush(stdout);
			btsleep(0.5);
			//Pull Tension
			motorSlackPulled = pullTension(motor);
			puck[motor]->setProperty(Puck::TENSION, false);
		} else {
			puck[motor]->setProperty(Puck::TENSION, false);
			joint_list.resize(0);
			printf("Error - Failed to engage autotensioner for joint %d\n", joint);
			return joint_list;
		}

		int rep_cnt = 0;
		int reps = 5; // Number of times to travel through joint range to distribute slack.
		printf("Distributing slack through joints\n");
		while (rep_cnt < reps) {
			printf("%dx.. ", reps - rep_cnt);
			fflush(stdout);
			wam.moveTo(jpSlack2[motor], 1.2, 0.75);
			wam.moveTo(jpSlack1[motor], 1.2, 0.75);
			if (joint == 3) {
				wam.moveTo(jpSlack1[1], 1.2, 0.75);
				wam.moveTo(jpSlack2[1], 1.2, 0.75);
			}
			if (joint == 6) {
				wam.moveTo(jpSlack1[4], 1.2, 0.75);
				wam.moveTo(jpSlack2[4], 1.2, 0.75);
			}
			rep_cnt++;
		}
		printf("\n");
		wam.moveTo(jpInitial[motor], 1.2, 0.75);
		wam.moveHome();

		//Remove J1 from list if enough tension pulled
		if (j1SlackPulled < slackThreshold[0]) {
			printf("Successfully Tensioned Joint 1\n");
			joint_list.erase(std::remove(joint_list.begin(), joint_list.end(), 1), joint_list.end());
			j1tens = false;
		}
	}
	switch (joint) {
	case 2:
	case 3:
		printf("Successfully Tensioned Joint 2 and 3\n");
		joint_list.erase(std::remove(joint_list.begin(), joint_list.end(), 2), joint_list.end());
		joint_list.erase(std::remove(joint_list.begin(), joint_list.end(), 3), joint_list.end());
		break;
	case 5:
	case 6:
		printf("Successfully Tensioned Joint 5 and 6\n");
		joint_list.erase(std::remove(joint_list.begin(), joint_list.end(), 5), joint_list.end());
		joint_list.erase(std::remove(joint_list.begin(), joint_list.end(), 6), joint_list.end());
		break;
	default:
		printf("Successfully Tensioned Joint %d\n", joint);
		joint_list.erase(std::remove(joint_list.begin(), joint_list.end(), joint), joint_list.end());
		break;
	}
	return joint_list;
}

template<size_t DOF>
bool AutoTension<DOF>::engage(int motor, double timeout = 20.0) {
	btsleep(1.0); // Let system settle, tang engage
	holdJP.setValue(wam.getJointPositions()); // Hold Position
	motorRamp.setSlope(0.8);
	motorRamp.setOutput((j2mp * wam.getJointPositions())[motor]); //Set the ramp to the proper start position
	wam.trackReferenceSignal(m2jpSys.output); // Start following the motor position controller
	systems::forceConnect(motorRamp.output, mpCommand.getElementInput(motor)); // Increase the commanded motor position using the ramp.
	motorRamp.smoothStart(10); // start the ramp
	double timer = 0.0;
	btsleep(0.1); // eliminate obscure transient torque readings
	watchdog.activate(motor, 0.5); // Start watching for the torque spike
	while (watchdog.watching && timer < timeout && wam.getJointPositions()[motor] > jpStopLow[motor] + tangMiss[motor]
			&& wam.getJointPositions()[motor] < jpStopHigh[motor] - tangMiss[motor]) {
		btsleep(0.05); // waiting for the torque spike indicating the tang has engaged
		timer += 0.05;
	}

	if (watchdog.watching == true)
		return false;

	watchdog.deactivate();
	motorRamp.stop(); // stop the ramp
	holdJP.setValue(wam.getJointPositions()); // Update the hold position
	systems::disconnect(motorRamp.output); // disconnect the ramp leaving the commanded motor position at the value.
	return true;
}

template<size_t DOF>
double AutoTension<DOF>::pullTension(int motor) {
	double start_pos = (j2mp * wam.getJointPositions())[motor]; // record our starting point
	tensionValue.setValue(tensionDefaults[motor] - exposedGravity.getValue()[motor]); // set the torque - this will be the torque applied to pull tension subtracting the torque already being applied by the gravity compensator
	systems::connect(tensionValue.output, mtCommand.getElementInput(motor)); // apply the torque
	sleep(5.0); // wait a few seconds
	double end_pos = (j2mp * wam.getJointPositions())[motor]; //record our ending point
	tensionValue.setValue(0.0);
	systems::disconnect(tensionValue.output); //stop applying the torque
	btsleep(2.0);

	holdJP.setValue(wam.getJointPositions()); // Update setPoint to account for movement
	printf("Slack taken up from motor %d  = %f radians\n", motor + 1, fabs(start_pos - end_pos));
	return fabs(start_pos - end_pos);
}

template<size_t DOF>
void AutoTension<DOF>::connectSystems() {
// Position control portion
	systems::connect(holdJP.output, j2mpSys.input);
	systems::connect(j2mpSys.output, mpCommand.input);
	systems::connect(mpCommand.output, m2jpSys.input);

// System allowing us to eliminate torques due to gravity comp.
	systems::connect(wam.gravity.output, gravtor2mtSys.input);
	systems::connect(gravtor2mtSys.output, exposedGravity.input);

// Torque control Portion
	systems::connect(wam.jpController.controlOutput, j2mtSys.input);
	systems::connect(j2mtSys.output, mtCommand.input);
	systems::connect(j2mtSys.output, watchdog.input);
	systems::connect(mtCommand.output, m2jtSys.input);

	wam.supervisoryController.registerConversion(makeIOConversion(wam.jpController.referenceInput, m2jtSys.output));
}

//wam_main Function
template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	/* Read configuration file */
	libconfig::Config config;
	try {
		config.readFile(CAL_CONFIG_FILE);
	} catch (const libconfig::FileIOException &fioex) {
		printf("EXITING: I/O error while reading %s\n", CAL_CONFIG_FILE);
		btsleep(5.0);
		return (false);
	} catch (const libconfig::ParseException &pex) {
		printf("EXITING: Parse error at %s: %d - %s\n", pex.getFile(), pex.getLine(), pex.getError());
		btsleep(5.0);
		return (false);
	}
	const libconfig::Setting& setting = config.lookup("autotension")[pm.getWamDefaultConfigPath()];

	std::vector<int> arg_list = validate_args<DOF>(argc, argv);
	if (arg_list.size() == 0) {
		pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
		return 1;
	}

	printf("Press <Enter> to begin autotensioning.\n");
	detail::waitForEnter();

	AutoTension<DOF> autotension(wam, setting);
	autotension.init(pm, arg_list);

	/* Autotension Specified Joints*/
	while (arg_list.size() != 0)
		arg_list = autotension.tensionJoint(arg_list);

	/* Re-fold and exit */
	if (autotension.hand != 0) {
		autotension.hand->open(Hand::GRASP);
		autotension.hand->close(Hand::SPREAD);
		autotension.hand->trapezoidalMove(Hand::jp_type(M_PI / 2.0), Hand::GRASP);
	}
	wam.moveHome();
	printf("\n**************************\n");
	printf("Autotensioning Routine Complete\n");
	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
	return 0;
}

int main(int argc, char** argv) {

	// Message to user
	printf("\n"
			"                  *** Barrett WAM Autotensioning Utility ***\n"
			"\n"
			"This utility will autotension the specified cables of your WAM Arm.\n"
			"Cable tensioning is necessary after signs of the WAM cables becoming\n"
			"loose after extended use, or after replacing any cables on your WAM Arm.\n"
			"After completion of this routine, you must zero calibrate and gravity\n"
			"calibrate the WAM, as pulling tension from the cables will introduce\n"
			"offsets to your previous calibrations.\n"
			"\n"
			"WAMs with serial numbers < 5 and WAM wrists with serial numbers < 9 are not\n"
			"eligible for autotensioning.\n"
			"\n"
			"This program assumes the WAM is mounted such that the base is horizontal.\n"
			"\n"
			"\n");

	// For clean stack traces
	barrett::installExceptionHandler();

	// Create our product manager
	ProductManager pm;
	pm.waitForWam();

	if (pm.foundWam4()) {
		return wam_main<4>(argc, argv, pm, *pm.getWam4(true, NULL));
	} else if (pm.foundWam7()) {
		return wam_main<7>(argc, argv, pm, *pm.getWam7(true, NULL));
	}
}
