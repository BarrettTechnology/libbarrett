/* Transforms forces to world frame, go to a few poses and acquire mass at end effector*/

#include <stdio.h>

#include<libconfig.h>

#include <barrett/math.h>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>
#include <barrett/standard_main_function.h>

using namespace barrett;

enum STATE {
	PLAYING, TOGGLING, TARING, QUIT
} curState = PLAYING, lastState = PLAYING;

//FeatherTouch Class
template<size_t DOF>
class FeatherTouch {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
protected:
	systems::Wam<DOF>& wam;
	ProductManager& pm;
	libconfig::Config conf;

	ForceTorqueSensor* fts;
	cf_type cCurrent, cOffset, cfAdjusted, cfTransformed;
	jp_type jpzero, jpi1, jpi2, jpi3, jpi4, jpi5, jpiY, jpiY2, jpStart;
	jt_type cutoff;
	Eigen::Quaterniond to;
	math::Matrix<3, 3, double> rot_mat;
	std::vector<double> massOff;
	systems::ToolForceToJointTorques<DOF> tf2jt;
	systems::FirstOrderFilter<jt_type> lpf;
	systems::RateLimiter<cf_type> cfRL;
	systems::ExposedOutput<cf_type> cfCmd;
	double massOffset;

public:
	bool ftOn;
	FeatherTouch(systems::Wam<DOF>& wam_, ProductManager& pm_) :
			wam(wam_), pm(pm_), fts(NULL), cutoff(100.0),massOff(6) {
	}
	bool
	init();

	~FeatherTouch() {
	}

	void
	calibrate();
	void
	start();
	void
	displayEntryPoint();
	void
	update();
	void
	tare();
	void
	connectFT();
	void
	disconnectFT();
}
;

// Initialization
template<size_t DOF>
bool FeatherTouch<DOF>::init() {
	wam.gravityCompensate();
	pm.getSafetyModule()->setVelocityLimit(1.5);

	if (pm.foundForceTorqueSensor()) {
		fts = pm.getForceTorqueSensor();
	} else {
		printf("No Force Torque Sensor Found - EXITING\n");
		return (false);
	}

	try {
		conf.readFile("feather_touch.conf");
	} catch (const libconfig::FileIOException &fioex) {
		printf("I/O error while reading feather_touch.conf - EXITING\n");
		btsleep(5.0);
		return (false);
	} catch (const libconfig::ParseException &pex) {
		printf("Parse error at %s: %d - %s - EXITING\n", pex.getFile(),
				pex.getLine(), pex.getError());
		btsleep(5.0);
		return (false);
	}

	try {
		massOffset = conf.lookup("ee_mass");
		printf("End-Effector Mass from Configuration File = %.3f\n",
				massOffset);
	} catch (const libconfig::SettingNotFoundException &nfex) {
		printf("No 'ee_mass' setting in configuration file.");
		return (false);
	}

	printf("Moving to WAM Zero and FT Tare\n\n");
	jpi1[1] = -1.57079633; // -X
	jpi2[1] = 1.57079633; // +X
	jpi3 = jpi2;
	jpi3[2] = -1.57079633; // +Y
	jpi4 = jpi3;
	jpi4[2] = 1.57079633; // -Y
	jpi5 = jpi2;
	jpi5[3] = 1.57079633; // +Z

	jpiY = jpi5;
	jpiY[2] -= 1.57079633;
	jpiY2 = jpiY;
	jpiY2[2] += 3.14159;

	jpStart[1] = 0.448;
	jpStart[2] = 0.046;
	jpStart[3] = 2.178;

	lpf.setLowPass(cutoff);
	return true;
}

// Calibration
template<size_t DOF>
void FeatherTouch<DOF>::calibrate() {
	printf("\nCalibrating!\n");
	// -X
	wam.moveTo(jpi1, true);
	sleep(2.0);
	fts->update();
	cCurrent = fts->getForce();
	cOffset[2] = cCurrent[2];
	cCurrent[2] -= cOffset[2];
	printf("Offset Values -X: X =  %.4f Y = %.4f Z = %.4f\n", cCurrent[0],
			cCurrent[1], cCurrent[2]);
	//Transformed
	to = wam.getToolOrientation();
	rot_mat = to.toRotationMatrix();
	cfTransformed = rot_mat.inverse() * cCurrent;
	printf("Transformed Values -X: X = %.4f Y = %.4f Z = %.4f\n",
			cfTransformed[0], cfTransformed[1], cfTransformed[2]);
	massOff[0] = -cfTransformed[2];

	// +X
	wam.moveTo(jpi2, true);
	sleep(2.0);
	fts->update();
	cCurrent = fts->getForce();
	cCurrent[2] -= cOffset[2];
	printf("Offset Values +X: X =  %.4f Y = %.4f Z = %.4f\n", cCurrent[0],
			cCurrent[1], cCurrent[2]);
	to = wam.getToolOrientation();
	rot_mat = to.toRotationMatrix();
	cfTransformed = rot_mat.inverse() * cCurrent;
	printf("Transformed Values +X: X = %.4f Y = %.4f Z = %.4f\n",
			cfTransformed[0], cfTransformed[1], cfTransformed[2]);
	massOff[1] = -cfTransformed[2];

	// +Y
	wam.moveTo(jpi3, true);
	sleep(2.0);
	fts->update();
	cCurrent = fts->getForce();
	cCurrent[2] -= cOffset[2];
	printf("Offset Values +Y: X =  %.4f Y = %.4f Z = %.4f\n", cCurrent[0],
			cCurrent[1], cCurrent[2]);
	to = wam.getToolOrientation();
	rot_mat = to.toRotationMatrix();
	cfTransformed = rot_mat.inverse() * cCurrent;
	printf("Transformed Values +Y: X = %.4f Y = %.4f Z = %.4f\n",
			cfTransformed[0], cfTransformed[1], cfTransformed[2]);
	massOff[2] = -cfTransformed[2];

	//Second positive Y
	wam.moveTo(jpiY, true);
	sleep(2.0);
	fts->update();
	cCurrent = fts->getForce();
	cCurrent[2] -= cOffset[2];
	printf("Second - Offset Values +Y: X =  %.4f Y = %.4f Z = %.4f\n",
			cCurrent[0], cCurrent[1], cCurrent[2]);
	to = wam.getToolOrientation();
	rot_mat = to.toRotationMatrix();
	cfTransformed = rot_mat.inverse() * cCurrent;
	printf("Transformed Values +Y: X = %.4f Y = %.4f Z = %.4f\n",
			cfTransformed[0], cfTransformed[1], cfTransformed[2]);
	massOff[2] = -cfTransformed[2];

	// -Y
	wam.moveTo(jpi4, true);
	sleep(2.0);
	fts->update();
	cCurrent = fts->getForce();
	cCurrent[2] -= cOffset[2];
	printf("Offset Values -Y: X =  %.4f Y = %.4f Z = %.4f\n", cCurrent[0],
			cCurrent[1], cCurrent[2]);
	to = wam.getToolOrientation();
	rot_mat = to.toRotationMatrix();
	cfTransformed = rot_mat.inverse() * cCurrent;
	printf("Transformed Values -Y: X = %.4f Y = %.4f Z = %.4f\n",
			cfTransformed[0], cfTransformed[1], cfTransformed[2]);
	massOff[3] = -cfTransformed[2];

	//Second negative Y
	wam.moveTo(jpiY2, true);
	sleep(2.0);
	fts->update();
	cCurrent = fts->getForce();
	cCurrent[2] -= cOffset[2];
	printf("Second - Offset Values -Y: X =  %.4f Y = %.4f Z = %.4f\n",
			cCurrent[0], cCurrent[1], cCurrent[2]);
	to = wam.getToolOrientation();
	rot_mat = to.toRotationMatrix();
	cfTransformed = rot_mat.inverse() * cCurrent;
	printf("Second - Transformed Values -Y: X = %.4f Y = %.4f Z = %.4f\n",
			cfTransformed[0], cfTransformed[1], cfTransformed[2]);
	massOff[2] = -cfTransformed[2];

	// +Z
	wam.moveTo(jpi5, true);
	sleep(2.0);
	fts->update();
	cCurrent = fts->getForce();
	cCurrent[2] -= cOffset[2];
	printf("Offset Values +Z: X =  %.4f Y = %.4f Z = %.4f\n", cCurrent[0],
			cCurrent[1], cCurrent[2]);
	to = wam.getToolOrientation();
	rot_mat = to.toRotationMatrix();
	cfTransformed = rot_mat.inverse() * cCurrent;
	printf("Transformed Values +Z: X = %.4f Y = %.4f Z = %.4f\n",
			cfTransformed[0], cfTransformed[1], cfTransformed[2]);
	massOff[4] = -cfTransformed[2];

	// -Z
	wam.moveTo(jpzero, true);
	sleep(2.0);
	fts->update();
	cCurrent = fts->getForce();
	cCurrent[2] -= cOffset[2];
	printf("Offset Values -Z: X =  %.4f Y = %.4f Z = %.4f\n", cCurrent[0],
			cCurrent[1], cCurrent[2]);
	to = wam.getToolOrientation();
	rot_mat = to.toRotationMatrix();
	cfTransformed = rot_mat.inverse() * cCurrent;
	printf("Transformed Values -Z: X = %.4f Y = %.4f Z = %.4f\n",
			cfTransformed[0], cfTransformed[1], cfTransformed[2]);
	massOff[5] = -cfTransformed[2];

	massOffset = 0.0;
	for (int i = 0; i < 6; i++)
		massOffset += massOff[i];
	massOffset = massOffset / 6.0;

	printf("\nThe End Effector Weighs Approximately %f grams\n\n",
			(massOffset / 9.81) * 1000);

	libconfig::Setting &setting = conf.lookup("ee_mass");
	setting = massOffset;
	conf.writeFile("feather_touch.conf");
	printf("Calibration saved to file: feather_touch.conf\n\n");
}

// Starting Feather Touch
template<size_t DOF>
void FeatherTouch<DOF>::tare() {
	wam.moveTo(jpzero, true);
	sleep(1.0);
	fts->tare();
	sleep(2.0);
}

// Starting Feather Touch
template<size_t DOF>
void FeatherTouch<DOF>::start() {
	wam.moveTo(jpStart);
	wam.idle();

	ftOn = true;
}

template<size_t DOF>
void FeatherTouch<DOF>::update() {
	fts->update();
	cCurrent = fts->getForce();
	cCurrent[2] -= cOffset[2];
	to = wam.getToolOrientation();
	rot_mat = to.toRotationMatrix();
	cfTransformed = rot_mat.inverse() * cCurrent;
	cfTransformed[2] += massOffset;
	cfCmd.setValue(cfTransformed);
}

template<size_t DOF>
void FeatherTouch<DOF>::disconnectFT() {
	wam.idle();
	ftOn = false;
}

template<size_t DOF>
void FeatherTouch<DOF>::connectFT() {
	systems::forceConnect(wam.kinematicsBase.kinOutput, tf2jt.kinInput);
	cfCmd.setValue(cf_type(0.0));
	cfRL.setLimit(cf_type(500.0)); //Hz
	systems::forceConnect(cfCmd.output, cfRL.input);
	systems::forceConnect(cfRL.output, tf2jt.input);
	systems::forceConnect(tf2jt.output,lpf.input);
	wam.trackReferenceSignal(lpf.output);
	ftOn = true;
}

template<size_t DOF>
void FeatherTouch<DOF>::displayEntryPoint() {
	// Instructions displayed to screen.
	printf("\n");
	printf("Commands:\n");
	printf("  f    Toggle feather touch on/off\n");
	printf("  t    Re-Tare the FT Sensor\n");
	printf("  q    Quit\n");
	printf("\n");

	std::string line;
	while (true) {
		// Continuously parse our line input
		std::getline(std::cin, line);
		switch (line[0]) {
		case 'f':
			printf("\nTurning Feather Touch %s\n\n", ftOn ? "OFF" : "ON");
			printf(">> ");
			curState = TOGGLING;
			break;
		case 't':
			printf("\nMoving to Zero and FT Tare\n\n");
			printf(">> ");
			curState = TARING;
			break;
		case 'q':
			printf("\nQuitting - Moving the WAM Home\n");
			curState = QUIT;
			break;
		default:
			printf(">> ");
			break;
		}
	}
}

template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm,
		systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	FeatherTouch<DOF> feather_touch(wam, pm);

	if (!feather_touch.init())
		return 1;

	feather_touch.tare();

	char calibrate;
	printf("Would you like to Calibrate the FT Sensor for Feather Touch?? (y/n) \n");
	scanf("%c", &calibrate);

	if (calibrate == 'y')
		feather_touch.calibrate();

	feather_touch.start();
	feather_touch.connectFT();

	boost::thread displayThread(&FeatherTouch<DOF>::displayEntryPoint,
			&feather_touch);

	bool running = true;

	while (running) {
		switch (curState) {
		case QUIT:
			running = false;
			break;
		case PLAYING:
			feather_touch.update();
			lastState = PLAYING;
			btsleep(0.0125);//80 Hz update rate
			break;
		case TOGGLING:
			switch (lastState) {
			case PLAYING:
				if (feather_touch.ftOn)
					feather_touch.disconnectFT();
				else
					feather_touch.connectFT();
				curState = PLAYING;
				lastState = PLAYING;
				break;
			default:
				btsleep(0.1);
				break;
			}
			break;
		case TARING:
			switch (lastState) {
			case PLAYING:
				feather_touch.tare();
				feather_touch.start();
				feather_touch.connectFT();
				curState = PLAYING;
				lastState = PLAYING;
				break;
			default:
				btsleep(0.1);
				break;
			}
			break;
		}
	}

	wam.moveHome();
	printf("\n\n");
	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
	return 0;
}
