#include <iostream>
#include <string>

#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>
#include <barrett/detail/stl_utils.h>

#include <barrett/standard_main_function.h>


using namespace barrett;
using detail::waitForEnter;


template <size_t DOF>
class JpCircle : public systems::SingleIO<double, typename units::JointPositions<DOF>::type> {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

public:
	JpCircle(jp_type startPos, double amplitude, double omega, const std::string& sysName = "JpCircle") :
		systems::SingleIO<double, jp_type>(sysName), jp(startPos), jp_0(jp), amp(amplitude), omega(omega) {
			// Check for robot type
			if (DOF>3) {
				// WAM - Use joints 3 and 4.
				i1 = 2;
				i2 = 3;
			} else if (DOF==3) {
				// Proficio - Use joints 2 and 3.
				i1 = 1;
				i2 = 2;
			} else {
				// This should never happen, since DOF check is done in main.
				std::cout << "Warning: No known robot with DOF < 3." << std::endl;
				i1 = 0;
				i2 = 0;
				amp = 0;
			}

		}
	virtual ~JpCircle() { this->mandatoryCleanUp(); }

protected:
	jp_type jp;
	jp_type jp_0;
	double amp, omega;
	double theta;
	int i1, i2;

	virtual void operate() {
		theta = omega * this->input.getValue();

		jp[i1] = amp * std::sin(theta) + jp_0[i1];
		jp[i2] = amp * (std::cos(theta) - 1.0) + jp_0[i2];

		this->outputValue->setData(&jp);
	}

private:
	DISALLOW_COPY_AND_ASSIGN(JpCircle);
};


class CpCircle : public systems::SingleIO<double, units::CartesianPosition::type> {
	BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;

public:
	CpCircle(cp_type startPos, double amplitude, double omega, const std::string& sysName = "CpCircle") :
		systems::SingleIO<double, cp_type>(sysName), cp(startPos), x_0(cp[0]), y_0(cp[1]), amp(amplitude), omega(omega) {}
	virtual ~CpCircle() { mandatoryCleanUp(); }

protected:
	cp_type cp;
	double x_0, y_0;
	double amp, omega;
	double theta;

	virtual void operate() {
		theta = omega * this->input.getValue();

		cp[0] = amp * (std::cos(theta) - 1.0) + x_0;
		cp[1] = amp * std::sin(theta) + y_0;

		this->outputValue->setData(&cp);
	}

private:
	DISALLOW_COPY_AND_ASSIGN(CpCircle);
};


template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	wam.gravityCompensate();


	const double TRANSITION_DURATION = 0.5;  // seconds

	const double JP_AMPLITUDE = 0.4;  // radians
	const double CP_AMPLITUDE = 0.1;  // meters
	const double FREQUENCY = 1.0;  // rad/s

	//Rate Limiter
	jp_type rt_jp_cmd;
	systems::RateLimiter<jp_type> jp_rl;
	//Sets the joints to move at 1 m/s
	const double rLimit[] = {1, 1, 1, 1, 1, 1, 1};

	for(size_t i = 0; i < DOF; ++i)
		rt_jp_cmd[i] = rLimit[i];

  // Set start position, depending on robot type and configuration.
	jp_type startPos(0.0);
	if (DOF > 3) {
		// WAM
		startPos[1] = -M_PI_2;
		startPos[3] = M_PI_2 + JP_AMPLITUDE;
	} else if (DOF == 3) {
		//Proficio
		jp_type temp = wam.getJointPositions();
		// Check configuration. Currently, there is no hardware support for determining
		// Proficio configuration. However, assuming that the user has run leftConfig or
		// rightConfig as needed, we can determine configuration from the angle of joint 3.
		// Joint 3 range is [0.12, 2.76] for right-configured Proficio and [-2.76, -0.12]
		// for left-configured Proficio.
		if (temp[2] < 0) {
			// left configuration
			startPos[2] = -1.25 + JP_AMPLITUDE;
		} else {
			// right configuration
			startPos[2] = 1.25 + JP_AMPLITUDE;
		}
	} else {
		std::cout << "Error: No known robot with DOF < 3. Quitting." << std::endl;
		// error
		return -1;
	}

	systems::Ramp time(pm.getExecutionManager(), 1.0);


	printf("Press [Enter] to move the end-point in circles using joint position control.");
	waitForEnter();

	wam.moveTo(startPos);
	//Indicate the current position and the maximum rate limit to the rate limiter
	jp_rl.setCurVal(wam.getJointPositions());
	jp_rl.setLimit(rt_jp_cmd);

	JpCircle<DOF> jpc(startPos, JP_AMPLITUDE, FREQUENCY);

	systems::connect(time.output, jpc.input);
	//Enforces that the individual joints move less than or equal to the above mentioned rate limit
	systems::connect(jpc.output, jp_rl.input);
	wam.trackReferenceSignal(jp_rl.output);
	time.smoothStart(TRANSITION_DURATION);

	printf("Press [Enter] to stop.");
	waitForEnter();
	time.smoothStop(TRANSITION_DURATION);
	wam.idle();


	printf("Press [Enter] to move the end-point in circles using Cartesian position control.");
	waitForEnter();

	wam.moveTo(startPos);
	CpCircle cpc(wam.getToolPosition(), CP_AMPLITUDE, FREQUENCY);

	time.reset();
	systems::connect(time.output, cpc.input);
	wam.trackReferenceSignal(cpc.output);
	time.smoothStart(TRANSITION_DURATION);

	printf("Press [Enter] to stop.");
	waitForEnter();
	time.smoothStop(TRANSITION_DURATION);
	wam.idle();

	printf("Press [Enter] to return home.");
	waitForEnter();
	wam.moveHome();
	wam.idle();

	// Wait for the user to press Shift-idle
	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
	return 0;
}
