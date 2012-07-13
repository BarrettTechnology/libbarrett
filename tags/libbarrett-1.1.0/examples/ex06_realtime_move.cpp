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
		systems::SingleIO<double, jp_type>(sysName), jp(startPos), j3_0(jp[2]), j4_0(jp[3]), amp(amplitude), omega(omega) {}
	virtual ~JpCircle() { this->mandatoryCleanUp(); }

protected:
	jp_type jp;
	double j3_0, j4_0;
	double amp, omega;
	double theta;

	virtual void operate() {
		theta = omega * this->input.getValue();

		jp[2] = amp * std::sin(theta) + j3_0;
		jp[3] = amp * (std::cos(theta) - 1.0) + j4_0;

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

	jp_type startPos(0.0);
	startPos[1] = -M_PI_2;
	startPos[3] = M_PI_2 + JP_AMPLITUDE;

	systems::Ramp time(pm.getExecutionManager(), 1.0);


	printf("Press [Enter] to move the end-point in circles using joint position control.");
	waitForEnter();

	wam.moveTo(startPos);
	JpCircle<DOF> jpc(startPos, JP_AMPLITUDE, FREQUENCY);

	systems::connect(time.output, jpc.input);
	wam.trackReferenceSignal(jpc.output);
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


	// Wait for the user to press Shift-idle
	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
	return 0;
}
