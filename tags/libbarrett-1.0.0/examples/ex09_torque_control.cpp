#include <iostream>
#include <string>

#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>
#include <barrett/detail/stl_utils.h>

#include <barrett/standard_main_function.h>


using namespace barrett;
using detail::waitForEnter;


template<size_t DOF>
class J1Spring : public systems::SingleIO<typename units::JointPositions<DOF>::type, typename units::JointTorques<DOF>::type> {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

public:
	explicit J1Spring(double centerAngle, double springConstant = 10.0, const std::string& sysName = "J1Spring") :
		systems::SingleIO<jp_type, jt_type>(sysName), center(centerAngle), gain(springConstant), jt(0.0) {}
	virtual ~J1Spring() { this->mandatoryCleanUp(); }

protected:
	static const size_t J_IDX = 0;

	double center, gain;
	jt_type jt;
	double theta;

	virtual void operate() {
		theta = this->input.getValue()[J_IDX];
		if (theta < center) {
			jt[J_IDX] = 0.0;
		} else {
			jt[J_IDX] = (center - theta) * gain;
		}

		this->outputValue->setData(&jt);
	}

private:
	DISALLOW_COPY_AND_ASSIGN(J1Spring);
};


template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	wam.gravityCompensate();


	printf("Press [Enter] to add a virtual spring to J1 (positive joint range only).");
	waitForEnter();

	J1Spring<DOF> j1s(wam.getJointPositions()[0]);
	systems::connect(wam.jpOutput, j1s.input);
	wam.trackReferenceSignal(j1s.output);


	// Wait for the user to press Shift-idle
	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
	return 0;
}
