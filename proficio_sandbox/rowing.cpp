/** 
 * @file rowing.cpp
 * @author Jonathan Hagstrand
 * @date 08/14/14
 */

/**
 *	Rowing Simulation Game. 
 *  Movements are damped based on the level
 *
 */

#include <iostream>

#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>
#include <barrett/standard_main_function.h>

using namespace barrett;

class Rowing : public systems::SingleIO<typename units::CartesianVelocity::type, typename units::CartesianForce::type>{
	BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;
	
public:
	Rowing(const std::string& sysName = "Rowing") : systems::SingleIO<cv_type,cf_type>(sysName), d(10) {}
	virtual ~Rowing() {mandatoryCleanUp();}
	
	void changeDampFactor(double dampFactor){d = dampFactor;}
	
protected:
	double d;
	cf_type result;
	virtual void operate(){
		const cv_type v = this->input.getValue();
		
		result = d * (-1 * (cf_type)v );
		
		this->outputValue->setData(&result);
	}
	
private:
	DISALLOW_COPY_AND_ASSIGN(Rowing);
};

template<size_t DOF>
typename units::JointTorques<DOF>::type saturateJt(
		const typename units::JointTorques<DOF>::type& x,
		const typename units::JointTorques<DOF>::type& limit) {
	int index;
	double minRatio;

	minRatio = (limit.cwise() / (x.cwise().abs())).minCoeff(&index);
	if (minRatio < 1.0) {
		return minRatio * x;
	} else {
		return x;
	}
}

template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam){
	
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
	jt_type Limits(45.0);
	jp_type start; start << 0.25, 0.0, 1.57;
	
	Rowing rowingHaptics;
	systems::ToolForceToJointTorques<DOF> tf2jt;
	systems::Callback<jt_type> jtSat(boost::bind(saturateJt<DOF>, _1, Limits));
	
	connect(wam.toolVelocity.output, rowingHaptics.input);
	connect(rowingHaptics.output, tf2jt.input);
	connect(wam.kinematicsBase.kinOutput, tf2jt.kinInput);
	connect(tf2jt.output, jtSat.input);
	
  wam.gravityCompensate();
	wam.moveTo(start);
	wam.idle();
	
	connect(jtSat.output, wam.input);
	
	bool active = true;
	std::string line, line2;
	while(active){
		printf(">>> ");
		std::getline(std::cin, line);
		
		switch(line[0]){
			case'd':
			case'D':
				printf("Chaning Damp Factor to: ");
				std::getline(std::cin, line2);
				rowingHaptics.changeDampFactor((double) line2[0]);
				break;
			case'q':
			case'Q':
				printf("Moving Home and quitting!\n");
				active = false;
				break;
			default:
				printf("Only options are D or Q is to quit.\n>>> ");
				break;
		}
	}
	
	wam.moveHome();
	printf("\n\n");
	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
	return 0;
}
