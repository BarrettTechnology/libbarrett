#include <iostream>
#include <string>
#include <time.h>

#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>
#include <barrett/detail/stl_utils.h>

#include <barrett/standard_main_function.h>

const double jmin[] = {-1.57, 0.785, 0.0, +1.57, -0.785, -0.785, 0.0};
const double jmax[] = {+1.57, 0.785, 0.0, +1.57, +0.785, +0.785, 0.0};
const double omega[] = {0.145, 0.11, 0.13, 0.3, 0.7, 0.4, 0.55};
const double TRANSITION_DURATION = 5.0;  // seconds
//const double tlim[] = {25, 20, 15, 15, 5, 5, 5};
const double tlim[] = {20, 15, 10, 10, 5, 5, 3};
enum {RAMP_STOPPED, RAMP_PAUSE, RAMP_RESUME, RAMP_RUN};
uint8_t rampState = RAMP_STOPPED;
double scout[7];

using namespace barrett;
using detail::waitForEnter;

template <size_t DOF>
class JpCircle : public systems::SingleIO<double, typename units::JointPositions<DOF>::type> {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

public:
	systems::System::Input<jt_type> sc;
	systems::System::Input<jt_type> grav;
	JpCircle(jp_type startPos, systems::Ramp *r, systems::Wam<DOF> *wam, const std::string& sysName = "JpCircle") :
		systems::SingleIO<double, jp_type>(sysName), sc(this), grav(this), jp(startPos), ramp(r), wam(wam) {
			//overlimit = false;
			//releaseTime = time(NULL);
			

		}
	
	virtual ~JpCircle() { this->mandatoryCleanUp(); }

protected:
	jp_type jp;
	//jt_type sum_jt, grav_jt;
	jt_type sum_jt;
	//jp_type jp_0;
	//double amp, omegax;
	double theta[7];
	//int i1, i2;
	
	systems::Ramp *ramp;
	
	systems::Wam<DOF> *wam;

	virtual void operate() {
        // Copy the simplecontroller's output to a global var for access by the ramp thread
		sum_jt = sc.getValue();
		for(uint8_t i=0; i < 7; i++){
			scout[i] = sum_jt[i];
		}

#if 0
		//sum_jt = sum.getValue();
		grav_jt = grav.getValue();
		sum_jt = sc.getValue();
		//grav_jt = wam->gravity.

		// Check for release
		if(!ramp->isRunning() && time(NULL) > releaseTime){
			rampState = RAMP_RESUME;
			
		}
		// Get torques due to (only) PID controllers

		// Compare against joint torque limits
		overlimit = false;
		for(uint8_t i = 0; i < DOF; i++){
			//std::cout << "j" << i+1 << ": " << sum_jt[i] << " " << grav_jt[i] << std::endl;
			if( fabs(sum_jt[i] /* - grav_jt[i] */) >= tlim[i]){
				overlimit = true;
				break;
			}
		}
		// Pause
		if(overlimit && ramp->isRunning()){
			rampState = RAMP_PAUSE;
			releaseTime = time(NULL) + 5; // 5 seconds in the future
		}
#endif
		// Update joint position commands

		theta[0] = omega[0] * this->input.getValue(); // getValue() returns ramp.time
		theta[1] = omega[1] * this->input.getValue();
		theta[2] = omega[2] * this->input.getValue();
		theta[3] = omega[3] * this->input.getValue();
		theta[4] = omega[4] * this->input.getValue();
		theta[5] = omega[5] * this->input.getValue();
		theta[6] = omega[6] * this->input.getValue();

		jp[0] = (jmax[0] - jmin[0]) * (1.0 - (std::cos(theta[0]) + 1.0) / 2.0)  + jmin[0];
		jp[1] = (jmax[1] - jmin[1]) * (1.0 - (std::cos(theta[1]) + 1.0) / 2.0)  + jmin[1];
		jp[2] = (jmax[2] - jmin[2]) * (1.0 - (std::cos(theta[2]) + 1.0) / 2.0)  + jmin[2];
		jp[3] = (jmax[3] - jmin[3]) * (1.0 - (std::cos(theta[3]) + 1.0) / 2.0)  + jmin[3];
		jp[4] = (jmax[4] - jmin[4]) * (1.0 - (std::cos(theta[4]) + 1.0) / 2.0)  + jmin[4];
		jp[5] = (jmax[5] - jmin[5]) * (1.0 - (std::cos(theta[5]) + 1.0) / 2.0)  + jmin[5];
		jp[6] = (jmax[6] - jmin[6]) * (1.0 - (std::cos(theta[6]) + 1.0) / 2.0)  + jmin[6];

		this->outputValue->setData(&jp);

	}

private:
	DISALLOW_COPY_AND_ASSIGN(JpCircle);
};

//template <size_t DOF>
void handleRamp(systems::Ramp *ramp, void *wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(7);

	bool overlimit;
	time_t releaseTime = time(NULL);

	while ( !boost::this_thread::interruption_requested() ) {

        //sum_jt = sum.getValue();
		//grav_jt = grav.getValue();
		//sum_jt = sc.getValue();
        //jt_type sc_jt;
        //sc_jt = ((systems::Wam<7>*)wam)->jtSum.getInput()[2];
		//grav_jt = wam->gravity.

		// Check for release
		if(!ramp->isRunning() && time(NULL) > releaseTime){
			ramp->smoothStart(TRANSITION_DURATION);
			std::cout << "Resuming..." << std::endl;
		}

		// Compare against joint torque limits
		overlimit = false;
		for(uint8_t i = 0; i < 7; i++){
			//std::cout << "j" << i+1 << ": " << sum_jt[i] << " " << grav_jt[i] << std::endl;
			if( fabs(scout[i] /* - grav_jt[i] */) >= tlim[i]){
				overlimit = true;
				break;
			}
		}
		// Pause
		if(overlimit && ramp->isRunning()){
			ramp->stop();
			releaseTime = time(NULL) + 5; // 5 seconds in the future
			for(uint8_t i = 0; i < 7; i++)
				std::cout << "j" << i+1 << ": " << scout[i] << std::endl;
			std::cout << "Paused..." << std::endl;
		}

		usleep(1000);
	}
}

void updateHand(Hand *hand, systems::Ramp *ramp) {
	//RatePrinter rp(2,20);
	typedef Hand::jp_type hjp_t;
	double O = 0.0;
	double C = 2.4;
	double SC = M_PI;

	hjp_t open(O);
	hjp_t closed(C);
	closed[3] = SC;

//	int puckTemp[Hand::DOF];
//	int motorTemp[Hand::DOF];

	while ( !boost::this_thread::interruption_requested() ) {
		//hand->update();
		

		while(!ramp->isRunning()) usleep(10000);
		hand->trapezoidalMove(closed, Hand::SPREAD);
		while(!ramp->isRunning()) usleep(10000);
		hand->trapezoidalMove(closed, Hand::GRASP);
		while(!ramp->isRunning()) usleep(10000);
		hand->trapezoidalMove(open, Hand::GRASP);
		while(!ramp->isRunning()) usleep(10000);
		hand->trapezoidalMove(open, Hand::SPREAD);
		while(!ramp->isRunning()) usleep(10000);
		hand->trapezoidalMove(closed, Hand::GRASP);
		while(!ramp->isRunning()) usleep(10000);
		hand->trapezoidalMove(open, Hand::GRASP);
//		hand.getPuckGroup().getProperty(Puck::TEMP, puckTemp);
//		hand.getPuckGroup().getProperty(Puck::THERM, motorTemp);

		//rp.update();
		//usleep(10000); // 100 Hz
	}
	
}

template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	wam.gravityCompensate();


	//const double TRANSITION_DURATION = 0.5;  // seconds

	//const double JP_AMPLITUDE = 0.4;  // radians
	//const double CP_AMPLITUDE = 0.1;  // meters
	//const double FREQUENCY = 1.0;  // rad/s

	//Rate Limiter
	jp_type rt_jp_cmd;
	systems::RateLimiter<jp_type> jp_rl;
	//Sets the joints to move at 1 m/s
	const double rLimit[] = {1, 1, 1, 1, 1, 1, 1};
	

	// Set start position, depending on robot type and configuration.
	jp_type startPos(0.0);

	for(size_t i = 0; i < DOF; ++i){
		rt_jp_cmd[i] = rLimit[i];
		startPos[i] = jmin[i];
	}


	
	systems::Ramp ramp(pm.getExecutionManager(), 1.0);

	

	printf("Press [Enter] to move the robot randomly using joint position control.");
	waitForEnter();

	wam.moveTo(startPos);
	Hand& hand = *pm.getHand();
	//hand.getPucks()[3]->setProperty(Puck::KP, 25);
	//hand.getPucks()[3]->setProperty(Puck::KD, 0);
	//hand.getPucks()[3]->setProperty(Puck::KI, 1);
	//hand.getPucks()[3]->setProperty(Puck::MT, 4000);
	hand.getPucks()[3]->setProperty(Puck::ACCEL, 50); // default = 200
	
	usleep(1000); hand.getPucks()[0]->setProperty(Puck::KP, 500); // default = 500
	usleep(1000); hand.getPucks()[0]->setProperty(Puck::KD, 2500); // default = 2500
	usleep(1000); hand.getPucks()[0]->setProperty(Puck::KI, 0); // default = 0
	usleep(1000); hand.getPucks()[0]->setProperty(Puck::ACCEL, 100); // default = 200

	usleep(1000); hand.getPucks()[1]->setProperty(Puck::KP, 500); // default = 500
	usleep(1000); hand.getPucks()[1]->setProperty(Puck::KD, 2500); // default = 2500
	usleep(1000); hand.getPucks()[1]->setProperty(Puck::KI, 0); // default = 0
	usleep(1000); hand.getPucks()[1]->setProperty(Puck::ACCEL, 100); // default = 200

	usleep(1000); hand.getPucks()[2]->setProperty(Puck::KP, 500); // default = 500
	usleep(1000); hand.getPucks()[2]->setProperty(Puck::KD, 2500); // default = 2500
	usleep(1000); hand.getPucks()[2]->setProperty(Puck::KI, 0); // default = 0
	usleep(1000); hand.getPucks()[2]->setProperty(Puck::ACCEL, 100); // default = 200

	hand.initialize();
	

	std::vector<boost::thread*> threads;
	threads.push_back(new boost::thread(updateHand, &hand, &ramp));
	threads.push_back(new boost::thread(handleRamp, &ramp, (void*)&wam));

	//Indicate the current position and the maximum rate limit to the rate limiter
	jp_rl.setCurVal(wam.getJointPositions());
	jp_rl.setLimit(rt_jp_cmd);

	JpCircle<DOF> jpc(startPos, &ramp, &wam);
	systems::connect(wam.supervisoryController.output, jpc.sc); // Pass the jointspace PID controller output
	systems::connect(wam.gravity.output, jpc.grav); // Pass the jointspace PID controller output

	systems::connect(ramp.output, jpc.input);
	//Enforces that the individual joints move less than or equal to the above mentioned rate limit
	systems::connect(jpc.output, jp_rl.input);
	wam.trackReferenceSignal(jp_rl.output);
	ramp.smoothStart(TRANSITION_DURATION);

	//printf("Press [Enter] to print.");
	//waitForEnter();
	//BARRETT_SCOPED_LOCK(getMutex());
#if 0
	//++ut;
	//typedef boost::intrusive::list<systems::System, boost::intrusive::member_hook<systems::System, systems::System::managed_hook_type, &systems::System::managedHook> > managed_system_list_type;
	//managed_system_list_type::iterator i(pm.getExecutionManager()->managedSystems.begin()), iEnd(pm.getExecutionManager()->managedSystems.end());
	//for (; i != iEnd; ++i) {
	//	std::cout << i->getName() << std::endl;
	//}
	std::cout << jpc.sum.isConnected() << " " << jpc.sum.valueDefined() << " " << jpc.sum.getValue() << std::endl;
	std::cout << jpc.grav.isConnected() << " " << jpc.grav.valueDefined() << " " << jpc.grav.getValue() << std::endl;

	printf("Press [Enter] to print.");
	waitForEnter();
	std::cout << jpc.sum.isConnected() << " " << jpc.sum.valueDefined() << " " << jpc.sum.getValue() << std::endl;
	std::cout << jpc.grav.isConnected() << " " << jpc.grav.valueDefined() << " " << jpc.grav.getValue() << std::endl;

	printf("Press [Enter] to print.");
	waitForEnter();
	std::cout << jpc.sum.isConnected() << " " << jpc.sum.valueDefined() << " " << jpc.sum.getValue() << std::endl;
	std::cout << jpc.grav.isConnected() << " " << jpc.grav.valueDefined() << " " << jpc.grav.getValue() << std::endl;
#endif

	printf("Press [Enter] to stop.\n");
	waitForEnter();
	ramp.smoothStop(TRANSITION_DURATION);
	
	// Stop the BHand thread
	for (size_t i = 0; i < threads.size(); ++i) {
		threads[i]->interrupt();
		threads[i]->join();
	}

	printf("Press [Enter] to return home.\n");
	waitForEnter();

	typedef Hand::jp_type hjp_t;
	double O = 0.0;
	double C = 2.4;
	double SC = M_PI;

	hjp_t open(O);
	hjp_t closed(C);
	closed[3] = SC;
	hand.trapezoidalMove(open, Hand::GRASP);
	hand.trapezoidalMove(closed, Hand::SPREAD);
	hand.trapezoidalMove(closed, Hand::GRASP);
	
	//wam.idle();

	wam.moveHome();
	wam.idle();

	// Wait for the user to press Shift-idle
	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
	return 0;
}
