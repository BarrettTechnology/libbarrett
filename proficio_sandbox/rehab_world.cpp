//Networking
#include <string.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>

// Barrett Library
#include <barrett/math.h>
#include <barrett/exception.h>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>
#define BARRETT_SMF_VALIDATE_ARGS
#include <barrett/standard_main_function.h>

#include "haptic_world.h"

//Constants describing out environment in meters
const double BACKWALLX = 0.3;
const double FRONTWALLX = 0.7;
const double LEFTWALLY = -0.25;
const double RIGHTWALLY = 0.25;
const double TOPWALLZ = 0.5;
const double BOTTOMWALLZ = 0.0;

//Button
const double BUTTONRAD = .15;
const double BUTTONSIZE = 0.01;

//Proxy Parameters
const double PROXYRAD = 0.03;

//Magnetic field
const double MAGFIELDSIZE = 0.01;

//Ball Parameters
const double BALLSTARTX = 0.45; //0.25;
const double BALLSTARTY = -0.125; //-0.2;
const double BALLSTARTZ = 0.25;
const double BALLR = 0.075;
const double BALLM = 50.0;
const double GRAVITY_FORCE = 0.00015;
const double AIR_DAMPING = 0.0005;

using namespace barrett;
BARRETT_UNITS_FIXED_SIZE_TYPEDEFS
;
BARRETT_UNITS_TYPEDEFS(10);

char* remoteHost = NULL;
cf_type currentForce;
cp_type proxyPosition;
cv_type proxyVelocity;
cp_type ballPosition;
cv_type ballVelocity;
v_type msg_tmp;
bool buttonPressed;
systems::ExposedOutput<v_type> message;

bool validate_args(int argc, char** argv) {
	if (argc != 2) {
		printf("Usage: %s <otherip>\n", argv[0]);
		return false;
	}
	remoteHost = argv[1];
	return true;
}

cf_type scale(boost::tuple<cf_type, double> t) {
	return t.get<0>() * t.get<1>();
}

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

cf_type hapticCalc(boost::tuple<cp_type, cv_type> haptuple) {
	cf_type cfSum;
	cp_type wamPosition = haptuple.get<0>();
	cv_type wamVelocity = haptuple.get<1>();

	proxyPosition = wamPosition;
	proxyVelocity = wamVelocity;

	double stiffness = 1000.0; // Spring Force
	double damping = 10.0; // Damping Force

	// Ball Calculations
	cp_type proxyBallVec = proxyPosition - ballPosition; // Should this be WAM Position??

	if (proxyBallVec.norm() < (BALLR + PROXYRAD)) {
		proxyPosition = ballPosition
				+ (BALLR + PROXYRAD) * proxyBallVec / proxyBallVec.norm();
		ballVelocity[0] = ballVelocity[0]
				+ (-(stiffness * (proxyPosition[0] - wamPosition[0])) / BALLM)
						* 0.002; // changes the balls velocity when touched by the proxy accordingly in all three dimensions
		ballVelocity[1] = ballVelocity[1]
				+ (-(stiffness * (proxyPosition[1] - wamPosition[1])) / BALLM)
						* 0.002;
		ballVelocity[2] = ballVelocity[2]
				+ (-(stiffness * (proxyPosition[2] - wamPosition[2])) / BALLM)
						* 0.002 - GRAVITY_FORCE;
		; //500Hz
	} else {
		ballVelocity[2] -= GRAVITY_FORCE; //gravity compensation on the ball
		ballVelocity -= AIR_DAMPING * ballVelocity; //damping due to air friction
	}

	cfSum[0] = stiffness * (proxyPosition[0] - wamPosition[0]);
	cfSum[1] = stiffness * (proxyPosition[1] - wamPosition[1]);
	cfSum[2] = stiffness * (proxyPosition[2] - wamPosition[2]);

	//Check for a collision with the ball and right wall using balls radius
	if (ballPosition[1] > RIGHTWALLY - BALLR) {
		ballVelocity[1] = -ballVelocity[1];
	}

	//Check for a collision with the ball and button / left wall using balls radius
	if (ballPosition[1] < LEFTWALLY + BUTTONSIZE + BALLR
			&& ballPosition[0]
					< BACKWALLX + ((FRONTWALLX - BACKWALLX) / 2) + BUTTONRAD
			&& ballPosition[0]
					> BACKWALLX + ((FRONTWALLX - BACKWALLX) / 2) - BUTTONRAD
			&& ballPosition[2]
					< BOTTOMWALLZ + ((TOPWALLZ - BOTTOMWALLZ) / 2) + BUTTONRAD
			&& ballPosition[2]
					> BOTTOMWALLZ + ((TOPWALLZ - BOTTOMWALLZ) / 2) - BUTTONRAD) {
		ballVelocity[1] = -ballVelocity[1];
	} else if (ballPosition[1] < LEFTWALLY + BALLR)
		ballVelocity[1] = -ballVelocity[1];

	//Check for a collision with the ball and ceiling using balls radius
	if (ballPosition[2] > TOPWALLZ - BALLR) {
		ballVelocity[2] = -ballVelocity[2];
	}

	//Check for a collision with the ball and floor using balls radius
	if (ballPosition[2] < BOTTOMWALLZ + BALLR) {
		ballVelocity[2] = -ballVelocity[2];
	}

	//Check for a collision with the ball and front wall using balls radius
	if (ballPosition[0] > FRONTWALLX - BALLR) {
		ballVelocity[0] = -ballVelocity[0];
	}

	//Check for a collision with the ball and back wall using balls radius
	if (ballPosition[0] < BACKWALLX + BALLR) {
		ballVelocity[0] = -ballVelocity[0];
	}

	ballPosition = ballPosition + 0.001 * ballVelocity;

	// Check for a collision with the back wall - Sticky
	if (wamPosition[0] < BACKWALLX + PROXYRAD) {
		proxyPosition[0] = BACKWALLX + PROXYRAD;
		cfSum[0] += (stiffness * (proxyPosition[0] - wamPosition[0])); //Spring Portion
		cfSum[0] -= damping * wamVelocity[0]; // Damping portion
		cfSum[1] -= 1 * damping * wamVelocity[1];
		cfSum[2] -= 1 * damping * wamVelocity[2];
	}

	// Check for a collision with the front wall - Nothing Keep person in box.
	if (wamPosition[0] > FRONTWALLX - PROXYRAD) {
		proxyPosition[0] = FRONTWALLX - PROXYRAD;
		cfSum[0] += (stiffness * (proxyPosition[0] - wamPosition[0])); //Spring Portion
		cfSum[0] -= damping * wamVelocity[0]; // Damping portion
	}

	// Check for a collision with the right wall - Magnetic
	if (wamPosition[1] > RIGHTWALLY - PROXYRAD - MAGFIELDSIZE) {
		proxyPosition[1] = RIGHTWALLY - PROXYRAD;
		cfSum[1] += (stiffness * (proxyPosition[1] - wamPosition[1])); //Spring Portion
		cfSum[1] -= damping * wamVelocity[1];
	}

	buttonPressed = false;
	// Check for a collision with the left button / WALL
	if (wamPosition[1] < LEFTWALLY + BUTTONSIZE + PROXYRAD
			&& wamPosition[0]
					< BACKWALLX + ((FRONTWALLX - BACKWALLX) / 2) + BUTTONRAD
			&& wamPosition[0]
					> BACKWALLX + ((FRONTWALLX - BACKWALLX) / 2) - BUTTONRAD
			&& wamPosition[2]
					< BOTTOMWALLZ + ((TOPWALLZ - BOTTOMWALLZ) / 2) + BUTTONRAD
			&& wamPosition[2]
					> BOTTOMWALLZ + ((TOPWALLZ - BOTTOMWALLZ) / 2) - BUTTONRAD) {
		proxyPosition[1] = LEFTWALLY + BUTTONSIZE + PROXYRAD;
		if (wamPosition[1] - proxyPosition[1] < -0.01) {
			proxyPosition[1] = LEFTWALLY + PROXYRAD;
			cfSum[1] += (stiffness * (proxyPosition[1] - wamPosition[1])); //Spring Portion
			cfSum[1] -= damping * wamVelocity[1]; // Damping portion

			buttonPressed = true;
			for (int i = 0; i < 3; i++)
				ballVelocity[0] = 0.0;
			ballPosition[0] = BALLSTARTX;
			ballPosition[1] = BALLSTARTY;
			ballPosition[2] = BALLSTARTZ;
		} else {
			cfSum[1] += (stiffness * (proxyPosition[1] - wamPosition[1])); //Spring Portion
			cfSum[1] -= damping * wamVelocity[1]; // Damping portion
		}
	} else if (wamPosition[1] < LEFTWALLY + PROXYRAD) {
		proxyPosition[1] = LEFTWALLY + PROXYRAD;
		cfSum[1] += (stiffness * (proxyPosition[1] - wamPosition[1])); //Spring Portion
		cfSum[1] -= damping * wamVelocity[1]; // Damping portion
	}

	// Check for a collision with the ceiling - Slippery inverse damping
	if (wamPosition[2] > TOPWALLZ - PROXYRAD) {
		proxyPosition[2] = TOPWALLZ - PROXYRAD;
		cfSum[2] += (0.5 * stiffness * (proxyPosition[2] - wamPosition[2])); //Spring Portion
		cfSum[2] -= damping * wamVelocity[2]; // Damping portion
		cfSum[0] += 0.2 * damping * wamVelocity[0];
		cfSum[1] += 0.2 * damping * wamVelocity[1];
	}

	// Check for a collision with the floor.
	if (wamPosition[2] < BOTTOMWALLZ + PROXYRAD) {
		proxyPosition[2] = BOTTOMWALLZ + PROXYRAD;
		cfSum[2] += (0.5 * stiffness * (proxyPosition[2] - wamPosition[2]));
		cfSum[2] -= damping * wamVelocity[2];
		if (sin(500 * wamPosition[1]) >= 0)
			cfSum[1] = 0.75 * stiffness * (proxyPosition[1] - wamPosition[1])
					- damping * wamVelocity[1] * 1.5; // creates the illusion of stripes on the wall by multiplying the position by sin(.5*position), and turns on the parallel damping when it is above 0 and turns it off when it is less than zero
	}
	return cfSum;
}

enum CONTACT_STATE {
	PLAYING, TARING, QUIT
} curState = PLAYING, lastState = PLAYING;

//HapticsDemo Class
template<size_t DOF>
class HapticsDemo {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
protected:
	systems::Wam<DOF>& wam;
	ProductManager& pm;
	libconfig::Config conf;

	cf_type cCurrent, cOffset, cfAdjusted, cfTransformed, cfSum;
	jp_type jpzero, jpi1, jpi2, jpi3, jpi4, jpi5, jpiY, jpiY2, jpStart;
	jt_type cutoff;
	systems::ToolForceToJointTorques<DOF> cf_tf2jt;
	systems::FirstOrderFilter<jt_type> cfLPF;
	systems::ExposedOutput<cf_type> cfCmd;
	systems::ExposedOutput<cp_type> ballPos, proxyPos;
	systems::ExposedOutput<cv_type> wamVel;
	double massOffset;
	cp_type ballCenter, boxCenter;
	math::Vector<3>::type boxSize;
	systems::HapticBall ball;
	systems::HapticBox box;

	systems::Summer<cf_type> dirSum;
	systems::Summer<double> depthSum;
	systems::PIDController<double, double> comp;
	systems::Constant<double> zero;
	systems::TupleGrouper<cp_type, cv_type> tg;
	systems::Callback<boost::tuple<cf_type, double>, cf_type> mult;
	systems::ToolForceToJointTorques<DOF> tf2jt;
	jt_type jtLimits;
	systems::Callback<jt_type> jtSat;
	jt_type jtCur;
	cf_type curForces;
	bool insideBox;
	struct sockaddr_in si_server;
	int port, sock, i, slen;
	char* buf;
	char* srv_addr;

	//Haptic portion
	systems::Callback<boost::tuple<cp_type, cv_type>, cf_type> haptics;

public:
	bool ftOn;
	double sumForces;
	HapticsDemo(systems::Wam<DOF>& wam_, ProductManager& pm_) :
			wam(wam_), pm(pm_), cutoff(100.0), ballCenter(
					0.4, -.3, 0.0), boxCenter(0.35, 0.4, 0.0), boxSize(0.3, 0.3,
					0.3), ball(ballCenter, 0.2), box(boxCenter, boxSize), zero(
					0.0), mult(scale), jtLimits(45.0), jtSat(
					boost::bind(saturateJt<DOF>, _1, jtLimits)), insideBox(
					true), slen(sizeof(si_server)), haptics(hapticCalc), sumForces(
					0.0) {
	}
	bool
	init();

	~HapticsDemo() {
	}

	bool
	setupNetworking();
	void
	displayEntryPoint();
	void
	connectForces();
	void
	visualizationThread();
}
;

// Initialization
template<size_t DOF>
bool HapticsDemo<DOF>::init() {
	wam.gravityCompensate();
	pm.getSafetyModule()->setVelocityLimit(1.5);
	pm.getSafetyModule()->setTorqueLimit(2.5);

	jpStart[0] = 0.0;
	jpStart[1] = 0.0;
	jpStart[2] = 1.57;

	for (int i = 0; i < 3; i++) {
		proxyPosition[i] = 0.0;
		proxyVelocity[i] = 0.0;
		ballVelocity[i] = 0.0;
	}
	ballPosition[0] = BALLSTARTX;
	ballPosition[1] = BALLSTARTY;
	ballPosition[2] = BALLSTARTZ;

	for (int j = 0; j < 10; j++) {
		if (j < 3)
			msg_tmp[j] = proxyPosition[j];
		else if (j < 6)
			msg_tmp[j] = proxyVelocity[j - 3];
		else if (j < 9)
			msg_tmp[j] = ballPosition[j - 6];
		else
			msg_tmp[j] = (double) buttonPressed;
	}

	cfLPF.setLowPass(cutoff);
	buttonPressed = false;

    wam.moveTo(jpStart);
    wam.idle();
	return true;
}

template<size_t DOF>
bool HapticsDemo<DOF>::setupNetworking() {
	buf = new char[1024];
	srv_addr = new char[16];
	memset(srv_addr, 0, 16);
	port = 5555;
//	if ((sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
	if ((sock = socket(AF_UNIX, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
		printf("Failure creating the socket");
		return false;
	}
	memcpy(srv_addr, remoteHost, strlen(remoteHost));
	memset((char *) &si_server, 0, sizeof(si_server));
	//si_server.sin_family = AF_INET;
  si_server.sin_family = AF_UNIX;
	si_server.sin_port = htons(port);
	if (inet_aton(srv_addr, &si_server.sin_addr) == 0) {
		printf("inet_aton() failed - EXITING\n");
		return false;
	}
	return true;
}

template<size_t DOF>
void HapticsDemo<DOF>::connectForces() {
	BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;
	systems::connect(wam.kinematicsBase.kinOutput, cf_tf2jt.kinInput);
	systems::connect(wam.toolPosition.output, tg.getInput<0>());
	systems::connect(wam.toolVelocity.output, tg.getInput<1>());
	systems::connect(tg.output, haptics.input);
	systems::connect(haptics.output, cf_tf2jt.input);
	systems::connect(cf_tf2jt.output, jtSat.input);
	systems::connect(jtSat.output, wam.input);
}

template<size_t DOF>
void HapticsDemo<DOF>::displayEntryPoint() {
	// Instructions displayed to screen.
	printf("\n");
	printf("Enter 'q' at any time to quit.\n\n>> ");

	std::string line;
	while (true) {
		std::getline(std::cin, line);
		switch (line[0]) {
		case 'q':
			printf("\nQuitting - Moving the WAM Home");
			curState = QUIT;
			break;
		default:
			printf(">> ");
			break;
		}
	}
}

template<size_t DOF>
void HapticsDemo<DOF>::visualizationThread() {
	while (curState == PLAYING) {
		sumForces = 0.0;
		for (int i = 0; i < 3; i++)
			sumForces += fabs(curForces[i]);
		for (int j = 0; j < 10; j++) {
			if (j < 3)
				msg_tmp[j] = proxyPosition[j];
			else if (j < 6)
				msg_tmp[j] = proxyVelocity[j - 3];
			else if (j < 9)
				msg_tmp[j] = ballPosition[j - 6];
			else
				msg_tmp[j] = (double) buttonPressed;
		}
		message.setValue(msg_tmp);
		btsleep(0.01);
	}
}

template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm,
		systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	HapticsDemo<DOF> haptics_demo(wam, pm);

	if (!haptics_demo.setupNetworking()) {
		return 1;
	}

	if (!haptics_demo.init())
		return 1;

	// instantiate Systems
	NetworkHaptics nh(pm.getExecutionManager(), remoteHost);

	message.setValue(msg_tmp);
	systems::forceConnect(message.output, nh.input);

	//haptics_demo.start();
	haptics_demo.connectForces();

	boost::thread displayThread(&HapticsDemo<DOF>::displayEntryPoint,
			&haptics_demo);
	boost::thread visualizeThread(&HapticsDemo<DOF>::visualizationThread,
			&haptics_demo);

	bool running = true;

	haptics_demo.ftOn = false;

	while (running) {
		switch (curState) {
		case QUIT:
			systems::disconnect(wam.input);
			running = false;
			break;
		case PLAYING:
			lastState = PLAYING;
			btsleep(0.1);
			break;
		default:
			break;
		}
	}
	wam.moveHome();
	printf("\n\n");
	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
	return 0;
}
