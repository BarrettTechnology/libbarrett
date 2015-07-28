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

#include "simon.h"

//Constants describing out environment in meters
const double BACKWALLX = 0.3;   const double FRONTWALLX = 0.7;
const double LEFTWALLY = -0.25; const double RIGHTWALLY = 0.25;
const double TOPWALLZ = 0.5;    const double BOTTOMWALLZ = 0.0;
const double CENTERX = FRONTWALLX - ((FRONTWALLX - BACKWALLX) / 2);
const double CENTERY = RIGHTWALLY - ((RIGHTWALLY - LEFTWALLY) / 2);
const double CENTERZ = TOPWALLZ - ((TOPWALLZ - BOTTOMWALLZ) / 2);
const double HEIGHT = TOPWALLZ - BOTTOMWALLZ;
const double WIDTH = LEFTWALLY - RIGHTWALLY;
const double DEPTH = FRONTWALLX - BACKWALLX;

//Button
const double BUTTONRAD = .15;
const double BUTTONDEPTH = 0.01;
const double LEFTBUTTON = LEFTWALLY + BUTTONDEPTH;
const double RIGHTBUTTON = RIGHTWALLY - BUTTONDEPTH;
const double TOPBUTTON = TOPWALLZ - BUTTONDEPTH;
const double FLOORBUTTON = BOTTOMWALLZ + BUTTONDEPTH;
const double BACKBUTTON = BACKWALLX + BUTTONDEPTH;
const double FRONTBUTTON = FRONTWALLX - BUTTONDEPTH;

//Proxy Parameters
const double PROXYRAD = 0.03;

//Magnetic field
const double MAGFIELDSIZE = 0.01;

//Ball Parameters
const double GRAVITY_FORCE = 0.00015;
const double AIR_DAMPING = 0.0005;

using namespace barrett;
BARRETT_UNITS_FIXED_SIZE_TYPEDEFS
;
BARRETT_UNITS_TYPEDEFS(8);

/* Global Variables Technically should all be scoped/encapsulated in Simon Class */
char* remoteHost = NULL;
cf_type currentForce;
cp_type proxyPosition;
cv_type proxyVelocity;
cp_type wamPosition;
v_type msg_tmp;
int buttonPressed;
int wallSelected;
int lastWall;
int level;
int buttonTouches;
bool b1p,b2p,b3p,b4p,b5p,b6p;
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

// Wall Generator Function
int wallGenerator(int _last){
    
    int wall;
    // Generate New Wall Selection 1-6
    wall = 1 + (rand() % 6);
    // verify not the same number as last time
    if(wall == _last){wall = wallGenerator(_last);}
    
    return wall;
}

// Haptic Calculation Function
cf_type hapticCalc(boost::tuple<cp_type, cv_type> haptuple) {
	cf_type cfSum;
	cp_type wamPosition = haptuple.get<0>();
	cv_type wamVelocity = haptuple.get<1>();

	proxyPosition = wamPosition;
	proxyVelocity = wamVelocity;

	double stiffness = 1200.0; /* Kp Spring Force */
	double damping = 5.0; /* Kd Damping Force */

  // Stiffness will occur as WAMPOS and ProxyPOS differ (aka hit wall or button)
	cfSum[0] = stiffness * (proxyPosition[0] - wamPosition[0]);
	cfSum[1] = stiffness * (proxyPosition[1] - wamPosition[1]);
	cfSum[2] = stiffness * (proxyPosition[2] - wamPosition[2]);

   /** START: Left Wall Collisions   Wall/Button #1 */
	if (wamPosition[1] < LEFTWALLY + BUTTONDEPTH + PROXYRAD
			&& wamPosition[0] < CENTERX + BUTTONRAD
			&& wamPosition[0] > CENTERX - BUTTONRAD
			&& wamPosition[2] < CENTERZ + BUTTONRAD
			&& wamPosition[2] > CENTERZ - BUTTONRAD) {		
    proxyPosition[1] = LEFTWALLY + BUTTONDEPTH + PROXYRAD;
		if (wamPosition[1] - proxyPosition[1] < -0.01) {
			proxyPosition[1] = LEFTWALLY + PROXYRAD;
			cfSum[1] += (stiffness * (proxyPosition[1] - wamPosition[1])); 
			cfSum[1] -= damping * wamVelocity[1]; 
			b1p = true; lastWall = 1;
		} else {
			cfSum[1] += (stiffness * (proxyPosition[1] - wamPosition[1])); 
			cfSum[1] -= damping * wamVelocity[1]; 
      b1p = false;
		}
	} else if (wamPosition[1] < LEFTWALLY + PROXYRAD) {
		proxyPosition[1] = LEFTWALLY + PROXYRAD;
		cfSum[1] += (stiffness * (proxyPosition[1] - wamPosition[1])); 
		cfSum[1] -= damping * wamVelocity[1]; 
    b1p = false;
	} /** END: Left Wall Collisions */

	/** START: Right Wall Collisions  Wall/Button #2 */
  if(wamPosition[1] > RIGHTWALLY - BUTTONDEPTH - PROXYRAD
			&& wamPosition[0] < CENTERX + BUTTONRAD
			&& wamPosition[0] > CENTERX - BUTTONRAD
			&& wamPosition[2] < CENTERZ + BUTTONRAD
			&& wamPosition[2] > CENTERZ - BUTTONRAD) {
		proxyPosition[1] = RIGHTWALLY - BUTTONDEPTH - PROXYRAD;
		if (wamPosition[1] - proxyPosition[1] > 0.01) {
			proxyPosition[1] = RIGHTWALLY - PROXYRAD;
			cfSum[1] += (stiffness * (proxyPosition[1] - wamPosition[1])); 
			cfSum[1] -= damping * wamVelocity[1]; 
      b2p = true;lastWall = 2;
		} else {
			cfSum[1] += (stiffness * (proxyPosition[1] - wamPosition[1])); 
			cfSum[1] -= damping * wamVelocity[1]; 
      b2p = false;
		}
  }else if (wamPosition[1] > RIGHTWALLY - PROXYRAD) {
		proxyPosition[1] = RIGHTWALLY - PROXYRAD;
		cfSum[1] += (stiffness * (proxyPosition[1] - wamPosition[1])); 
		cfSum[1] -= damping * wamVelocity[1];
    b2p = false;
	} // END: Right Wall Collisions

	// START: Ceiling Collisions. Wall/Button #3
  if (wamPosition[2] > TOPWALLZ - BUTTONDEPTH - PROXYRAD
      && wamPosition[0] < CENTERX + BUTTONRAD
			&& wamPosition[0] > CENTERX - BUTTONRAD
			&& wamPosition[1] < CENTERY + BUTTONRAD
			&& wamPosition[1] > CENTERY - BUTTONRAD) {
    proxyPosition[2] = TOPWALLZ - BUTTONDEPTH - PROXYRAD;
    if(wamPosition[2] - proxyPosition[2] > 0.01){ // lock proxy position
      proxyPosition[2] = TOPWALLZ - PROXYRAD;
      cfSum[2] += (stiffness * (proxyPosition[2] - wamPosition[2]));
      cfSum[2] -= damping * wamVelocity[2];
      b3p = true;lastWall = 3;
    } else{ // not locked proxy position
      cfSum[2] += (stiffness * (proxyPosition[2] - wamPosition[2]));
      cfSum[2] -= damping * wamVelocity[2];          
      b3p = false;
    }
  } else if (wamPosition[2] > TOPWALLZ - PROXYRAD) {
		proxyPosition[2] = TOPWALLZ - PROXYRAD;
		cfSum[2] += (stiffness * (proxyPosition[2] - wamPosition[2])); 
		cfSum[2] -= damping * wamVelocity[2];
    b3p = false; 
	} // END: Ceiling Collisions.

	// START: Floor Collisions. Button/Wall #4
  if (wamPosition[2] < BOTTOMWALLZ + BUTTONDEPTH + PROXYRAD
      && wamPosition[0] < CENTERX + BUTTONRAD
			&& wamPosition[0] > CENTERX - BUTTONRAD
			&& wamPosition[1] < CENTERY + BUTTONRAD
			&& wamPosition[1] > CENTERY - BUTTONRAD) {
    proxyPosition[2] = BOTTOMWALLZ + BUTTONDEPTH + PROXYRAD;
    if(wamPosition[2] - proxyPosition[2] < -0.01){ // lock proxy position
      proxyPosition[2] = BOTTOMWALLZ + PROXYRAD;
      cfSum[2] += (stiffness * (proxyPosition[2] - wamPosition[2]));
      cfSum[2] -= damping * wamVelocity[2];
      b4p = true; lastWall = 4;
    } else{ // not locked proxy position
      cfSum[2] += (stiffness * (proxyPosition[2] - wamPosition[2]));
      cfSum[2] -= damping * wamVelocity[2];
      b4p = false;          
    }
  } else if (wamPosition[2] < BOTTOMWALLZ + PROXYRAD) { // not in
		proxyPosition[2] = BOTTOMWALLZ + PROXYRAD;
		cfSum[2] += (stiffness * (proxyPosition[2] - wamPosition[2]));
		cfSum[2] -= damping * wamVelocity[2];
    b4p = false;
	} // END: Floor Collisions.


	// START: Back Wall Collisions Wall/Button #5
  if(wamPosition[0] < BACKWALLX + BUTTONDEPTH + PROXYRAD
      && wamPosition[1] < CENTERY + BUTTONRAD
      && wamPosition[1] > CENTERY - BUTTONRAD
      && wamPosition[2] < CENTERZ + BUTTONRAD
			&& wamPosition[2] > CENTERZ - BUTTONRAD) {
      proxyPosition[0] = BACKWALLX + BUTTONDEPTH + PROXYRAD;
		if (wamPosition[0] - proxyPosition[0] < -0.01) {
			proxyPosition[0] = BACKWALLX + PROXYRAD;
			cfSum[0] += (stiffness * (proxyPosition[0] - wamPosition[0])); 
			cfSum[0] -= damping * wamVelocity[0]; 
			b5p = true;
		} else {
			cfSum[0] += (stiffness * (proxyPosition[0] - wamPosition[0])); 
			cfSum[0] -= damping * wamVelocity[0]; 
			b5p = false;lastWall = 5;
		}
  }else if (wamPosition[0] < BACKWALLX + PROXYRAD) {
		proxyPosition[0] = BACKWALLX + PROXYRAD;
		cfSum[0] += (stiffness * (proxyPosition[0] - wamPosition[0])); 
		cfSum[0] -= damping * wamVelocity[0]; 
    b5p = false;
	} // END: Back Collisions

	// START: Front Collisions  Wall/Button #6
  if(wamPosition[0] > FRONTWALLX - BUTTONDEPTH - PROXYRAD
      && wamPosition[1] < CENTERY + BUTTONRAD
      && wamPosition[1] > CENTERY - BUTTONRAD
			&& wamPosition[2] < CENTERZ + BUTTONRAD
			&& wamPosition[2] > CENTERZ - BUTTONRAD) {
      proxyPosition[0] = FRONTWALLX - BUTTONDEPTH - PROXYRAD;
		if (wamPosition[0] - proxyPosition[0] > 0.01) {
			proxyPosition[0] = FRONTWALLX - PROXYRAD;
			cfSum[0] += (stiffness * (proxyPosition[0] - wamPosition[0])); 
			cfSum[0] -= damping * wamVelocity[0]; 
			b6p = true; lastWall = 6;
		} else {
			cfSum[0] += (stiffness * (proxyPosition[0] - wamPosition[0])); 
			cfSum[0] -= damping * wamVelocity[0]; 
      b6p = false;
		}
  }else if (wamPosition[0] > FRONTWALLX - PROXYRAD) {
		proxyPosition[0] = FRONTWALLX - PROXYRAD;
		cfSum[0] += (stiffness * (proxyPosition[0] - wamPosition[0])); 
		cfSum[0] -= damping * wamVelocity[0]; 
    b6p = false;
	} // END: Front Collisions

  /**
   * Add Damping as level increases. 1/2 Damping factor above.
   */
  cfSum -= wamVelocity * (damping / 2) * (double)level;

	/* Sum button Statuses to give one int value */
	buttonPressed = (int)b1p * 1 + (int)b2p * 2 + (int)b3p * 4 + 
		(int)b4p * 8 + (int)b5p * 16 + (int)b6p * 32;

  /* Wall Generator Update */
  if(wallSelected == lastWall){
    wallSelected = wallGenerator(lastWall);
    buttonTouches++;
  }

  /* Every 10 successul Button Touches up level of difficulty. */
  if(buttonTouches > 10){
    level++;
    buttonTouches = 0;
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
  bool b1p, b2p, b3p, b4p, b5p, b6p;
//  int selectedWall, lastWall, buttonTouches, level;
	jt_type cutoff;
	systems::ToolForceToJointTorques<DOF> cf_tf2jt;
	systems::FirstOrderFilter<jt_type> cfLPF;
	systems::ExposedOutput<cf_type> cfCmd;
	systems::ExposedOutput<cp_type> proxyPos;
	systems::ExposedOutput<cv_type> wamVel;
	double massOffset;
	cp_type boxCenter;
	math::Vector<3>::type boxSize;
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
//			wam(wam_), pm(pm_), cutoff(100.0), boxCenter(0.35, 0.4, 0.0),
//			boxSize(0.3, 0.3,0.3), box(boxCenter, boxSize),
			wam(wam_), pm(pm_), cutoff(100.0), boxCenter(CENTERX, CENTERY, CENTERZ),
			boxSize(DEPTH, WIDTH,HEIGHT), box(boxCenter, boxSize),
			zero(0.0), mult(scale), jtLimits(45.0), jtSat(boost::bind(saturateJt<DOF>, _1, jtLimits)),
			insideBox(true), slen(sizeof(si_server)), haptics(hapticCalc), sumForces(0.0) {
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

	jpStart << 0.0, 0.0, 1.57;

	for (int i = 0; i < 3; i++) {
		proxyPosition[i] = 0.0;
		proxyVelocity[i] = 0.0;
	}

	buttonPressed = 0;
	wallSelected = 0;
  lastWall = 0;
  buttonTouches = 0;
  level = 0;
  b1p = b2p = b3p = b4p = b5p = b6p = false;
	
	for (int j = 0; j < 8; j++) {
		if (j < 3)
			msg_tmp[j] = proxyPosition[j];
		else if (j < 6)
			msg_tmp[j] = proxyVelocity[j - 3];
		else if (j < 7)
			msg_tmp[j] = (double) wallSelected;
		else
			msg_tmp[j] = (double) buttonPressed;
	}

	 cfLPF.setLowPass(cutoff);
	
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
	if ((sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
		printf("Failure creating the socket");
		return false;
	}
	memcpy(srv_addr, remoteHost, strlen(remoteHost));
	memset((char *) &si_server, 0, sizeof(si_server));
	si_server.sin_family = AF_INET;
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

/**
 * Print Command Options to screen
 */
template<size_t DOF>
void HapticsDemo<DOF>::displayEntryPoint() {
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
      printf("Button Status: %d, Wall Status: %d\n", buttonPressed, wallSelected);      
      printf("Current level: %d, Wall Touches: %d\n>> ",level, buttonTouches);
			break;
		}
	}
}

/**
 * Package Data to be sent to Python Visualization
 */
template<size_t DOF>
void HapticsDemo<DOF>::visualizationThread() {
	while (curState == PLAYING) {
		sumForces = 0.0;
		for (int i = 0; i < 3; i++)
			sumForces += fabs(curForces[i]);
	
    for (int j = 0; j < 8; j++) {
      if (j < 3)
	      msg_tmp[j] = proxyPosition[j];
      else if (j < 6)
	      msg_tmp[j] = proxyVelocity[j - 3];
      else if (j < 7)
	      msg_tmp[j] = (double) wallSelected;
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
