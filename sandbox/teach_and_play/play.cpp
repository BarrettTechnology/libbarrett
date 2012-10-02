/*
 * play.cpp
 *
 *  Created on: Sept 26, 2012
 *      Author: Kyle Maroney
 */

#include <string>
#include <boost/tuple/tuple.hpp>
#include <boost/ref.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/tokenizer.hpp>
#include <libconfig.h++>
#include <Eigen/Core>

#include <barrett/exception.h>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>
#define BARRETT_SMF_VALIDATE_ARGS
#include <barrett/standard_main_function.h>

#include "control_mode_switcher.h"

using namespace barrett;
using detail::waitForEnter;

enum STATE {
	PLAYING, STOPPED, PAUSED, QUIT
} curState = STOPPED, lastState = STOPPED;

char* ctrlMode = NULL;
bool vcMode = false;

bool validate_args(int argc, char** argv) {
	switch (argc) {
	case 2:
		if (boost::filesystem::exists(argv[1])) {
			printf("\nTrajectory to be played in current control mode: %s\n\n",
					argv[1]);
			return true;
			break;
		} else {
			printf("\nTrajectory not found in location specified: %s\n\n",
					argv[1]);
			return false;
			break;
		}
	case 3:
		ctrlMode = argv[2];
		if (boost::filesystem::exists(argv[1])
				&& (strcmp(ctrlMode, "cc") == 0 || strcmp(ctrlMode, "-cc") == 0
						|| strcmp(ctrlMode, "vc") == 0
						|| strcmp(ctrlMode, "-vc") == 0)) {
			printf(
					"\nTrajectory to be played in %s mode: %s\n\n",
					strcmp(ctrlMode, "vc") == 0
							|| strcmp(ctrlMode, "-vc") == 0 ?
							"voltage control" : "current control", argv[1]);
			if (strcmp(ctrlMode, "vc") == 0 || strcmp(ctrlMode, "-vc") == 0)
				vcMode = true;
			return true;
			break;
		} else {
			printf("\nTrajectory not found in location specified: %s\n\n",
					argv[1]);
			return false;
			break;
		}

	default:
		printf("Usage: %s <path/to/trajectory> [<Control Mode (cc or vc)>]\n",
				argv[0]);
		return false;
		break;
	}
}

//Play Class
template<size_t DOF>
class Play {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
protected:
	systems::Wam<DOF>& wam;
	Hand* hand;
	ProductManager& pm;
	std::string playName;
	int inputType;
	const libconfig::Setting& setting;
	libconfig::Config config;
	typedef boost::tuple<double, jp_type> input_jp_type;
	typedef boost::tuple<double, cp_type> input_cp_type;
	typedef boost::tuple<double, Eigen::Quaterniond> input_quat_type;

	ControlModeSwitcher<DOF>* cms;

	std::vector<input_cp_type, Eigen::aligned_allocator<input_cp_type> >* cpVec;
	std::vector<input_quat_type, Eigen::aligned_allocator<input_quat_type> >* qVec;
	math::Spline<jp_type>* jpSpline;
	math::Spline<cp_type>* cpSpline;
	math::Spline<Eigen::Quaterniond>* qSpline;
	systems::Callback<double, jp_type>* jpTrajectory;
	systems::Callback<double, cp_type>* cpTrajectory;
	systems::Callback<double, Eigen::Quaterniond>* qTrajectory;
	systems::TupleGrouper<cp_type, Eigen::Quaterniond> poseTg;
	systems::Ramp time;

public:
	int dataSize;
	bool loop;

	Play(systems::Wam<DOF>& wam_, ProductManager& pm_, std::string filename_,
			const libconfig::Setting& setting_) :
			wam(wam_), hand(NULL), pm(pm_), playName(filename_), inputType(0), setting(
					setting_), cms(NULL), cpVec(NULL), qVec(NULL), jpSpline(
					NULL), cpSpline(NULL), qSpline(NULL), jpTrajectory(NULL), cpTrajectory(
					NULL), qTrajectory(NULL), time(pm.getExecutionManager()), dataSize(
					0), loop(false) {
	}
	bool
	init();

	~Play() {
	}

	void
	displayEntryPoint();
	void
	moveToStart();
	void
	startPlayback();
	void
	pausePlayback();
	bool
	playbackActive();
	void
	disconnectSystems();
	void
	reconnectSystems();

private:
	DISALLOW_COPY_AND_ASSIGN(Play);

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

}
;

// Initialization - Gravity compensating, setting safety limits, parsing input file and creating trajectories
template<size_t DOF>
bool Play<DOF>::init() {
	// Turn on Gravity Compensation
	wam.gravityCompensate(true);

	// Modify the WAM Safety Limits
	pm.getSafetyModule()->setTorqueLimit(3.0);
	pm.getSafetyModule()->setVelocityLimit(1.5);

	// Create our control mode switcher to go between current control mode and voltage control mode
	cms = new ControlModeSwitcher<DOF>(pm, wam,
			setting["control_mode_switcher"]);

	//Create stream from input file
	std::ifstream fs(playName.c_str());
	std::string line;

	// Check to see the data type specified on the first line (jp_type or pose_type)
	// this will inform us if we are tracking 4DOF WAM Joint Angles, 7DOF WAM Joint Angles, or WAM Poses
	std::getline(fs, line);
	// Using a boost tokenizer to parse the data of the file into our vector.
	boost::char_separator<char> sep(",");
	typedef boost::tokenizer<boost::char_separator<char> > t_tokenizer;
	t_tokenizer tok(line, sep);
	if (strcmp(line.c_str(), "pose_type") == 0) {
		// Create our spline and trajectory if the first line of the parsed file informs us of a pose_type
		inputType = 1;
		cpVec = new std::vector<input_cp_type,
				Eigen::aligned_allocator<input_cp_type> >();
		qVec = new std::vector<input_quat_type,
				Eigen::aligned_allocator<input_quat_type> >();

		float fLine[8];
		input_cp_type cpSamp;
		input_quat_type qSamp;
		while (true) {
			std::getline(fs, line);
			if (!fs.good())
				break;
			t_tokenizer tok(line, sep);
			int j = 0;
			for (t_tokenizer::iterator beg = tok.begin(); beg != tok.end();
					++beg) {
				fLine[j] = boost::lexical_cast<float>(*beg);
				j++;
			}
			boost::get<0>(cpSamp) = fLine[0];
			boost::get<0>(qSamp) = boost::get<0>(cpSamp);

			boost::get<1>(cpSamp) << fLine[1], fLine[2], fLine[3];
			boost::get<1>(qSamp) = Eigen::Quaterniond(fLine[4], fLine[5],
					fLine[6], fLine[7]);
			boost::get<1>(qSamp).normalize();
			cpVec->push_back(cpSamp);
			qVec->push_back(qSamp);
		}
		// Make sure the vectors created are the same size
		assert(cpVec->size() == qVec->size());
		// Create our splines between points
		cpSpline = new math::Spline<cp_type>(*cpVec);
		qSpline = new math::Spline<Eigen::Quaterniond>(*qVec);
		// Create trajectories from the splines
		cpTrajectory = new systems::Callback<double, cp_type>(
				boost::ref(*cpSpline));
		qTrajectory = new systems::Callback<double, Eigen::Quaterniond>(
				boost::ref(*qSpline));
	} else if (strcmp(line.c_str(), "jp_type") == 0) {
		// Create our spline and trajectory if the first line of the parsed file informs us of a jp_type
		std::vector<input_jp_type, Eigen::aligned_allocator<input_jp_type> > jp_vec;
		float fLine[DOF + 1];
		input_jp_type samp;
		while (true) {
			std::getline(fs, line);
			if (!fs.good())
				break;
			t_tokenizer tok(line, sep);
			int j = 0;
			for (t_tokenizer::iterator beg = tok.begin(); beg != tok.end();
					++beg) {
				fLine[j] = boost::lexical_cast<float>(*beg);
				j++;
			}
			boost::get<0>(samp) = fLine[0];
			// To handle the different WAM configurations
			if (j == 5)
				boost::get<1>(samp) << fLine[1], fLine[2], fLine[3], fLine[4];
			else
				boost::get<1>(samp) << fLine[1], fLine[2], fLine[3], fLine[4], fLine[5], fLine[6], fLine[7];
			jp_vec.push_back(samp);
		}
		// Create our splines between points
		jpSpline = new math::Spline<jp_type>(jp_vec);
		// Create our trajectory
		jpTrajectory = new systems::Callback<double, jp_type>(
				boost::ref(*jpSpline));
	} else {
		// The first line does not contain "jp_type or pose_type" return false and exit.
		printf(
				"EXITING: First line of file must specify jp_type or pose_type data.");
		btsleep(1.5);
		return false;
	}

	//Close the file
	fs.close();
	printf("\nFile Contains data in the form of: %s\n\n",
			inputType == 0 ? "jp_type" : "pose_type");

	// Set our control mode
	if (vcMode == 1) {
		printf("Switching system to voltage control mode");
		cms->voltageControl();
		//Allow the mechanical system to settle
		btsleep(2.0);
	} else {
		printf("Verifying system is in current control mode");
		cms->currentControl();
	}

	pm.getExecutionManager()->startManaging(time); //starting time management
	return true;
}

// This function will run in a different thread and control displaying to the screen and user input
template<size_t DOF>
void Play<DOF>::displayEntryPoint() {
// We will check to see if a hand is present and initialize if so.
	hand = pm.getHand();
	if (hand != NULL) {
		printf(
				"Press [Enter] to initialize the Hand. (Make sure it has room!)");
		waitForEnter();
		hand->initialize();
	}
// Some hand variables allow for switching between open and close positions
	Hand::jp_type currentPos(0.0);
	Hand::jp_type nextPos(M_PI);
	nextPos[3] = 0;

// Instructions displayed to screen.
	printf("\n");
	printf("Commands:\n");
	printf("  p    Play\n");
	printf("  i    Pause\n");
	printf("  s    Stop\n");
	printf("  l    Loop\n");
	printf("  q    Quit\n");
	printf("  At any time, press [Enter] to open or close the Hand.\n");
	printf("\n");

	std::string line;
	while (true) {
		// Continuously parse our line input
		printf(">> ");
		std::getline(std::cin, line);

		if (line.size() == 0) { // Enter press recognized without any input
			// If hand is present, open or close accordingly
			if (hand != NULL) {
				hand->trapezoidalMove(nextPos, false);
				std::swap(currentPos, nextPos);
			}
		} else { // User input - Set State
			switch (line[0]) {
			case 'p':
				curState = PLAYING;
				break;
			case 'i':
				curState = PAUSED;
				break;
			case 's':
				loop = false;
				curState = STOPPED;
				break;
			case 'l':
				loop = true;
				curState = PLAYING;
				break;
			case 'q':
				loop = false;
				cms->currentControl();
				curState = QUIT;
				break;
			default:
				break;
			}
		}
	}
}

// Function to evaluate and move to the first pose in the trajectory
template<size_t DOF>
void Play<DOF>::moveToStart() {
	if (inputType == 0) {
		wam.moveTo(jpSpline->eval(jpSpline->initialS()), true);
	} else
		wam.moveTo(
				boost::make_tuple(cpSpline->eval(cpSpline->initialS()),
						qSpline->eval(qSpline->initialS())));
}

template<size_t DOF>
void Play<DOF>::startPlayback() {
	time.start();
}

template<size_t DOF>
void Play<DOF>::pausePlayback() {
	time.stop();
}

template<size_t DOF>
bool Play<DOF>::playbackActive() {
	if (inputType == 0)
		return (jpTrajectory->input.getValue() < jpSpline->finalS());
	else {
		return (cpTrajectory->input.getValue() < cpSpline->finalS());
	}
}

template<size_t DOF>
void Play<DOF>::disconnectSystems() {
	disconnect(wam.input);
	wam.idle();
	time.stop();
	time.setOutput(0.0);
}

template<size_t DOF>
void Play<DOF>::reconnectSystems() {
	if (inputType == 0) {
		systems::forceConnect(time.output, jpTrajectory->input);
		wam.trackReferenceSignal(jpTrajectory->output);
	} else {
		systems::forceConnect(time.output, cpTrajectory->input);
		systems::forceConnect(time.output, qTrajectory->input);
		systems::forceConnect(cpTrajectory->output, poseTg.getInput<0>());
		systems::forceConnect(qTrajectory->output, poseTg.getInput<1>());
		wam.trackReferenceSignal(poseTg.output);
	}
}

template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm,
		systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
	std::string filename(argv[1]);

// Load our vc_calibration file.
	libconfig::Config config;
	std::string calibration_file;
	if (DOF == 4)
		calibration_file = "calibration4.conf";
	else
		calibration_file = "calibration7.conf";
	config.readFile(calibration_file.c_str());
	config.getRoot();

	Play<DOF> play(wam, pm, filename, config.getRoot());

	if (!play.init())
		return 1;

	boost::thread displayThread(&Play<DOF>::displayEntryPoint, &play);

	bool playing = true;
	while (playing) {
		switch (curState) {
		case QUIT:
			playing = false;
			break;
		case PLAYING:
			switch (lastState) {
			case STOPPED:
				play.moveToStart();
				play.reconnectSystems();
				play.startPlayback();
				lastState = PLAYING;
				break;
			case PAUSED:
				play.startPlayback();
				lastState = PLAYING;
				break;
			case PLAYING:
				if (play.playbackActive()) {
					btsleep(0.1);
					break;
				} else if (play.loop) {
					play.disconnectSystems();
					lastState = STOPPED;
					curState = PLAYING;
					break;
				} else {
					curState = STOPPED;
					break;
				}
			default:
				break;
			}
			break;
		case PAUSED:
			switch (lastState) {
			case PLAYING:
				play.pausePlayback();
				lastState = PAUSED;
				break;
			case PAUSED:
				btsleep(0.1);
				break;
			case STOPPED:
				break;
			default:
				break;
			}
			break;
		case STOPPED:
			switch (lastState) {
			case PLAYING:
				play.disconnectSystems();
				lastState = STOPPED;
				break;
			case PAUSED:
				play.disconnectSystems();
				lastState = STOPPED;
				break;
			case STOPPED:
				btsleep(0.1);
				break;
			default:
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
