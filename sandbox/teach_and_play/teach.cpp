/*
 * teach.cpp
 *
 *  Created on: Aug 31, 2012
 *      Author: km
 */

#include <iostream>
#include <vector>
#include <string>
#include <fstream>

#include <boost/ref.hpp>
#include <boost/bind.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/thread.hpp>
#include <curses.h>

#include <barrett/detail/stl_utils.h>
#include <barrett/math.h>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/log.h>
#include <barrett/products/product_manager.h>

#define BARRETT_SMF_VALIDATE_ARGS
#include <barrett/standard_main_function.h>

#include "control_mode_switcher.h"

using namespace barrett;
using detail::waitForEnter;
using systems::connect;
using systems::disconnect;
using systems::reconnect;

int recordType = 0;

bool validate_args(int argc, char** argv) {
	switch (argc) {
	case 2:
		printf("\nTrajectory to be recorded in joint space: %s\n\n", argv[1]);
		return true;
		break;
	case 3: {
		char* recordMode(argv[2]);
		if ((strcmp(recordMode, "jp") == 0 || strcmp(recordMode, "-jp") == 0
				|| strcmp(recordMode, "pose") == 0
				|| strcmp(recordMode, "-pose") == 0)) {
			printf(
					"\nTrajectory to be recorded in %s: %s\n\n",
					strcmp(recordMode, "jp") == 0
							|| strcmp(recordMode, "-jp") == 0 ?
							"joint space" : "Cartesian space", argv[1]);
			if (strcmp(recordMode, "pose") == 0
					|| strcmp(recordMode, "-pose") == 0)
				recordType = 1;
			return true;
			break;
		} else {
			printf(
					"\nIncorrect recording data type specified: must be jp or pose\n\n");
			return false;
			break;
		}
	}
	default:
		printf("Usage: %s <trajectory_name> [<Record Mode (jp or pose)>]\n",
				argv[0]);
		return false;
		break;
	}
}

//Teach Class
template<size_t DOF>
class Teach {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
protected:
	systems::Wam<DOF>& wam;
	ProductManager& pm;
	std::string tmpStr, saveName, fileOut;
	char* tmpFile;
	struct LateInitializedDataMembers;
	LateInitializedDataMembers* d;
	int dLine, dX, dY, key;
	bool initCurses, displaying;
	typedef boost::tuple<double, jp_type> input_jp_type;
	systems::Ramp time;
	systems::TupleGrouper<double, jp_type> jpLogTg;
	systems::TupleGrouper<double, pose_type> poseLogTg;
	typedef boost::tuple<double, jp_type> jp_sample_type;
	typedef boost::tuple<double, pose_type> pose_sample_type;
	systems::PeriodicDataLogger<jp_sample_type>* jpLogger;
	systems::PeriodicDataLogger<pose_sample_type>* poseLogger;

public:
	Teach(systems::Wam<DOF>& wam_, ProductManager& pm_, std::string filename_) :
			wam(wam_), pm(pm_), tmpStr("/tmp/btXXXXXX"), saveName(filename_), dLine(
					0), dX(0), dY(0), key(0), initCurses(true), displaying(
					true), time(NULL) {
	}
	bool
	init();

	~Teach() {
		delete[] tmpFile;
	}

	void
	record();
	void
	display();
	void
	createSpline();
}
;

template<size_t DOF>
bool Teach<DOF>::init() {
	tmpFile = new char[tmpStr.length() + 1];
	strcpy(tmpFile, tmpStr.c_str());
	if (mkstemp(tmpFile) == -1) {
		printf("ERROR: Couldn't create temporary file!\n");
		return false;
	}

	pm.getSafetyModule()->setVelocityLimit(1.5);
	pm.getSafetyModule()->setTorqueLimit(3.0);

	wam.gravityCompensate();

	pm.getExecutionManager()->startManaging(time); //starting time manager

	if (recordType == 0)
		jpLogger = new systems::PeriodicDataLogger<jp_sample_type>(
				pm.getExecutionManager(),
				new barrett::log::RealTimeWriter<jp_sample_type>(tmpFile,
						pm.getExecutionManager()->getPeriod()), 1);
	else
		poseLogger = new systems::PeriodicDataLogger<pose_sample_type>(
				pm.getExecutionManager(),
				new barrett::log::RealTimeWriter<pose_sample_type>(tmpFile,
						pm.getExecutionManager()->getPeriod()), 1);
	return true;
}

template<size_t DOF>
void Teach<DOF>::record() {
	BARRETT_SCOPED_LOCK(pm.getExecutionManager()->getMutex());

	if (recordType == 0) {
		connect(time.output, jpLogTg.template getInput<0>());
		connect(wam.jpOutput, jpLogTg.template getInput<1>());
		connect(jpLogTg.output, jpLogger->input);
	} else {
		connect(time.output, poseLogTg.template getInput<0>());
		connect(wam.toolPose.output, poseLogTg.template getInput<1>());
		connect(poseLogTg.output, poseLogger->input);
	}

	time.start();
}

template<size_t DOF>
void Teach<DOF>::display() {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
	int cnt = 0;
	int scnt = 0;
	if (initCurses) {
		// Set up the ncurses environment
		initscr();
		//curs_set(0);
		noecho();
		timeout(0);
		cbreak();

		//mvprintw(dLine++, 0, "WAM");
		//mvprintw(dLine++, 0, "     Joint Positions (rad): ");
		getyx(stdscr, dY, dX);

		for (size_t i = 0; i < DOF; i++)
			//mvprintw(dLine++, 0, "                        J%zu: ", i + 1);
			refresh();
		initCurses = false;
	}
	while (displaying) {
		dLine = 2;
		//key = getch();
		//jp_type jpc = wam.getJointPositions();
		for (size_t i = 0; i < DOF; i++)
			//mvprintw(dLine++, dX, "[%6.3f]", jpc[i]);
			if (getch() != -1) {
				cnt++;
			}
		mvprintw(dLine++, dX, "Key = %d", getch());
		mvprintw(dLine++, dX, "Count = %d", cnt);
		mvprintw(dLine++, dX, "Count = %d", scnt);
		scnt++;
		refresh();
		btsleep(0.1);
		if (scnt > 50)
			displaying = false;
	}
	endwin();
}

template<size_t DOF>
void Teach<DOF>::createSpline() {
	saveName = "recorded/" + saveName;
	if (recordType == 0) {
		jpLogger->closeLog();
		disconnect(jpLogger->input);
		// Build spline between recorded points
		log::Reader<jp_sample_type> lr(tmpFile);
		lr.exportCSV(saveName.c_str());
	}
	else{
		poseLogger->closeLog();
		disconnect(poseLogger->input);
		log::Reader<pose_sample_type> pr(tmpFile);
		pr.exportCSV(saveName.c_str());
	}
	// Adding our datatype as the first line of the recorded trajectory
	fileOut = saveName + ".csv";
	std::ifstream in(saveName.c_str());
	std::ofstream out(fileOut.c_str());
	if (recordType == 0)
		out << "jp_type\n";
	else
		out << "pose_type\n";
	out << in.rdbuf();
	out.close();
	in.close();
	remove(saveName.c_str());
	printf("Trajectory saved to the location: %s \n\n ", fileOut.c_str());
}

template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm,
		systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	std::string filename(argv[1]);

	Teach<DOF> teach(wam, pm, filename);

	teach.init();

	printf("\nPress [Enter] to start teaching.\n");
	waitForEnter();
	teach.record();
	//boost::thread t(&Teach<DOF>::display, &teach);

	printf("Press [Enter] to stop teaching.\n");
	waitForEnter();
	teach.createSpline();

	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);

	return 0;
}
