/*
 * zerocal.cpp
 *
 *  Created on: Apr 8, 2010
 *      Author: cd
 *      Author: dc
 */

#include <iostream>
#include <string>
#include <vector>
#include <cstdlib>  // For std::atexit()
#include <cassert>

#include <unistd.h>  // For usleep()

#include <curses.h>
#include <boost/lexical_cast.hpp>

#include <barrett/exception.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>
#include <barrett/detail/stl_utils.h>


using namespace barrett;
using detail::waitForEnter;


class CalibrationStep {
public:
	explicit CalibrationStep(const std::string& title_ = "") :
		selected(false), active(false), title(title_) {}
	virtual ~CalibrationStep() {}

	void setSelected(bool on) { selected = on; }
	void setActive(bool on) { active = on; }

	static const int T_OFF = 1;
	static const int L_OFF = 8;
	virtual int display(int top, int left) {
		if (selected) {
			attron(A_BOLD);
			mvprintw(top,left, "--> ] ");
		} else {
			mvprintw(top,left, "    ] ");
		}
		printw("%s", title.c_str());
		if(selected) {
			attroff(A_BOLD);
		}

		return 1;
	}


	static int height(int origTop) {
		int y, x;
		getyx(stdscr, y,x);
		return y - origTop + 1;
	}

protected:
	bool selected, active;
	std::string title;
};

template<size_t DOF>
class AdjustJoint : public CalibrationStep {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

public:
	explicit AdjustJoint(size_t jointIndex, const jp_type& pos, jp_type* offsets_) :
		CalibrationStep("Joint " + boost::lexical_cast<std::string>(jointIndex+1)),
		j(jointIndex), pos(pos_), offsets(offsets_)
	{
		assert(jointIndex >= 0);
		assert(jointIndex < DOF);
		assert(offsets != NULL);
	}
	virtual ~AdjustJoint() {}

	virtual int display(int top, int left) {
		const int origTop = top;
		CalibrationStep::display(top, left);  // Call super

		top += T_OFF;
		left += L_OFF;

		if (active) {
			mvprintw(top,left, "OMG! %d", j);
		}

		return height(origTop);
	}

protected:
	size_t j;
	const jp_type pos;
	jp_type* offsets;
};

template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	// Record the starting position. (Should be identical to wam.getHomePosition().)
	const jp_type home(wam.getJointPositions());
	jp_type offsets(0.0);

	// Disable torque saturation because gravity compensation is off
	wam.jpController.setControlSignalLimit(jp_type());
	wam.moveTo(home);  // Hold position


	// Set up ncurses
	initscr();
	curs_set(0);
	noecho();
	timeout(0);
	std::atexit((void (*)())endwin);

	std::vector<CalibrationStep*> steps;
	for (size_t i = 0; i < DOF; ++i) {
		steps.push_back(new AdjustJoint<DOF>(i));
	}
	steps[1]->setSelected(true);
	steps[2]->setActive(true);

	int top = 5;
	for (size_t i = 0; i < DOF; ++i) {
		top += steps[i]->display(top,0);
	}

	refresh();
	detail::purge(steps);


	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
	// Make sure the user applies their new calibration
	pm.getSafetyModule()->setWamZeroed(false);
	return 0;
}


int main(int argc, char** argv) {
	// Give us pretty stack-traces when things die
	installExceptionHandler();


	printf(
"\n"
"*** Barrett WAM Zero Calibration Utility ***\n"
"\n"
"This utility will help you set the joint angle offsets for your WAM Arm. Joint\n"
"angle offsets affect operations that rely on the robot's kinematics, such as\n"
"gravity compensation, haptics, and Cartesian position and orientation control.\n"
"You should perform this calibration any time you replace a broken cable, add\n"
"tension to a cable, or re-mount a Puck.\n"
"\n"
"The WAM's standard home position is described in the User Manual. However, you\n"
"may choose any other pose to be your home position if it is more convenient for\n"
"your application. You will need to return the WAM to the home position on power-\n"
"up and after certain types of safety faults.\n"
"\n"
"Modern WAMs have absolute encoders on their motors, so the WAM only needs to be\n"
"returned to within one motor revolution of its home position. This translates to\n"
"roughly +/- 5 degrees at the joint. (The tolerance is different for each joint.)\n"
"\n"
"This program assumes the WAM is mounted in its standard orientation.\n"
"\n"
"\n"
	);


	ProductManager pm;
	pm.waitForWam(false);
	pm.wakeAllPucks();

	SafetyModule* sm = pm.getSafetyModule();
	if (sm == NULL) {
		printf("ERROR: No SafetyModule found.\n");
		return 1;
	} else {
		sm->setWamZeroed(false);
	}

	// Remove existing zerocal information, if present
	libconfig::Setting& llSetting = pm.getConfig().lookup(pm.getWamDefaultConfigPath())["low_level"];
	if (llSetting.exists("zeroangle")) {
		llSetting.remove(llSetting["zeroangle"].getIndex());
		syslog(LOG_ERR, "** Ignoring previous \"zeroangle\" vector **");
	}

	printf(">>> Please *carefully* place the WAM in its home position, then press [Enter].");
	waitForEnter();

	if (pm.foundWam4()) {
		return wam_main(argc, argv, pm, *pm.getWam4());
	} else if (pm.foundWam7()) {
		return wam_main(argc, argv, pm, *pm.getWam7());
	} else {
		printf(">>> ERROR: No WAM was found. Perhaps you have found a bug in ProductManager::waitForWam().\n");
		return 1;
	}
}
