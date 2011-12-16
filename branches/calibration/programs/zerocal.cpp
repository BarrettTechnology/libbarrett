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
#include <cmath>

#include <unistd.h>  // For usleep()

#include <curses.h>
#include <boost/lexical_cast.hpp>
#include <libconfig.h++>

#include <barrett/exception.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>
#include <barrett/detail/stl_utils.h>

#include "get_key.h"


using namespace barrett;
using detail::waitForEnter;

//const char CAL_CONFIG_FILE[] = "/etc/barrett/calibration.conf";
const char CAL_CONFIG_FILE[] = "/home/robot/libbarrett/config/calibration.conf";


class CalibrationStep {
public:
	explicit CalibrationStep(const std::string& title_ = "") :
		selected(false), active(false), title(title_) {}
	virtual ~CalibrationStep() {}

	void setSelected(bool on) { selected = on; }
	void setActive(bool on) {
		if (active != on) {
			active = on;
			onChangeActive();
		}
	}

	virtual void onChangeActive() {}
	virtual bool onKeyPress(enum Key k) {
		return true;
	}

	static const int L_OFFSET = 8;
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
	explicit AdjustJoint(size_t jointIndex, const jp_type& pos_, systems::Wam<DOF>* wam_, jp_type* offsets_) :
		CalibrationStep("Joint " + boost::lexical_cast<std::string>(jointIndex+1)),
		j(jointIndex), pos(pos_), wam(wam_), offsets(offsets_), state(0)
	{
		assert(jointIndex >= 0);
		assert(jointIndex < DOF);
		assert(wam != NULL);
		assert(offsets != NULL);
	}
	virtual ~AdjustJoint() {}


	virtual void onChangeActive() {
		if (active) {
			state = 0;
		}
	}

	virtual bool onKeyPress(enum Key k) {
		switch (k) {

		}

		return true;
	}

	virtual int display(int top, int left) {
		int line = top + CalibrationStep::display(top, left);  // Call super
		left += L_OFFSET;

		if (active) {
			mvprintw(line++,left, "--> Press [Enter] to move to: [%0.2f", pos[0]);
			for (size_t i = 1; i < DOF; ++i) {
				printw(", %0.2f", pos[i]);
			}
			printw("]");

			mvprintw(line++,left, "    Adjust Joint %d to make the outer link vertical", j+1);
		}

		return height(top);
	}

protected:
	size_t j;
	const jp_type pos;
	systems::Wam<DOF>* wam;
	jp_type* offsets;

	int state;
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


	// Read from config
	std::vector<CalibrationStep*> steps;
	try {
		libconfig::Config config;
		config.readFile(CAL_CONFIG_FILE);
		const libconfig::Setting& setting = config.lookup("zerocal")[pm.getWamDefaultConfigPath()];
		assert(setting.isList());
		assert(setting.getLength() >= 1);
		assert(setting.getLength() <= (int)DOF);

		for (int i = 0; i < setting.getLength(); ++i) {
			assert(setting[i].isList());
			assert(setting[i].getLength() == 2);

			const libconfig::Setting& jointListSetting = setting[i][0];
			assert(jointListSetting.isArray()  ||  jointListSetting.isList());
			jp_type pos(setting[i][1]);

			for (int j = 0; j < jointListSetting.getLength(); ++j) {
				assert(jointListSetting[j].isNumber());
				int jointNumer = jointListSetting[j];
				assert(jointNumer >= 1);
				assert(jointNumer <= (int)DOF);

				steps.push_back(new AdjustJoint<DOF>(jointNumer-1, pos, &wam, &offsets));
			}
		}
	} catch (libconfig::ParseException e) {
		printf(">>> CONFIG FILE ERROR on line %d of %s: \"%s\"\n", e.getLine(), CAL_CONFIG_FILE, e.getError());
		return 1;
	} catch (libconfig::SettingNotFoundException e) {
		printf(">>> CONFIG FILE ERROR in %s: could not find \"%s\"\n", CAL_CONFIG_FILE, e.getPath());
		return 1;
	} catch (libconfig::SettingTypeException e) {
		printf(">>> CONFIG FILE ERROR in %s: \"%s\" is the wrong type\n", CAL_CONFIG_FILE, e.getPath());
		return 1;
	}


	// Set up ncurses
	initscr();
	curs_set(0);
	noecho();
	timeout(0);
	std::atexit((void (*)())endwin);


	// Display loop
	assert(steps.size() >= 2);
	int selected = 1;
	steps[selected]->setSelected(true);
	int active = 1;
	steps[active]->setActive(true);

	enum Key k;
	while (pm.getSafetyModule()->getMode() == SafetyModule::ACTIVE) {
		clear();

		k = getKey();
		if (active == -1) {  // Menu mode
			switch (k) {
			case K_UP:
				steps[selected]->setSelected(false);
				selected = std::max(0, selected-1);
				steps[selected]->setSelected(true);
				break;
			case K_DOWN:
				steps[selected]->setSelected(false);
				selected = std::min((int)steps.size()-1, selected+1);
				steps[selected]->setSelected(true);
				break;
			case K_ENTER:
				active = selected;
				steps[active]->setActive(true);
				break;
			default:
				break;
			}
		} else {  // Adjust mode
			switch (k) {
			case K_BACKSPACE:
			case K_ESCAPE:
			case K_TAB:
				steps[active]->setActive(false);
				active = -1;
				break;
			default:
				// Forward to active item
				break;
			}
		}

		int top = 5;
		for (size_t i = 0; i < DOF; ++i) {
			top += steps[i]->display(top,0);
		}

		refresh();
		usleep(100000);  // Slow loop to ~10Hz
	}

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
