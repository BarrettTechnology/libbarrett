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
#include <cassert>
#include <cmath>

#include <unistd.h>  // For usleep()

#include <curses.h>
#include <boost/lexical_cast.hpp>
#include <Eigen/Core>
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


class ScopedAttr {
public:
	ScopedAttr() : isOn(false), a(-1) {}
	explicit ScopedAttr(unsigned long attr, bool startOn = false) : isOn(false), a(-1) {
		set(attr, startOn);
	}
	~ScopedAttr() { off(); }

	void set(unsigned long attr, bool startOn = true) {
		if (isOn) {
			off();
		}
		a = attr;
		if (startOn) {
			on();
		}
	}
	void on() {
		if (!isOn) {
			attron(a);
			isOn = true;
		}
	}
	void off() {
		if (isOn) {
			attroff(a);
			isOn = false;
		}
	}

protected:
	bool isOn;
	unsigned long a;
};


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

	static const int L_OFFSET = 10;
	virtual int display(int top, int left) {
		ScopedAttr sa(A_BOLD);
		if (selected) {
			sa.on();
			mvprintw(top,left, "--> ] ");
		} else {
			mvprintw(top,left, "    ] ");
		}
		printw("%s", title.c_str());

		return height(top);
	}

	static int height(int origTop) {
		return getcury(stdscr) - origTop + 1;
	}

protected:
	bool selected, active;
	std::string title;
};

template<size_t DOF>
class AdjustJointStep : public CalibrationStep {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	static const int DEFAULT_DIGIT = -2;
	static const int MAX_DIGIT = 0;
	static const int MIN_DIGIT = -3;

public:
	explicit AdjustJointStep(size_t jointIndex, systems::Wam<DOF>* wamPtr, const jp_type& calPos_, jp_type* calOffsetPtr, jp_type* zeroPosPtr, jp_type* mechPosPtr) :
		CalibrationStep("Joint " + boost::lexical_cast<std::string>(jointIndex+1)),
		j(jointIndex), wam(*wamPtr), calPos(calPos_), calOffset(*calOffsetPtr), zeroPos(*zeroPosPtr), mechPos(*mechPosPtr), state(0), digit(DEFAULT_DIGIT)
	{
		assert(jointIndex >= 0);
		assert(jointIndex < DOF);
		assert(wamPtr != NULL);
		assert(calOffsetPtr != NULL);
	}
	virtual ~AdjustJointStep() {}


	virtual void onChangeActive() {
		if (active) {
			state = 0;
			digit = DEFAULT_DIGIT;
		}
	}

	virtual bool onKeyPress(enum Key k) {
		Eigen::Matrix<int, DOF,1> mech;

		switch (state) {
		case 0:
			if (k == K_ENTER) {
				state++;
				move();
			}
			break;

		case 1:
			// This transition is handled in display().
			break;

		default:
			switch (k) {
			case K_LEFT:
				digit = std::min(MAX_DIGIT, digit+1);
				break;
			case K_RIGHT:
				digit = std::max(MIN_DIGIT, digit-1);
				break;

			case K_UP:
				calOffset[j] += std::pow(10.0, digit);
				move();
				break;
			case K_DOWN:
				calOffset[j] -= std::pow(10.0, digit);
				move();
				break;

			case K_ENTER:
				// Record actual joint position, not commanded joint position
				zeroPos[j] = wam.getJointPositions()[j];

				wam.llww.getLowLevelWam().getPuckGroup().getProperty(Puck::MECH, mech.data());
				mechPos[j] = (wam.llww.getLowLevelWam().getPuckToJointPositionTransform() * mech.template cast<double>())[j];
				return false;  // Move on to the next joint!
				break;

			default:
				break;
			}
		}

		return true;
	}

	virtual int display(int top, int left) {
		int line = top + CalibrationStep::display(top, left);  // Call super
		left += L_OFFSET;

		if (active) {
			ScopedAttr sa(A_STANDOUT, true);

			switch (state) {
			case 0:
				mvprintw(line++,left, "Press [Enter] to move to: [%0.2f", calPos[0]);
				for (size_t i = 1; i < DOF; ++i) {
					printw(", %04.2f", calPos[i]);
				}
				printw("]");
				break;
			case 1:
				mvprintw(line++,left, "Wait for the WAM to finish moving.");

				if (wam.moveIsDone()) {  // Have we arrived yet?
					state++;
				}
				break;
			default:
				mvprintw(line++,left, "Use arrow keys to adjust the Joint %d offset.", j+1);
				sa.off();
				line++;
				mvprintw(line++,left, "                 ");
				attron(A_UNDERLINE);
				printw("Offset (rad)");
				attroff(A_UNDERLINE);
				mvprintw(  line,left, "                    %+06.3f", calOffset[j]);

				int x = getcurx(stdscr);
				mvchgat(line, x-5-digit + ((digit<0) ? 1:0), 1, A_BOLD, 0, NULL);

				line += 2;
				mvprintw(line++,left, "Press [Enter] once the outer link is vertical.");
				break;
			}
		}

		return height(top);
	}

protected:
	void move() {
		while ( !wam.moveIsDone() ) {
			usleep(20000);
		}
		wam.moveTo(jp_type(calPos + calOffset), false);
	}

	size_t j;
	systems::Wam<DOF>& wam;
	const jp_type calPos;
	jp_type& calOffset;
	jp_type& zeroPos;
	jp_type& mechPos;

	int state, digit;
};


class ExitStep : public CalibrationStep {
public:
	explicit ExitStep(const std::string& title_ = "Exit") : CalibrationStep(title_) {}
	virtual ~ExitStep() {}

	virtual bool onKeyPress(enum Key k) {
		return k != K_ENTER;
	}

	virtual int display(int top, int left) {
		int line = top + CalibrationStep::display(top, left);  // Call super
		left += L_OFFSET;

		if (active) {
			ScopedAttr sa(A_STANDOUT, true);
			mvprintw(line++,left, "Press [Enter] to move the home position and record calibration data.");
		}

		return height(top);
	}
};


template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	// Record the starting position. (Should be identical to wam.getHomePosition().)
	const jp_type home(wam.getJointPositions());
	jp_type calOffset(0.0);
	jp_type zeroPos(0.0);
	jp_type mechPos(0.0);

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
			jp_type calPos(setting[i][1]);

			for (int j = 0; j < jointListSetting.getLength(); ++j) {
				assert(jointListSetting[j].isNumber());
				int jointNumer = jointListSetting[j];
				assert(jointNumer >= 1);
				assert(jointNumer <= (int)DOF);

				steps.push_back(new AdjustJointStep<DOF>(jointNumer-1, &wam, calPos, &calOffset, &zeroPos, &mechPos));
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
	steps.push_back(new ExitStep);


	// Set up ncurses
	initscr();
	curs_set(0);
	noecho();
	timeout(0);


	// Display loop
	assert(steps.size() >= 1);
	int selected = 0;
	steps[selected]->setSelected(true);
	int active = 0;
	steps[active]->setActive(true);

	enum Key k;
	bool done = false;
	while (pm.getSafetyModule()->getMode() == SafetyModule::ACTIVE  &&  !done) {
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
				// Forward to the active item by calling onKeyPress()
				if ( !steps[active]->onKeyPress(k) ) {
					// This step is done. Move to the next one.
					steps[selected]->setSelected(false);
					steps[active]->setActive(false);
					++selected;
					if (selected < (int)steps.size()) {
						active = selected;
						steps[selected]->setSelected(true);
						steps[active]->setActive(true);
					} else {  // Finished the last step!
						done = true;
					}
				}
				break;
			}
		}

		clear();
		mvprintw(0,18, "*** Barrett WAM Zero Calibration Utility ***");
		int top = 4;
		for (size_t i = 0; i < steps.size(); ++i) {
			top += steps[i]->display(top,0);
		}
		refresh();

		usleep(100000);  // Slow loop down to ~10Hz
	}
	endwin();  // Turn off ncurses
	detail::purge(steps);


	// Restore stdout's line buffering
	if (freopen(NULL, "w", stdout) == NULL) {
		fprintf(stderr, ">>> ERROR: freopen(stdout) failed.\n");
		return 1;
	}


	if (done) {
		printf(">>> Calibration completed!\n");
		wam.moveHome(false);
	} else {
		printf(">>> ERROR: WAM was Idled before the calibration was completed.\n");
		return 1;
	}


	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
	// Make sure the user applies their new calibration
	pm.getSafetyModule()->setWamZeroed(false);

	std::cout << calOffset << "\n";
	std::cout << zeroPos << "\n";
	std::cout << mechPos << "\n";

	return 0;
}


int main(int argc, char** argv) {
	// Give us pretty stack-traces when things die
	installExceptionHandler();


	printf(
"\n"
"                  *** Barrett WAM Zero Calibration Utility ***\n"
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
