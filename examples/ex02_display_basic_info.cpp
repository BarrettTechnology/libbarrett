#include <cstdlib>

#include <curses.h>

#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>
#include <barrett/detail/stl_utils.h>

#include <barrett/standard_main_function.h>


using namespace barrett;
using detail::waitForEnter;


template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);


	wam.gravityCompensate();


	ForceTorqueSensor* fts = NULL;
	if (pm.foundForceTorqueSensor()) {
		fts = pm.getForceTorqueSensor();
	}

	Hand* hand = NULL;
	if (pm.foundHand()) {
		hand = pm.getHand();

		printf(">>> Press [Enter] to initialize Hand. (Make sure it has room!)");
		waitForEnter();
		hand->initialize();
	}


	initscr();
	curs_set(0);
	noecho();
	timeout(0);
	std::atexit((void (*)())endwin);


	int line = 0;

	mvprintw(line++,0, "WAM");
	mvprintw(line++,0, "  Joint Positions (rad):");
	mvprintw(line++,0, "    Joint Torques (N*m):");
	line++;

	if (fts != NULL) {
		mvprintw(line++,0, "F/T Sensor");
		mvprintw(line++,0, "     Force (N):");
		mvprintw(line++,0, "  Torque (N*m):");
		line++;
	}

	if (hand != NULL) {
		mvprintw(line++,0, "Hand");
		mvprintw(line++,0, "  Inner Position (rad):");
		mvprintw(line++,0, "  Outer Position (rad):");
		mvprintw(line++,0, "  Strain-gauge sensors:");
		if ( !hand->hasStrainSensors() ) {
			printw(" n/a");
		}
		mvprintw(line++,0, "       Tactile sensors:");
		if ( !hand->hasTactSensors() ) {
			printw(" n/a");
		}
		line++;
	}

	refresh();


	// Wait for the user to press Shift-idle
	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE, false);
	return 0;
}
