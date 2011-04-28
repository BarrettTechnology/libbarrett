#include <cstdlib>

#include <unistd.h>

#include <curses.h>

#include <barrett/math.h>
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


	int wamY = 0, wamX = 0;
	int ftsY = 0, ftsX = 0;
	int handY = 0, handX = 0;
	int line = 0;

	mvprintw(line++,0, "WAM");
	mvprintw(line++,0, "  Joint Positions (rad): ");
	getyx(stdscr, wamY, wamX);
	mvprintw(line++,0, "    Joint Torques (N*m): ");
	line++;

	if (fts != NULL) {
		mvprintw(line++,0, "F/T Sensor");
		mvprintw(line++,0, "     Force (N): ");
		getyx(stdscr, ftsY, ftsX);
		mvprintw(line++,0, "  Torque (N*m): ");
		line++;
	}

	if (hand != NULL) {
		mvprintw(line++,0, "Hand");
		mvprintw(line++,0, "  Inner Position (rad): ");
		getyx(stdscr, handY, handX);
		mvprintw(line++,0, "  Outer Position (rad): ");
		mvprintw(line++,0, "  Strain-gauge sensors: ");
		if ( !hand->hasStrainSensors() ) {
			printw(" n/a");
		}
		mvprintw(line++,0, "       Tactile sensors: ");
		if ( !hand->hasTactSensors() ) {
			printw(" n/a");
		}
		line++;
	}

	refresh();


	jp_type jp;
	jt_type jt;
	cf_type cf;
	ct_type ct;
	Hand::jp_type hjp;

	bool going = true;
	while (going  &&  pm.getSafetyModule()->getMode() == SafetyModule::ACTIVE) {
		switch(getch()) {
		case 'x':
		case 'q':
			going = false;
			break;

		case 'p':
			break;

		default:
			break;
		}


		// WAM
		line = wamY;

		jp = math::saturate(wam.getJointPositions(), 9.999);
		mvprintw(line++,wamX, "[%6.3f", jp[0]);
		for (size_t i = 1; i < DOF; ++i) {
			printw(", %6.3f", jp[i]);
		}
		printw("]");

		jt = math::saturate(wam.getJointTorques(), 99.99);
		mvprintw(line++,wamX, "[%6.2f", jt[0]);
		for (size_t i = 1; i < DOF; ++i) {
			printw(", %6.2f", jt[i]);
		}
		printw("]");


		// FTS
		if (fts != NULL) {
			line = ftsY;

            fts->update();
            cf = math::saturate(fts->getForce(), 99.99);
        	mvprintw(line++,ftsX, "[%6.2f, %6.2f, %6.2f]", cf[0], cf[1], cf[2]);
            ct = math::saturate(fts->getTorque(), 9.999);
        	mvprintw(line++,ftsX, "[%6.3f, %6.3f, %6.3f]", ct[0], ct[1], ct[2]);
		}


		// Hand
		if (hand != NULL) {
			line = handY;

			hand->updatePosition();
			hjp = math::saturate(hand->getInnerLinkPosition(), 9.999);
			mvprintw(line++,handX, "[%6.3f, %6.3f, %6.3f, %6.3f]", hjp[0], hjp[1], hjp[2], hjp[3]);
			hjp = math::saturate(hand->getOuterLinkPosition(), 9.999);
			mvprintw(line++,handX, "[%6.3f, %6.3f, %6.3f, %6.3f]", hjp[0], hjp[1], hjp[2], hjp[3]);

			if (hand->hasStrainSensors()) {
				hand->updateStrain();
				mvprintw(line++,handX, "[%4d, %4d, %4d, %4d]", hand->getStrain()[0], hand->getStrain()[1], hand->getStrain()[2], hand->getStrain()[3]);
			}

			if (hand->hasTactSensors()) {
				hand->updateTactFull();

//				for (size_t i = 0; i < tps.size(); ++i) {
//					graphPressures(stdscr, 0,i*(3*WIDTH+2), tps[i]->getFullData());
//				}
			}
		}

		refresh();
		usleep(200000);
	}

	endwin();

	// Wait for the user to press Shift-idle
	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
	return 0;
}
