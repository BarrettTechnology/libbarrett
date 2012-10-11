/* ex04_display_basic_info.cpp
 *
 * This example uses the ncurses library to display information about the WAM,
 * and (if present) the Barrett Force-Torque Sensor (FTS) and BarrettHand. The
 * ProductManager is used to check for these products and get handles to objects
 * that allow us to interact with them.
 */


#include <iostream>  // For std::cin
#include <string>  // For std::string and std::getline()
#include <cstdlib>  // For std::atexit()

// The ncurses library allows us to write text to any location on the screen
#include <curses.h>

#include <barrett/os.h>  // For btsleep()
#include <barrett/math.h>  // For barrett::math::saturate()
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>

#include <barrett/standard_main_function.h>


using namespace barrett;


void waitForEnter() {
	std::string line;
	std::getline(std::cin, line);
}

// Functions that help display data from the Hand's (optional) tactile sensors.
// Note that the palm tactile sensor has a unique cell layout that these
// functions do not print correctly.
const int TACT_CELL_HEIGHT = 3;
const int TACT_CELL_WIDTH = 6;
const int TACT_BOARD_ROWS = 8;
const int TACT_BOARD_COLS = 3;
const int TACT_BOARD_STRIDE = TACT_BOARD_COLS * TACT_CELL_WIDTH + 2;
void drawBoard(WINDOW *win, int starty, int startx, int rows, int cols,
		int tileHeight, int tileWidth);
void graphPressures(WINDOW *win, int starty, int startx,
		const TactilePuck::v_type& pressures);


template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);


	wam.gravityCompensate();


	// Is an FTS attached?
	ForceTorqueSensor* fts = NULL;
	if (pm.foundForceTorqueSensor()) {
		fts = pm.getForceTorqueSensor();
		fts->tare();
	}

	// Is a Hand attached?
	Hand* hand = NULL;
	std::vector<TactilePuck*> tps;
	if (pm.foundHand()) {
		hand = pm.getHand();

		printf(">>> Press [Enter] to initialize Hand. (Make sure it has room!)");
		waitForEnter();
		hand->initialize();
	}


	// Set up the ncurses environment
	initscr();
	curs_set(0);
	noecho();
	timeout(0);

	// Make sure we cleanup after ncurses when the program exits
	std::atexit((void (*)())endwin);


	// Set up the static text on the screen
	int wamY = 0, wamX = 0;
	int ftsY = 0, ftsX = 0;
	int handY = 0, handX = 0;
	int line = 0;

	mvprintw(line++,0, "WAM");
	mvprintw(line++,0, "     Joint Positions (rad): ");
	getyx(stdscr, wamY, wamX);
	mvprintw(line++,0, "  Joint Velocities (rad/s): ");
	mvprintw(line++,0, "       Joint Torques (N*m): ");
	mvprintw(line++,0, "         Tool Position (m): ");
	mvprintw(line++,0, "   Tool Orientation (quat): ");
	line++;

	if (fts != NULL) {
		mvprintw(line++,0, "F/T Sensor");
		mvprintw(line++,0, "             Force (N): ");
		getyx(stdscr, ftsY, ftsX);
		mvprintw(line++,0, "          Torque (N*m): ");
		mvprintw(line++,0, "  Acceleration (m/s^2): ");
		line++;
	}

	if (hand != NULL) {
		mvprintw(line++,0, "Hand");
		mvprintw(line++,0, "      Inner Position (rad): ");
		getyx(stdscr, handY, handX);
		mvprintw(line++,0, "      Outer Position (rad): ");
		mvprintw(line++,0, "  Fingertip Torque sensors: ");
		if ( !hand->hasFingertipTorqueSensors() ) {
			printw(" n/a");
		}
		mvprintw(line++,0, "           Tactile sensors: ");
		if (hand->hasTactSensors()) {
			tps = hand->getTactilePucks();
			for (size_t i = 0; i < tps.size(); ++i) {
				drawBoard(stdscr,
						line, i * TACT_BOARD_STRIDE,
						TACT_BOARD_ROWS, TACT_BOARD_COLS,
						TACT_CELL_HEIGHT, TACT_CELL_WIDTH);
			}
		} else {
			printw(" n/a");
		}
		line++;
	}


	// Display loop!
	jp_type jp;
	jv_type jv;
	jt_type jt;
	cp_type cp;
	Eigen::Quaterniond to;
	math::Matrix<6,DOF> J;

	cf_type cf;
	ct_type ct;
	ca_type ca;

	Hand::jp_type hjp;


	// Fall out of the loop once the user Shift-idles
	while (pm.getSafetyModule()->getMode() == SafetyModule::ACTIVE) {
		// WAM
		line = wamY;

		// math::saturate() prevents the absolute value of the joint positions
		// from exceeding 9.9999. This puts an upper limit on the length of the
		// string that gets printed to the screen below. We do this to make sure
		// that the string will fit properly on the screen.
		jp = math::saturate(wam.getJointPositions(), 9.999);
		mvprintw(line++,wamX, "[%6.3f", jp[0]);
		for (size_t i = 1; i < DOF; ++i) {
			printw(", %6.3f", jp[i]);
		}
		printw("]");

		jv = math::saturate(wam.getJointVelocities(), 9.999);
		mvprintw(line++,wamX, "[%6.3f", jv[0]);
		for (size_t i = 1; i < DOF; ++i) {
			printw(", %6.3f", jv[i]);
		}
		printw("]");

		jt = math::saturate(wam.getJointTorques(), 99.99);
		mvprintw(line++,wamX, "[%6.2f", jt[0]);
		for (size_t i = 1; i < DOF; ++i) {
			printw(", %6.2f", jt[i]);
		}
		printw("]");

		cp = math::saturate(wam.getToolPosition(), 9.999);
    	mvprintw(line++,wamX, "[%6.3f, %6.3f, %6.3f]", cp[0], cp[1], cp[2]);

		to = wam.getToolOrientation();  // We work only with unit quaternions. No saturation necessary.
    	mvprintw(line++,wamX, "%+7.4f %+7.4fi %+7.4fj %+7.4fk", to.w(), to.x(), to.y(), to.z());


		// FTS
		if (fts != NULL) {
			line = ftsY;

            fts->update();
            cf = math::saturate(fts->getForce(), 99.99);
        	mvprintw(line++,ftsX, "[%6.2f, %6.2f, %6.2f]", cf[0], cf[1], cf[2]);
            ct = math::saturate(fts->getTorque(), 9.999);
        	mvprintw(line++,ftsX, "[%6.3f, %6.3f, %6.3f]", ct[0], ct[1], ct[2]);

        	fts->updateAccel();
            ca = math::saturate(fts->getAccel(), 99.99);
        	mvprintw(line++,ftsX, "[%6.2f, %6.2f, %6.2f]", ca[0], ca[1], ca[2]);
		}


		// Hand
		if (hand != NULL) {
			line = handY;
			hand->update();  // Update all sensors

			hjp = math::saturate(hand->getInnerLinkPosition(), 9.999);
			mvprintw(line++,handX, "[%6.3f, %6.3f, %6.3f, %6.3f]",
					hjp[0], hjp[1], hjp[2], hjp[3]);
			hjp = math::saturate(hand->getOuterLinkPosition(), 9.999);
			mvprintw(line++,handX, "[%6.3f, %6.3f, %6.3f, %6.3f]",
					hjp[0], hjp[1], hjp[2], hjp[3]);

			if (hand->hasFingertipTorqueSensors()) {
				mvprintw(line,handX, "[%4d, %4d, %4d, %4d]",
						hand->getFingertipTorque()[0],
						hand->getFingertipTorque()[1],
						hand->getFingertipTorque()[2],
						hand->getFingertipTorque()[3]);
			}

			line += 2;
			if (hand->hasTactSensors()) {
				for (size_t i = 0; i < tps.size(); ++i) {
					graphPressures(stdscr, line, i * TACT_BOARD_STRIDE,
							tps[i]->getFullData());
				}
			}
		}


		refresh();  // Ask ncurses to display the new text
		btsleep(0.1);  // Slow the loop rate down to roughly 10 Hz
	}

	return 0;
}


void drawBoard(WINDOW *win, int starty, int startx, int rows, int cols,
		int tileHeight, int tileWidth) {
	int endy, endx, i, j;

	endy = starty + rows * tileHeight;
	endx = startx + cols * tileWidth;

	for (j = starty; j <= endy; j += tileHeight)
		for (i = startx; i <= endx; ++i)
			mvwaddch(win, j, i, ACS_HLINE);
	for (i = startx; i <= endx; i += tileWidth)
		for (j = starty; j <= endy; ++j)
			mvwaddch(win, j, i, ACS_VLINE);
	mvwaddch(win, starty, startx, ACS_ULCORNER);
	mvwaddch(win, endy, startx, ACS_LLCORNER);
	mvwaddch(win, starty, endx, ACS_URCORNER);
	mvwaddch(win, endy, endx, ACS_LRCORNER);
	for (j = starty + tileHeight; j <= endy - tileHeight; j += tileHeight) {
		mvwaddch(win, j, startx, ACS_LTEE);
		mvwaddch(win, j, endx, ACS_RTEE);
		for (i = startx + tileWidth; i <= endx - tileWidth; i += tileWidth)
			mvwaddch(win, j, i, ACS_PLUS);
	}
	for (i = startx + tileWidth; i <= endx - tileWidth; i += tileWidth) {
		mvwaddch(win, starty, i, ACS_TTEE);
		mvwaddch(win, endy, i, ACS_BTEE);
	}
}

void graphCell(WINDOW *win, int starty, int startx, double pressure) {
	int i, chunk;
	char c;

	int value = (int)(pressure * 256.0) / 102;  // integer division
//	int value = (int)(pressure * 256.0) / 50; // integer division
	for (i = 4; i >= 0; --i) {
		chunk = (value <= 7) ? value : 7;
		value -= chunk;

		switch (chunk) {
		default:  c = '#'; break;
		case 2:   c = '~'; break;
		case 1:   c = '-'; break;
		case 0:   c = '_'; break;
		}
		mvwprintw(win, starty + 1, startx + i, "%c", c);

		switch (chunk - 4) {
		case 3:   c = '#'; break;
		case 2:   c = '~'; break;
		case 1:   c = '-'; break;
		case 0:   c = '_'; break;
		default:  c = ' '; break;
		}
		mvwprintw(win, starty, startx + i, "%c", c);
	}
}

void graphPressures(WINDOW *win, int starty, int startx,
		const TactilePuck::v_type& pressures) {
	for (int i = 0; i < pressures.size(); ++i) {
		graphCell(win,
				starty + 1 + TACT_CELL_HEIGHT *
						(TACT_BOARD_ROWS - 1 - (i / 3 /* integer division */)),
				startx + 1 + TACT_CELL_WIDTH * (i % TACT_BOARD_COLS),
				pressures[i]);
	}
}
