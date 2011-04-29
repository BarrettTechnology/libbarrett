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


const int TACT_CELL_HEIGHT = 3;
const int TACT_CELL_WIDTH = 6;
const int TACT_BOARD_ROWS = 8;
const int TACT_BOARD_COLS = 3;
const int TACT_BOARD_STRIDE = TACT_BOARD_COLS * TACT_CELL_WIDTH + 2;
void drawBoard(WINDOW *win, int starty, int startx, int rows, int cols,
		int tileHeight, int tileWidth);
void graphPressures(WINDOW *win, int starty, int startx, const TactilePuck::v_type& pressures);


template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);


	wam.gravityCompensate();


	ForceTorqueSensor* fts = NULL;
	if (pm.foundForceTorqueSensor()) {
		fts = pm.getForceTorqueSensor();
		fts->tare();
	}

	Hand* hand = NULL;
	std::vector<TactilePuck*> tps;
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


	jp_type jp;
	jt_type jt;
	cf_type cf;
	ct_type ct;
	Hand::jp_type hjp;

	while (pm.getSafetyModule()->getMode() == SafetyModule::ACTIVE) {
		switch(getch()) {
		case 'p':
			break;

		default:
			break;
		}


		// WAM
		line = wamY;

		jp = math::saturate(wam.getJointPositions(), 9.9999);
		mvprintw(line++,wamX, "[%7.4f", jp[0]);
		for (size_t i = 1; i < DOF; ++i) {
			printw(", %7.4f", jp[i]);
		}
		printw("]");

		jt = math::saturate(wam.getJointTorques(), 99.999);
		mvprintw(line++,wamX, "[%7.3f", jt[0]);
		for (size_t i = 1; i < DOF; ++i) {
			printw(", %7.3f", jt[i]);
		}
		printw("]");


		// FTS
		if (fts != NULL) {
			line = ftsY;

            fts->update();
            cf = math::saturate(fts->getForce(), 99.999);
        	mvprintw(line++,ftsX, "[%7.3f, %7.3f, %7.3f]", cf[0], cf[1], cf[2]);
            ct = math::saturate(fts->getTorque(), 9.9999);
        	mvprintw(line++,ftsX, "[%7.4f, %7.4f, %7.4f]", ct[0], ct[1], ct[2]);
		}


		// Hand
		if (hand != NULL) {
			line = handY;

			hand->updatePosition();
			hjp = math::saturate(hand->getInnerLinkPosition(), 9.9999);
			mvprintw(line++,handX, "[%7.4f, %7.4f, %7.4f, %7.4f]",
					hjp[0], hjp[1], hjp[2], hjp[3]);
			hjp = math::saturate(hand->getOuterLinkPosition(), 9.9999);
			mvprintw(line++,handX, "[%7.4f, %7.4f, %7.4f, %7.4f]",
					hjp[0], hjp[1], hjp[2], hjp[3]);

			if (hand->hasStrainSensors()) {
				hand->updateStrain();
				mvprintw(line,handX, "[%4d, %4d, %4d, %4d]",
						hand->getStrain()[0], hand->getStrain()[1],
						hand->getStrain()[2], hand->getStrain()[3]);
			}

			line += 2;
			if (hand->hasTactSensors()) {
				hand->updateTactFull();

				for (size_t i = 0; i < tps.size(); ++i) {
					graphPressures(stdscr, line, i * TACT_BOARD_STRIDE,
							tps[i]->getFullData());
				}
			}
		}

		refresh();
		usleep(200000);
	}

	return 0;
}


void drawBoard(WINDOW *win, int starty, int startx, int rows, int cols, int tileHeight, int tileWidth) {
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

void graphPressures(WINDOW *win, int starty, int startx, const TactilePuck::v_type& pressures) {
	for (int i = 0; i < pressures.size(); ++i) {
		graphCell(win,
				starty + 1 + TACT_CELL_HEIGHT * (TACT_BOARD_ROWS - 1 - (i / 3 /* integer division */)),
				startx + 1 + TACT_CELL_WIDTH * (i % TACT_BOARD_COLS),
				pressures[i]);
	}
}

