/*
 * full_system_test.cpp
 *
 *  Created on: Nov 18, 2010
 *      Author: dc
 */

#include <vector>
#include <sstream>

#include <unistd.h>
#include <native/timer.h>
//#include <rtdk.h>

#include <curses.h>

#include <boost/thread.hpp>
#include <boost/ref.hpp>

#include <barrett/exception.h>
#include <barrett/detail/stl_utils.h>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>


using namespace barrett;
using detail::waitForEnter;
using boost::ref;


bool g_DisplayActive = false;
#define WIDTH  6
#define HEIGHT 3


enum btkey {
   BTKEY_UNKNOWN = -2,
   BTKEY_NOKEY = -1,
   BTKEY_TAB = 9,
   BTKEY_ENTER = 10,
   BTKEY_ESCAPE = 27,
   BTKEY_BACKSPACE = 127,
   BTKEY_UP = 256,
   BTKEY_DOWN = 257,
   BTKEY_LEFT = 258,
   BTKEY_RIGHT = 259
};
enum btkey btkey_get()
{
   int c1,c2,c3;

   /* Get the key from ncurses */
   c1 = getch();
   if (c1 == ERR) return BTKEY_NOKEY;

   /* Get all keyboard characters */
   if (32 <= c1 && c1 <= 126) return (enum btkey)c1;

   /* Get special keys */
   switch (c1)
   {
      case BTKEY_TAB:
      case BTKEY_ENTER:
      case BTKEY_BACKSPACE:
            return (enum btkey)c1;
      /* Get extended keyboard chars (eg arrow keys) */
      case 27:
         c2 = getch();
         if (c2 == ERR) return BTKEY_ESCAPE;
         if (c2 != 91) return BTKEY_UNKNOWN;
         c3 = getch();
         switch (c3)
         {
            case 65: return BTKEY_UP;
            case 66: return BTKEY_DOWN;
            case 67: return BTKEY_RIGHT;
            case 68: return BTKEY_LEFT;
            default: return BTKEY_UNKNOWN;
         }
      default:
         return BTKEY_UNKNOWN;
   }
}

void drawMenu(WINDOW *win, int starty, int startx, const std::string& title, const std::vector<std::string>& labels, const std::vector<bool>& checked, size_t activeIndex) {
	mvwprintw(win, starty,startx, title.c_str());

	for (size_t i = 0; i < labels.size(); ++i) {
		if (i == activeIndex) {
			wstandout(win);
		}
		mvwprintw(win, starty+i+1,startx, "   %s", labels[i].c_str());
		if (checked[i]) {
			mvwaddch(win, starty+i+1,startx+1, 'X' | A_BOLD);
		}
		if (i == activeIndex) {
			wstandend(win);
		}
	}
}

void board(WINDOW *win, int starty, int startx, int lines, int cols, int tile_width, int tile_height)
{
	int endy, endx, i, j;

	endy = starty + lines * tile_height;
	endx = startx + cols  * tile_width;

	for(j = starty; j <= endy; j += tile_height)
		for(i = startx; i <= endx; ++i)
			mvwaddch(win, j, i, ACS_HLINE);
	for(i = startx; i <= endx; i += tile_width)
		for(j = starty; j <= endy; ++j)
			mvwaddch(win, j, i, ACS_VLINE);
	mvwaddch(win, starty, startx, ACS_ULCORNER);
	mvwaddch(win, endy, startx, ACS_LLCORNER);
	mvwaddch(win, starty, endx, ACS_URCORNER);
	mvwaddch(win, 	endy, endx, ACS_LRCORNER);
	for(j = starty + tile_height; j <= endy - tile_height; j += tile_height)
	{	mvwaddch(win, j, startx, ACS_LTEE);
		mvwaddch(win, j, endx, ACS_RTEE);
		for(i = startx + tile_width; i <= endx - tile_width; i += tile_width)
			mvwaddch(win, j, i, ACS_PLUS);
	}
	for(i = startx + tile_width; i <= endx - tile_width; i += tile_width)
	{	mvwaddch(win, starty, i, ACS_TTEE);
		mvwaddch(win, endy, i, ACS_BTEE);
	}
//	wrefresh(win);
}

void graphCell(WINDOW *win, int starty, int startx, double pressure) {
    int i, chunk;
    char c;

    int value = (int)(pressure * 256.0) / 102;  // integer division
    for (i = 4; i >= 0; --i) {
        chunk = (value <= 7) ? value : 7;
        value -= chunk;

        switch (chunk) {
        default:  c = '#'; break;
        case 2:   c = '~'; break;
        case 1:   c = '-'; break;
        case 0:   c = '_'; break;
        }
        mvwprintw(win, starty+1, startx+i, "%c", c);

        switch (chunk-4) {
        case 3:   c = '#'; break;
        case 2:   c = '~'; break;
        case 1:   c = '-'; break;
        case 0:   c = '_'; break;
        default:  c = ' '; break;
        }
        mvwprintw(win, starty, startx+i, "%c", c);
    }

//    wrefresh(win);
}

void graphPressures(WINDOW *win, int starty, int startx, const TactilePuck::v_type& pressures) {
	for (int i = 0; i < pressures.size(); ++i) {
	    graphCell(win, starty+1 + (7 - (i/3 /* integer division */))*HEIGHT, startx+1 + (i%3)*WIDTH, pressures[i]);
	}
}

int puckTemp[Hand::DOF];
int motorTemp[Hand::DOF];
void runDisplay(bool& going, ProductManager& pm)
{
	initscr();
	curs_set(0);
	noecho();
	timeout(0);

	Hand* hand = pm.getHand();
	std::vector<TactilePuck*> tps;
	if (hand != NULL) {
		tps = hand->getTactilePucks();
		for (size_t i = 0; i < tps.size(); ++i) {
			board(stdscr, 0,i*(3*WIDTH+2), 8,3, WIDTH,HEIGHT);
		}
	}

	mvprintw(26,0, "Hand");
	mvprintw(27,0, "  Update-rate (Hz):");
	mvprintw(28,0, "    Inner Position:");
	mvprintw(29,0, "    Outer Position:");
	mvprintw(30,0, "     Strain gauges:");
	mvprintw(31,0, "     Puck temp (C):");
	mvprintw(32,0, "    Motor temp (C):");

	mvprintw(26,60, "F/T Sensor");
	mvprintw(27,60, "  Update-rate (Hz):");

	g_DisplayActive = true;
	std::stringstream ss;
	ss.precision(3);
	ss.setf(std::ios::fixed, std::ios::floatfield);

	while (going) {
		btkey c = btkey_get();
		if (c != BTKEY_NOKEY) {
			switch (c) {
			case 'q':
			case 'x':
				going = false;
				break;

			default:
				break;
			}
		}

		if (hand != NULL) {
			for (size_t i = 0; i < tps.size(); ++i) {
				graphPressures(stdscr, 0,i*(3*WIDTH+2), tps[i]->getFullData());
			}

			ss.str("");
			ss << hand->getInnerLinkPosition();
			mvprintw(28,20, "%-40s", ss.str().c_str());
			ss.str("");
			ss << hand->getOuterLinkPosition();
			mvprintw(29,20, "%-40s", ss.str().c_str());

			mvprintw(30,20, "[%4d, %4d, %4d, %4d]", hand->getStrain()[0], hand->getStrain()[1], hand->getStrain()[2], hand->getStrain()[3]);
			mvprintw(31,20, "[%3d, %3d, %3d, %3d]", puckTemp[0], puckTemp[1], puckTemp[2], puckTemp[3]);
			mvprintw(32,20, "[%3d, %3d, %3d, %3d]", motorTemp[0], motorTemp[1], motorTemp[2], motorTemp[3]);
		}

		refresh();
		usleep(100000);
	}
	g_DisplayActive = false;

	usleep(250000);
	endwin();
}


class RatePrinter {
public:
	RatePrinter(int _y, int _x) :
		y(_y), x(_x)
	{
		now = rt_timer_read();
		lastUpdate = now;
	}

	void update() {
		now = rt_timer_read();
		if (g_DisplayActive) {
			mvprintw(y,x, "%6.2f", 1e9 / ((double) now - lastUpdate));
		}
		lastUpdate = now;
	}

	int y;
	int x;

protected:
	RTIME now;
	RTIME lastUpdate;
};


template<size_t DOF>
void moveWam(const bool& going, ProductManager& bm, systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	bm.getSafetyModule()->waitForMode(SafetyModule::ACTIVE, false);

	systems::ExposedOutput<jp_type> eo;
	eo.setValue(wam.getJointPositions());
	wam.trackReferenceSignal(eo.output);

	while (going) {
		sleep(1);
	}
}

void moveHand(const bool& going, Hand& hand) {
	Hand::jp_type tmp;
	std::vector<Hand::jp_type> pos;

	tmp.setZero();
	pos.push_back(tmp);

	tmp.setConstant(M_PI);
	tmp[3] = 0;
	pos.push_back(tmp);

	tmp.setZero();
	pos.push_back(tmp);

	tmp.setZero();
	tmp[3] = M_PI;
	pos.push_back(tmp);

	tmp.setConstant(M_PI);
	pos.push_back(tmp);

	tmp.setZero();
	tmp[3] = M_PI;
	pos.push_back(tmp);


	while (going) {
		for (size_t i = 0; i < pos.size()  &&  going; ++i) {
			sleep(3);
//			std::cout << pos[i] << "\n";
			hand.trapezoidalMove(pos[i], false);
		}
	}

	hand.idle();
}

void readHand(const bool& going, Hand& hand) {
	RatePrinter rp(27,20);

	while (going) {
//		usleep(100);
		hand.updatePosition();
		hand.updateStrain();
		hand.updateTactFull();

		hand.getGroup().getProperty(Puck::TEMP, puckTemp);
		hand.getGroup().getProperty(Puck::THERM, motorTemp);
//		std::cout << hand.getPosition() << " [" << hand.getStrain()[0] << "," << hand.getStrain()[1] << "," << hand.getStrain()[2] << "," << hand.getStrain()[3] << "]" << "\n";

		rp.update();
	}
}

void readFTS(const bool& going, ForceTorqueSensor& fts) {
	RatePrinter rp(27,80);

	while (going) {
//		usleep(100);
		fts.update();

		rp.update();
	}
}


int main(int argc, char** argv) {
	//rt_print_auto_init(1);

	// Give us pretty stack-traces when things die
	installExceptionHandler();

	ProductManager pm;
	pm.wakeAllPucks();

	bool going = true;
	std::vector<boost::thread*> threads;


	// Hand
	if (pm.foundHand()) {
		printf("Starting Hand...\n");
		Hand& hand = *pm.getHand();

		printf(">>> Press [Enter] to initialize Hand. (Make sure it has room!)");
		waitForEnter();
		hand.initialize();

		threads.push_back(new boost::thread(moveHand, ref(going), ref(hand)));
		threads.push_back(new boost::thread(readHand, ref(going), ref(hand)));
	} else {
		printf(">>> WARNING: No Hand found\n");
	}

	// F/T
	if (pm.foundForceTorqueSensor()) {
		printf("Starting F/T Sensor...\n");
		threads.push_back(new boost::thread(readFTS, ref(going), ref(*pm.getForceTorqueSensor())));
	} else {
		printf(">>> WARNING: No F/T Sensor found\n");
	}

	// WAM
	if (pm.foundWam4()) {
		printf("Starting WAM4...\n");
		threads.push_back(new boost::thread(moveWam<4>, ref(going), ref(pm), ref(*pm.getWam4(false))));
	} else if (pm.foundWam7()) {
		printf("Starting WAM7...\n");
		threads.push_back(new boost::thread(moveWam<7>, ref(going), ref(pm), ref(*pm.getWam7(false))));
	} else {
		printf(">>> WARNING: No WAM found\n");
	}


	runDisplay(going, pm);
//	printf(">>> Press [Enter] to exit.");
//	waitForEnter();



	// clean up
	printf("Cleaning up...\n");
	going = false;
	for (size_t i = 0; i < threads.size(); ++i) {
		threads[i]->join();
	}
}
