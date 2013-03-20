/*
 * tactile_test.cpp
 *
 *  Created on: Jun 15, 2010
 *      Author: dc
 */

#include <iostream>

#include <curses.h>

#include <barrett/detail/stl_utils.h>
#include <barrett/products/product_manager.h>


using namespace barrett;
using detail::waitForEnter;

const double THRESH = 8.0;
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


int main(int argc, char** argv) {
	Puck* puck;
	TactilePuck tactPuck;
	ProductManager pm;


	printf("Found Hand Pucks:\n");
	for (size_t i = 0; i < pm.getHandPucks().size(); ++i) {
		if (pm.getHandPucks()[i] != NULL) {
			printf("  ID = %d\n", pm.getHandPucks()[i]->getId());
		}
	}
	printf("\n");
	Puck::wake(pm.getHandPucks());

	size_t streamId = 0;
	printf("Stream TACT data from ID = ");
	std::cin >> streamId;
	waitForEnter();
	puck = pm.getPuck(streamId);
	if (puck == NULL) {
		printf("ERROR: Puck %zu was not found on the bus.\n", streamId);
		return 1;
	}
	tactPuck.setPuck(puck);


	bool firstRun = true;
	int numBadTransitions = 0;
	const TactilePuck::v_type& pressures = tactPuck.getFullData();
	TactilePuck::v_type pressures_1;
	TactilePuck::v_type tmp;


	initscr();
	curs_set(0);
	noecho();
	timeout(0);
    board(stdscr, 0,0, 8,3, WIDTH,HEIGHT);

    const std::string title("Active motors:");
    const size_t numMenuItems = 5;
    std::vector<std::string> labels;
    labels.push_back("Puck 11");
    labels.push_back("Puck 12");
    labels.push_back("Puck 13");
    labels.push_back("Puck 14");
    labels.push_back("Exit");

    std::vector<bool> checked(numMenuItems, false);
    size_t activeIndex = 0;

    drawMenu(stdscr, 5,20, title, labels, checked, activeIndex);
    mvprintw(0,20, "Number of transitions larger than %f N*cm^-2:", THRESH);

	while (true) {
		btkey c = btkey_get();
		if (c != BTKEY_NOKEY) {
			if (c == BTKEY_UP  &&  activeIndex != 0) {
				--activeIndex;
			} else if (c == BTKEY_DOWN  &&  activeIndex != (numMenuItems - 1)) {
				++activeIndex;
			} else if (c == BTKEY_ENTER) {
				if (activeIndex == (numMenuItems - 1)) {  // exit
					break;
				} else {
					checked[activeIndex] = !checked[activeIndex];
					Puck* p = pm.getPuck(activeIndex + ProductManager::FIRST_HAND_ID);
					if (p != NULL) {
						if (checked[activeIndex]) {
							p->setProperty(Puck::TSTOP, 0);
							p->setProperty(Puck::MODE, MotorPuck::MODE_TORQUE);
						} else {
							p->setProperty(Puck::MODE, MotorPuck::MODE_IDLE);
						}
					}
				}
			}

		    drawMenu(stdscr, 5,20, title, labels, checked, activeIndex);
		}

//		// wait for key press
//		while (getch() == ERR) {
//			usleep(100000);
//		}

		tactPuck.updateFull();
		graphPressures(stdscr, 0,0, pressures);

		if (firstRun) {
			firstRun = false;
			pressures_1 = pressures;
		}
		tmp = pressures - pressures_1;
		for (int i = 0; i < tmp.size(); ++i) {
			if (abs(tmp[i] > THRESH)) {
				++numBadTransitions;
			}
		}
		mvprintw(1,23, "%d", numBadTransitions);
		pressures_1 = pressures;

		refresh();
	}

	endwin();

	for (size_t i = 0; i < pm.getHandPucks().size(); ++i) {
		if (pm.getHandPucks()[i] != NULL) {
			pm.getHandPucks()[i]->setProperty(Puck::MODE, MotorPuck::MODE_IDLE);
		}
	}

	return 0;
}
