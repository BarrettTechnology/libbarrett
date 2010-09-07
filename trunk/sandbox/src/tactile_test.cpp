/*
 * tactile_test.cpp
 *
 *  Created on: Jun 15, 2010
 *      Author: dc
 */

#include <iostream>
#include <vector>
#include <string>
#include <cstdlib>

#include <sys/mman.h>
#include <unistd.h>
#include <native/task.h>

#include <curses.h>

#include <barrett/cdlbt/bus/bus.h>
#include <barrett/cdlbt/bus/bus_can.h>


const int HAND_DOF = 4;
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

void waitForEnter() {
	static std::string line;
	std::getline(std::cin, line);
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
	wrefresh(win);
}

void graphCell(WINDOW *win, int starty, int startx, int value) {
    int i, chunk;
    char c;

    value /= 102;  // integer division
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

    wrefresh(win);
}

void graphMessage(WINDOW *win, int starty, int startx, unsigned char * d) {  // data is 8 bytes wide
    int i = (d[0] >> 4) * 5;

    graphCell(win, starty+1 + (7 - (i/3 /* integer division */))*HEIGHT, startx+1 + (i%3)*WIDTH, (((int)d[0]&0x000F)<<8) | ((int)d[1]&0x00FF));
    ++i;
    graphCell(win, starty+1 + (7 - (i/3 /* integer division */))*HEIGHT, startx+1 + (i%3)*WIDTH, ((int)d[2]&0x00FF)<<4 | ((int)d[3]&0x00F0)>>4);
    ++i;
    graphCell(win, starty+1 + (7 - (i/3 /* integer division */))*HEIGHT, startx+1 + (i%3)*WIDTH, ((int)d[3]&0x000F)<<8 | (int)d[4]&0x00FF);
    ++i;
    graphCell(win, starty+1 + (7 - (i/3 /* integer division */))*HEIGHT, startx+1 + (i%3)*WIDTH, ((int)d[5]&0x00FF)<<4 | ((int)d[6]&0x00F0)>>4);
    ++i;

    if (i < 24) {
        graphCell(win, starty+1 + (7 - (i/3 /* integer division */))*HEIGHT, startx+1 + (i%3)*WIDTH, ((int)d[6]&0x000F)<<8 | (int)d[7]&0x00FF);
    }
}



int main(int argc, char** argv) {
	int osTact = 0, numPucks = 0;
	int id, property;
	long value1;
	unsigned char data[8];


	mlockall(MCL_CURRENT|MCL_FUTURE);
	rt_task_shadow(new RT_TASK, NULL, 10, 0);

	int port = 0;
	switch (argc) {
	case 1:
		printf("No port argument given. Using default.\n");
		break;
	case 2:
		port = atoi(argv[1]);
		break;
	default:
		printf("ERROR: Expected 1 or 0 arguments.\n");
		return -1;
		break;
	}

	printf("Using CAN bus port %d.\n", port);
	bt_bus_can* dev = NULL;
	if (bt_bus_can_create(&dev, port)) {
		printf("Couldn't open the CAN bus.\n");
		return -1;
	}

	printf(">>> Make sure the Hand is attached and powered.\n");

	printf("Waking hand pucks ...\n");

	for(int i = 11; i <= 14; i++) {
		bt_bus_can_set_property(dev, i, 5, 2);  // Set STAT to STATUS_READY
	}
	usleep((long)1e6);

	printf("Initializing tactile sensors...\n");
	for(int i = 11; i <= 14; i++) {
		bt_bus_can_async_get_property(dev, i, 5);
		usleep(1000);
		switch (bt_bus_can_async_read(dev, &id, &property, &value1, NULL, NULL, 0, 1)) {
		case 0:
			++numPucks;
			printf("Found Puck %d.\n", i);

			if (value1 == 0) {
				bt_bus_can_set_property(dev, i, 5, 2);  // Set STAT to STATUS_READY
				usleep((long)1e6);
			}

			bt_bus_can_get_property(dev, i, 107, &value1, NULL, 1);
			if (value1 == -2) {
				printf("Failed to init tactile sensors on ID=%d.\n", i);
			}

			break;

		case 3:  // the puck was not found
			continue;
			break;

		default:
			printf("CAN read error.\n");
			break;
		}
	}

	printf(" ... done.\n\n");


	int streamId = 0;
	printf("Stream TACT data from ID = ");
	std::cin >> streamId;
	waitForEnter();


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

	while (true) {
		btkey c = btkey_get();
		if (c != BTKEY_NOKEY) {
			if (c == BTKEY_UP  &&  activeIndex != 0) {
				--activeIndex;
			} else if (c == BTKEY_DOWN  &&  activeIndex != (numMenuItems - 1)) {
				++activeIndex;
			} else if (c == BTKEY_ENTER) {
				if (activeIndex == (numMenuItems - 1)) {
					break;
				} else {
					checked[activeIndex] = !checked[activeIndex];
					if (checked[activeIndex]) {
						bt_bus_can_set_property(dev, activeIndex+11, 78, 0);  // Set TSTOP to 0
						bt_bus_can_set_property(dev, activeIndex+11, 8, 2);  // Set MODE to TORQUE
					} else {
						bt_bus_can_set_property(dev, activeIndex+11, 8, 0);  // Set MODE to IDLE
					}
				}
			}

		    drawMenu(stdscr, 5,20, title, labels, checked, activeIndex);
		}

//		// wait for key press
//		while (getch() == ERR) {
//			usleep(100000);
//		}

		bt_bus_can_set_property(dev, streamId, 106, 2);
		usleep(100000);
		osTact = 5;  // 5 messages are returned for full TACT data
		while (osTact--) {
			bt_bus_can_async_read(dev, &id, NULL, &value1, NULL, data, 0, 1);
		    graphMessage(stdscr, 0,0, data);
		}
	}

	endwin();

	for (int i = 0; i < HAND_DOF; ++i) {
		if (checked[i]) {
			bt_bus_can_set_property(dev, i+11, 8, 0);  // Set MODE to IDLE
		}
	}

	return 0;
}
