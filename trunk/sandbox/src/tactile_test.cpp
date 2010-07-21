/*
 * tactile_test.cpp
 *
 *  Created on: Jun 15, 2010
 *      Author: dc
 */

#include <iostream>
#include <string>
#include <cstdlib>
//#include <stdlib.h>

#include <sys/mman.h>
#include <unistd.h>
#include <native/task.h>

#include <curses.h>

#include <barrett/cdlbt/bus/bus.h>
#include <barrett/cdlbt/bus/bus_can.h>


const int HAND_DOF = 4;
#define WIDTH  6
#define HEIGHT 3


//void waitForEnter() {
//	static std::string line;
//	std::getline(std::cin, line);
//}

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
/*
    int i, chunk;
    char c;
    for (i = 3; i >= 0; --i) {
        chunk = value & 7;
        value >>= 3;

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
 */

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

    graphCell(win, starty+1 + (7 - (i/3 /* integer division */))*HEIGHT, startx+1 + (i%3)*WIDTH, ((int)(d[0]&0x0f))<<8 | d[1]);
    ++i;
    graphCell(win, starty+1 + (7 - (i/3 /* integer division */))*HEIGHT, startx+1 + (i%3)*WIDTH, (((int)d[2])<<4) | (d[3]>>4));
    ++i;
    graphCell(win, starty+1 + (7 - (i/3 /* integer division */))*HEIGHT, startx+1 + (i%3)*WIDTH, ((int)(d[3]&0x0f))<<8 | d[4]);
    ++i;
    graphCell(win, starty+1 + (7 - (i/3 /* integer division */))*HEIGHT, startx+1 + (i%3)*WIDTH, (((int)d[5])<<4) | (d[6]>>4));
    ++i;

    if (i < 24) {
        graphCell(win, starty+1 + (7 - (i/3 /* integer division */))*HEIGHT, startx+1 + (i%3)*WIDTH, ((int)(d[6]&0x0f))<<8 | d[7]);
    }
}



int main(int argc, char** argv) {
	int osTact = 0, numPucks = 0;
	long tactTop10[HAND_DOF];
	int id, property;
	long value1, value2;
	unsigned char data[8];
	bool going = true;


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
//	waitForEnter();

	printf("Waking hand pucks ...\n");

	for(int i = 11; i <= 14; i++) {
		bt_bus_can_set_property(dev, i, 5, 2); // Set STAT to STATUS_READY
	}
	usleep((long)5e5);

	printf("Initializing tactile sensors...\n");
	for(int i = 11; i <= 14; i++) {
		bt_bus_can_async_get_property(dev, i, 5);
		usleep(1000);
		switch (bt_bus_can_async_read(dev, &id, &property, &value1, NULL, NULL, 0, 1)) {
		case 0:
			++numPucks;
			printf("Found Puck %d.\n", i);

			if (value1 == 0) {
				bt_bus_can_set_property(dev, i, 5, 2); // Set STAT to STATUS_READY
				usleep((long)5e5);
			}

			bt_bus_can_get_property(dev, i, 106, &value1, NULL, 1);
			if (value1 != 0) {
				printf("Failed to init tactile sensors on ID=%d. Reported %ld sensors.\n", i, value1);
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

	printf(" ... done.\n");


	initscr();
	curs_set(0);
	noecho();
	timeout(0);
    board(stdscr, 0,0, 8,3, WIDTH,HEIGHT);

	while (true) {
		usleep(100000);
		if (getch() != ERR) {
			break;
		}
//		waitForEnter();

		bt_bus_can_set_property(dev, BT_BUS_CAN_GROUPID(HAND_GRP), 106, 2);
		osTact = 5*numPucks;  // 5 messages are returned for full TACT data
		while (osTact--) {
			bt_bus_can_async_read(dev, &id, NULL, &value1, NULL, data, 1, 1);
		    graphMessage(stdscr, 0,0, data);
//			printf("0x%.3x :", id);
//			for (int i = 0; i < value1; ++i) {
//				printf(" %.2x", data[i]);
//			}
//			printf("\n");
		}
	}

//	osTact = numPucks;
//	bt_bus_can_set_property(dev, BT_BUS_CAN_GROUPID(HAND_GRP), 106, 1);
//
//	while (going) {
//		bt_bus_can_async_read(dev, &id, &property, &value1, &value2, NULL, 1, 1);
//		switch (id) {
//		// hand
//		case 11:
//		case 12:
//		case 13:
//		case 14:
//			switch (property) {
//			case 106:  // tact
//				tactTop10[id-11] = (value1 & 0xFFFFFF00) >> 8;
//
//				if (--osTact == 0) {
//					printf("%06lX %06lX %06lX %06lX\n", tactTop10[0], tactTop10[1], tactTop10[2], tactTop10[3]);
//
//					waitForEnter();
//
//					bt_bus_can_set_property(dev, BT_BUS_CAN_GROUPID(HAND_GRP), 106, 1);
//					osTact = numPucks;
//				}
//				break;
//
//			default:
//				printf("%s: Spurious CAN message (ID=%d, PROPERTY=%d, VALUE=%ld)\n", __func__, id, property, value1);
//			}
//			break;
//
//		default:
//			printf("%s: Spurious CAN message (ID=%d, PROPERTY=%d, VALUE=%ld)\n", __func__, id, property, value1);
//		}
//	}

	endwin();

	return 0;
}
