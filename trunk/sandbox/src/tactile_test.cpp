/*
 * tactile_test.cpp
 *
 *  Created on: Jun 15, 2010
 *      Author: dc
 */

#include <iostream>
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


void waitForEnter() {
	static std::string line;
	std::getline(std::cin, line);
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
		bt_bus_can_set_property(dev, i, 5, 2); // Set STAT to STATUS_READY
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
				bt_bus_can_set_property(dev, i, 5, 2); // Set STAT to STATUS_READY
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

	while (true) {
		// exit on key press
		usleep(100000);
		if (getch() != ERR) {
			break;
		}

//		// wait for key press
//		while (getch() == ERR) {
//			usleep(100000);
//		}

		bt_bus_can_set_property(dev, streamId, 106, 2);
		osTact = 5;  // 5 messages are returned for full TACT data
		while (osTact--) {
			bt_bus_can_async_read(dev, &id, NULL, &value1, NULL, data, 1, 1);
		    graphMessage(stdscr, 0,0, data);
		}
	}

	endwin();

	return 0;
}
