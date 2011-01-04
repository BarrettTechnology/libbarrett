/*
 * can_terminal.cpp
 *
 *  Created on: Aug 19, 2010
 *      Author: dc
 */

#include <iostream>
#include <string>
#include <cstdio>

#include <unistd.h>

#include <boost/thread.hpp>
#include <barrett/bus/can_socket.h>


using namespace barrett;


void readThread(const bus::CANSocket* bus, const bool* going) {
	int ret;
	int id;
	unsigned char data[bus::CANSocket::MAX_MESSAGE_LEN];
	size_t len;

	while (*going) {
		ret  = bus->receiveRaw(id, data, len, false);
		if (ret == 0) {  // success
			printf("(0x%03x)", id);
			for (size_t i = 0; i < len; ++i) {
				printf(" %02x", data[i]);
			}
			printf("\n");
		} else if (ret != 1) {  // error other than no data
			printf("ERROR: bus::CANSocket::receive() returned %d.\n", ret);
		}

		usleep(100000);
	}
}


int main(int argc, char** argv) {
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
	bus::CANSocket bus(port);

	bool going = true;
	boost::thread thread(readThread, &bus, &going);

	std::string line;
	int ret;
	int id;
	unsigned char data[bus::CANSocket::MAX_MESSAGE_LEN];
	size_t len;
	while (going) {
		printf(">> ");
		std::getline(std::cin, line);
		len = sscanf(line.c_str(), "%3x %2x %2x %2x %2x %2x %2x %2x %2x", &id,
				(unsigned int*)(data+0), (unsigned int*)(data+1), (unsigned int*)(data+2), (unsigned int*)(data+3),
				(unsigned int*)(data+4), (unsigned int*)(data+5), (unsigned int*)(data+6), (unsigned int*)(data+7)) - 1;
		if (len < 0  ||  len > 8) {
			printf("ERROR: Input format. No message sent.\n");
			continue;
		}

		ret = bus.send(id, data, len);
		if (ret) {
			printf("ERROR: bus::CANSocket::send() returned %d.\n", ret);
		}
	}

	going = false;
	thread.join();

	return 0;
}
