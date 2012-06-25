/*
 * ex11_master_master.cpp
 *
 *  Created on: Feb 22, 2010
 *      Author: Christopher Dellin
 *      Author: Dan Cody
 *      Author: Brian Zenowich
 */

#include <iostream>
#include <string>
#include <vector>

#include <boost/thread.hpp>

#include <barrett/os.h>
#include <barrett/detail/stl_utils.h>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>

#define BARRETT_SMF_VALIDATE_ARGS
#include <barrett/standard_main_function.h>

#include "ex11_master_master.h"


using namespace barrett;
using detail::waitForEnter;


void ghcEntryPoint(GimbalsHandController* ghc, const char* remoteHost);
void handEntryPoint(Hand* hand, const char* remoteHost);


bool validate_args(int argc, char** argv) {
	if (argc != 2  &&  argc != 3) {
		printf("Usage: %s <remoteHost> [--auto]\n", argv[0]);
		printf("  --auto : Automatically link WAMs and start Hand or Gimbals Hand Controller thread\n");

		return false;
	}

	return true;
}


template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);


	const jp_type SYNC_POS(0.0);  // the position each WAM should move to before linking


	MasterMaster<DOF> mm(pm.getExecutionManager(), argv[1]);
	systems::connect(wam.jpOutput, mm.input);


	wam.gravityCompensate();


	std::vector<std::string> autoCmds;
	if (argc == 3) {  // auto init
		if (pm.foundGimbalsHandController()) {
			autoCmds.push_back("g");
		}
		if (pm.foundHand()) {
			autoCmds.push_back("h");
		}
		autoCmds.push_back("l");
	}

	boost::thread* ghcThread = NULL;
	boost::thread* handThread = NULL;
	std::string line;
	v_type gainTmp;
	while (true) {
		if (autoCmds.empty()) {
			printf(">>> ");
			std::getline(std::cin, line);
		} else {
			line = autoCmds.back();
			autoCmds.pop_back();
		}

		switch (line[0]) {
		case 'l':
			if (mm.isLinked()) {
				mm.unlink();
			} else {
				wam.moveTo(SYNC_POS);

				printf("Press [Enter] to link with the other WAM.");
				waitForEnter();
				mm.tryLink();
				wam.trackReferenceSignal(mm.output);

				btsleep(0.1);  // wait an execution cycle or two
				if (mm.isLinked()) {
					printf("Linked with remote WAM.\n");
				} else {
					printf("WARNING: Linking was unsuccessful.\n");
				}
			}

			break;

		case 'g':
			if (ghcThread == NULL) {
				if (pm.foundGimbalsHandController()) {
					ghcThread = new boost::thread(ghcEntryPoint, pm.getGimbalsHandController(), argv[1]);
					printf("Started Gimbals Hand Controller thread.\n");
				} else {
					printf("WARNING: No Gimbals Hand Controller found.\n");
				}
			} else {
				ghcThread->interrupt();
				ghcThread->join();
				delete ghcThread;
				ghcThread = NULL;
				printf("Stopped Gimbals Hand Controller thread.\n");
			}

			break;

		case 'h':
			if (handThread == NULL) {
				if (pm.foundHand()) {
					Hand* hand = pm.getHand();

					printf("Press [Enter] to initialize Hand. (Make sure it has room!)");
					waitForEnter();
					hand->initialize();

					handThread = new boost::thread(handEntryPoint, hand, argv[1]);
					printf("Started Hand thread.\n");
				} else {
					printf("WARNING: No Hand found.\n");
				}
			} else {
				handThread->interrupt();
				handThread->join();
				delete handThread;
				handThread = NULL;
				printf("Stopped Hand thread.\n");
			}

			break;

		case 't':
			size_t jointIndex;
			{
				size_t jointNumber;
				std::cout << "\tJoint: ";
				std::cin >> jointNumber;
				jointIndex = jointNumber - 1;

				if (jointIndex >= DOF) {
					std::cout << "\tBad joint number: " << jointNumber;
					break;
				}
			}

			char gainId;
			std::cout << "\tGain identifier (p, i, or d): ";
			std::cin >> line;
			gainId = line[0];

			std::cout << "\tCurrent value: ";
			switch (gainId) {
			case 'p':
				gainTmp = wam.jpController.getKp();
				break;
			case 'i':
				gainTmp = wam.jpController.getKi();
				break;
			case 'd':
				gainTmp = wam.jpController.getKd();
				break;

			default:
				std::cout << "\tBad gain identifier.";
			}
			std::cout << gainTmp[jointIndex] << std::endl;

			std::cout << "\tNew value: ";
			std::cin >> gainTmp[jointIndex];
			switch (gainId) {
			case 'p':
				wam.jpController.setKp(gainTmp);
				break;
			case 'i':
				wam.jpController.setKi(gainTmp);
				break;
			case 'd':
				wam.jpController.setKd(gainTmp);
				break;

			default:
				std::cout << "\tBad gain identifier.";
			}

			break;
			
		default:
			printf("\n");
			printf("    'l' to toggle linking with other WAM\n");
			printf("    'h' to toggle Hand thread\n");
			printf("    'g' to toggle Gimbals Hand Controller thread\n");
			printf("    't' to tune control gains\n");

			break;
		}
	}


	delete ghcThread;
	delete handThread;

	return 0;
}


int openSocket(const char* remoteHost, int port = 3333) {
	int sock;

	int err;
	long flags;
	struct sockaddr_in bind_addr;
	struct sockaddr_in their_addr;

	/* Create socket */
	sock = socket(PF_INET, SOCK_DGRAM, 0);
	if (sock == -1)
	{
		(logMessage("openSocket(): Failed.  %s: Could not create socket.")  % __func__).raise<std::runtime_error>();
	}

	/* Set socket to non-blocking, set flag associated with open file */
	flags = fcntl(sock, F_GETFL, 0);
	if (flags < 0)
	{
		(logMessage("openSocket(): Failed.  %s: Could not get socket flags.")  % __func__).raise<std::runtime_error>();
	}
	flags |= O_NONBLOCK;
	err = fcntl(sock, F_SETFL, flags);
	if (err < 0)
	{
		(logMessage("openSocket(): Failed.  %s: Could not set socket flags.")  % __func__).raise<std::runtime_error>();
	}

	/* Set up the bind address */
	bind_addr.sin_family = AF_INET;
	bind_addr.sin_port = htons(port);
	bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	err = bind(sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr));
	if (err == -1)
	{
		(logMessage("openSocket(): Failed.  %s: Could not bind to socket on port %d.")  % __func__  % port).raise<std::runtime_error>();
	}

	/* Set up the other guy's address */
	their_addr.sin_family = AF_INET;
	their_addr.sin_port = htons(port);
	err = ! inet_pton(AF_INET, remoteHost, &their_addr.sin_addr);
	if (err)
	{
		(logMessage("openSocket(): Failed.  %s: Bad IP argument '%s'.")  % __func__  % remoteHost).raise<std::runtime_error>();
	}

	/* Call "connect" to set datagram destination */
	err = connect(sock, (struct sockaddr *)&their_addr, sizeof(struct sockaddr));
	if (err)
	{
		(logMessage("openSocket(): Failed.  %s: Could not set datagram destination.")  % __func__).raise<std::runtime_error>();
	}

	return sock;
}

void ghcEntryPoint(GimbalsHandController* ghc, const char* remoteHost) {
	static const int SIZE_OF_MSG = sizeof(unsigned char);

	int sock = openSocket(remoteHost);
	unsigned char data = 0, data_1 = 0;

	while ( !boost::this_thread::interruption_requested() ) {
		ghc->update();

		data =	(ghc->getPointerOpen() << 0) | (ghc->getPointerClose() << 1) |
				(ghc->getMiddleOpen() << 2) | (ghc->getMiddleClose() << 3) |
				(ghc->getThumbOpen() << 4) | (ghc->getThumbClose() << 5) |
				(ghc->getRockerUp() << 6) | (ghc->getRockerDown() << 7);
		send(sock, &data, SIZE_OF_MSG, 0);

		if (data != data_1) {
			printf("%d,%d  %d,%d  %d,%d  %d,%d  %f\n",
					ghc->getThumbOpen(), ghc->getThumbClose(),
					ghc->getPointerOpen(), ghc->getPointerClose(),
					ghc->getMiddleOpen(), ghc->getMiddleClose(),
					ghc->getRockerUp(), ghc->getRockerDown(),
					ghc->getKnob());
			data_1 = data;
		}

		btsleep(0.01);
	}

	close(sock);
}


double velCommand(bool open, bool close, double speed = 1.25) {
	if (open  &&  !close) {
		return -speed;
	} else if (close  &&  !open) {
		return speed;
	} else {
		return 0.0;
	}
}

void handEntryPoint(Hand* hand, const char* remoteHost) {
	static const int SIZE_OF_MSG = sizeof(unsigned char);

	int sock = openSocket(remoteHost);
	unsigned char data = 0, data_1 = 0;
	Hand::jv_type hjv;

	const int MAX_MISSED = 50;
	int numMissed = MAX_MISSED;

	while (!boost::this_thread::interruption_requested()) {
		// Get any packets in the buffer
		if (numMissed <= MAX_MISSED) {  // Keep numMissed from wrapping.
			++numMissed;
		}
		while (recv(sock, &data, SIZE_OF_MSG, 0) == SIZE_OF_MSG) {
			numMissed = 0;
		}

		// If things havn't changed, just wait and loop
		if (numMissed  ||  data == data_1) {
			// If we havn't seen a message in a while, stop the Hand.
			if (numMissed == MAX_MISSED) {
				printf("Sending stop command to hand.\n");
				hand->idle();
			}
		} else {
			hjv[0] = velCommand(data & (1<<2), data & (1<<3));  // Middle
			hjv[1] = velCommand(data & (1<<0), data & (1<<1));  // Pointer
			hjv[2] = velCommand(data & (1<<4), data & (1<<5));  // Thumb
			hjv[3] = velCommand(data & (1<<6), data & (1<<7));  // Rocker
			hand->velocityMove(hjv);
			std::cout << "Velocity: " << hjv << std::endl;

			data_1 = data;
		}

		btsleep(0.01);
	}

	close(sock);
}
