/**
 * @file mirror_image.cpp
 *
 * @date 02/22/10
 * @author Christopher Dellin
 * @author Dan Cody
 * @author Brian Zenowich
 */
/**
 * Revision History
 * 07/31/14 - JH - Altered to Work for Proficio and Added Exit Option.
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

#include "mirror_image.h"


using namespace barrett;
using detail::waitForEnter;

/* Validate CommandLine Arguments */
bool validate_args(int argc, char** argv) {
	if (argc != 2  &&  argc != 3) {
		printf("Usage: %s <remoteHost> [--auto]\n", argv[0]);
		printf("  --auto : Automatically link WAMs and start Hand or Gimbals Hand Controller thread\n");

		return false;
	}

	return true;
}

/* Main Function */
template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam) {

	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	jp_type SYNC_POS(wam.getHomePosition());  // the position each WAM should move to before linking
	SYNC_POS(0) = 0.0; SYNC_POS(1) = 0.0;

	MasterMaster<DOF> mm(pm.getExecutionManager(), argv[1]);
	systems::connect(wam.jpOutput, mm.input);

	wam.gravityCompensate();

	std::vector<std::string> autoCmds;
	autoCmds.push_back("l");


	std::string line, qV;
	v_type gainTmp;
	bool active = true;
	while (active) {
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

				printf("Press [Enter] to link with the other Proficio.");
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
		case'q':
			printf("Quiting and moving Home.\n");
			if(mm.unlink()){
				printf("Did You Unlink the 2nd Proficio?[y/n]\n");
				std::cin >> qV;
				switch(qV[0]){
					case'y':
					case'Y':
						active = false;
						wam.moveHome();
						break;
					case'n':
					case'N':
						break;
					default:
						break;
				}
			}
			break;
			
		default:
			printf("\n");
			printf("    'l' to toggle linking with other Proficio\n");
			printf("    't' to tune control gains\n");
			printf("    'q' to quit\n");
			break;
		}
	}

	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
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

double velCommand(bool open, bool close, double speed = 1.25) {
	if (open  &&  !close) {
		return -speed;
	} else if (close  &&  !open) {
		return speed;
	} else {
		return 0.0;
	}
}
