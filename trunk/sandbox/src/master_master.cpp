/*
 * master_master.cpp
 *
 *  Created on: Feb 22, 2010
 *      Author: Christopher Dellin
 *      Author: Dan Cody
 *      Author: Brian Zenowich
 */


#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <string>

#include <sys/socket.h> /* For sockets */
#include <fcntl.h>      /* To change socket to nonblocking mode */
#include <arpa/inet.h>  /* For inet_aton() */
#include <unistd.h>  // usleep

#include <native/task.h>
#include <native/mutex.h>

#include <boost/ref.hpp>
#include <boost/bind.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/thread.hpp>
#include <libconfig.h++>

#include <barrett/exception.h>
#include <barrett/detail/debug.h>
#include <barrett/thread/abstract/mutex.h>
#include <barrett/math.h>
#include <barrett/units.h>
#include <barrett/log.h>
#include <barrett/systems.h>
#include <barrett/wam.h>

#include "master_master.h"


//namespace log = barrett::log;
namespace math = barrett::math;
namespace systems = barrett::systems;
namespace units = barrett::units;
using barrett::Wam;
using systems::connect;
using systems::reconnect;
using systems::disconnect;


//using boost::bind;
//using boost::ref;


#define b1(bits) ((bits >> 0) & 1)
#define b2(bits) ((bits >> 1) & 1)
#define b3(bits) ((bits >> 2) & 1)
#define b4(bits) ((bits >> 3) & 1)
#define b5(bits) ((bits >> 4) & 1)
#define b6(bits) ((bits >> 5) & 1)
#define b7(bits) ((bits >> 6) & 1)
#define b8(bits) ((bits >> 7) & 1)

//char motor[3] = {'1','2','3'};
int speed = -100;
int spreadspeed = -20;
int port = 3333;

const size_t J6_IDX = 5;
const double GIMBALS_J6_OFFSET = M_PI/2;

const size_t DOF = 4;
const double T_s = 0.002;

typedef Wam<DOF>::jt_type jt_type;
typedef Wam<DOF>::jp_type jp_type;
typedef Wam<DOF>::jv_type jv_type;


void waitForEnter() {
	static std::string line;
	std::getline(std::cin, line);
}

void handleHandCommands(struct bt_bus *bus, bool* going);


int main(int argc, char** argv) {
	// check arguments
	if (argc != 2  &&  argc != 3) {
		printf("Usage: %s <otherip> [-g]\n", argv[0]);
		return 0;
	}


	barrett::installExceptionHandler();  // give us pretty stack traces when things die


	bool isGimbals = (argc == 3);
	jp_type origin(0.0);
	if (isGimbals) {
		origin[J6_IDX] += GIMBALS_J6_OFFSET;
	}


	libconfig::Config config;
	config.readFile("/etc/wam/wam4-new.config");

	systems::RealTimeExecutionManager rtem(T_s, false);
	systems::System::defaultExecutionManager = &rtem;


    // instantiate Systems
	Wam<DOF> wam(config.lookup("wam"));

	systems::MasterMaster<DOF> mm(argv[1]);

	systems::Constant<double> j6Offset(GIMBALS_J6_OFFSET);
	systems::Summer<double> add;
	systems::ArraySplitter<jp_type> asAdd;
	systems::ArrayEditor<jp_type> aeAdd;

	std::bitset<2> polarity;
	polarity[0] = true;
	polarity[1] = false;
	systems::Summer<double> subtract(polarity);
//	systems::Summer<double> subtract("+-");
	systems::ArraySplitter<jp_type> asSubtract;
	systems::ArrayEditor<jp_type> aeSubtract;

	systems::FirstOrderFilter<jp_type> lpf1;
	systems::FirstOrderFilter<jp_type> lpf2;
	systems::Summer<jp_type, 2> sum;
	systems::Gain<jp_type, double, jp_type> half(0.5);


	lpf1.setLowPass(jp_type(300), jp_type(1));
	lpf2.setLowPass(jp_type(300), jp_type(1));


	// connect systems
	systems::System::Output<jp_type>* mmOutput;
	if (isGimbals) {
//		connect(wam.jpOutput, asSubtract.input);
//		connect(asSubtract.getOutput(J6_IDX), subtract.getInput(0));
//		connect(j6Offset.output, subtract.getInput(1));
//		connect(wam.jpOutput, aeSubtract.input);
//		connect(subtract.output, aeSubtract.getElementInput(J6_IDX));
//		connect(aeSubtract.output, mm.input);
//
//		connect(mm.output, asAdd.input);
//		connect(asAdd.getOutput(J6_IDX), add.getInput(0));
//		connect(j6Offset.output, add.getInput(1));
//		connect(mm.output, aeAdd.input);
//		connect(add.output, aeAdd.getElementInput(J6_IDX));
//		mmOutput = &aeAdd.output;

		connect(wam.jpOutput, asSubtract.input);
		connect(asSubtract.getOutput(J6_IDX), subtract.getInput(0));
		connect(j6Offset.output, subtract.getInput(1));
		connect(wam.jpOutput, aeSubtract.input);
		connect(subtract.output, aeSubtract.getElementInput(J6_IDX));
		connect(aeSubtract.output, mm.input);

		connect(mm.output, asAdd.input);
		connect(asAdd.getOutput(J6_IDX), add.getInput(0));
		connect(j6Offset.output, add.getInput(1));
		connect(mm.output, aeAdd.input);
		connect(add.output, aeAdd.getElementInput(J6_IDX));
		mmOutput = &aeAdd.output;
	} else {
		connect(wam.jpOutput, mm.input);
		mmOutput = &mm.output;
	}
	connect(*mmOutput, lpf1.input);
	connect(lpf1.output, lpf2.input);

	connect(wam.jpOutput, sum.getInput(0));
//	connect(*mmOutput, sum.getInput(1));
	connect(lpf2.output, sum.getInput(1));
	connect(sum.output, half.input);


	// start the main loop!
	rtem.start();

	std::cout << "Press [Enter] to compensate for gravity.\n";
	waitForEnter();
	wam.gravityCompensate();



	bool going = true;
	std::string line;

	boost::thread handThread;
	bool handThreadGoing = false;

	math::Vector<DOF>::type gainTmp;


	while (going) {
		std::cout << ">>> ";
		std::getline(std::cin, line);

		switch (line[0]) {
		case 'h':
			if (handThreadGoing) {
				handThreadGoing = false;
				handThread.join();
			} else {
				boost::thread tmpThread(handleHandCommands, wam.wam.wambot->bus, &handThreadGoing);
				handThread.swap(tmpThread);
			}
			break;

		case 'r':
			printf("Waking hand pucks ...\n");
			//   bus = wam.wam.wambot->bus;
			for (int i = 11; i <= 14; i++)
				bt_bus_set_property(wam.wam.wambot->bus, i, 5, 0); // Set STAT to STATUS_RESET

			usleep((long) 1e6);

			for (int i = 11; i <= 14; i++)
				bt_bus_set_property(wam.wam.wambot->bus, i, 5, 2); // Set STAT to STATUS_READY

			usleep((long) 1e6);

			for (int i = 11; i <= 14; i++)
				bt_bus_set_property(wam.wam.wambot->bus, i, 78, 50); // Set TSTOP to 50 ms

			printf("Initializing hand ...\n");
			break;

		case 'l':
			if (mm.isLocked()) {
				mm.unlock();
			} else {
				// build spline to setPoint
				std::vector<Wam<DOF>::jp_type> vec;
				vec.push_back(wam.getJointPositions());
				vec.push_back(origin);
				math::Spline<Wam<DOF>::jp_type> spline(vec);
				math::TrapezoidalVelocityProfile profile(.5, 1.0, 0, spline.changeInX());

				systems::Ramp time;
				systems::Callback<double, Wam<DOF>::jp_type> trajectory(boost::bind(boost::ref(spline), boost::bind(boost::ref(profile), _1)));

				time.setSamplePeriod(T_s);

				systems::connect(time.output, trajectory.input);
				wam.trackReferenceSignal(trajectory.output);
				time.start();


				std::cout << "Press [Enter] to start sending joint positions.\n";
				waitForEnter();

			//	wam.trackReferenceSignal(half.output);
				wam.trackReferenceSignal(*mmOutput);
			//	wam.trackReferenceSignal(lpf.output);


				std::cout << "Press [Enter] to link with the other WAM.\n";
				waitForEnter();
				mm.tryToLock();
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
//		case 'q':
//		case 'x':
//			going = false;
//			break;

		default:
//			std::cout << mm.input.getValue() << std::endl << wam.jpController.referenceInput.getValue();
//			std::cout << subtract.polarity[0] << " " << subtract.polarity[1] << std::endl;
//			std::cout << add.getInput(0).getValue() << " " << add.getInput(1).getValue() << " " << aeAdd.getElementInput(J6_IDX).getValue() << std::endl;
//			std::cout << subtract.getInput(0).getValue() << " " << subtract.getInput(1).getValue() << " " << aeSubtract.getElementInput(J6_IDX).getValue() << std::endl;
			std::cout	<< "\n\t'l' to toggle locking with other WAM\n"
						<< "\t'h' to toggle hand controller buttons\n"
						<< "\t't' to tune control gains\n";
			break;
		}
	}



//	std::cout << "Press [Enter] to idle.\n";
//	waitForEnter();
//	wam.idle();
//	wam.gravityCompensate(false);

	std::cout << "Shift-idle, then press [Enter].\n";
	waitForEnter();
	rtem.stop();

	return 0;
}



void handleHandCommands(struct bt_bus *bus, bool* going) {
	int err, i;
   int sock;
   long flags;
   int buflen;
   int buflenlen;
   struct sockaddr_in bind_addr;
   //struct bhand * bhand;
//   int going;
   unsigned char old_bits;
   int num_missed;
   //int i;
   int realtime;
//   struct bt_bus *bus;

   printf("Starting bhand-remote-server on port %d ...\n", port);
   /* Create socket */
   sock = socket(PF_INET, SOCK_DGRAM, 0);
   if (sock == -1)
   {
	  printf("Could not create socket.\n");
	  return;
   }

   /* Set socket to non-blocking, set flag associated with open file */
   flags = fcntl(sock, F_GETFL, 0);
   if (flags < 0)
   {
	  printf("Could not get socket flags.\n");
	  return;
   }
   flags |= O_NONBLOCK;
   err = fcntl(sock, F_SETFL, flags);
   if (err < 0)
   {
	  printf("Could not set socket flags.\n");
	  return;
   }

   /* Set socket buffer size */
   buflenlen = sizeof(buflen);
   buflen = 10 * sizeof(unsigned char);
   err = setsockopt(sock, SOL_SOCKET, SO_SNDBUF, (char *)&buflen, buflenlen);
   if (err)
   {
	  printf("Could not set output buffer size.\n");
	  return;
   }

   /* Set up the bind address */
   bind_addr.sin_family = AF_INET;
   bind_addr.sin_port = htons(port);
   bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
   err = bind(sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr));
   if (err == -1)
   {
	  printf("Could not bind to socket on port %d.", port);
	  return;
   }

   // register this task with xenomai so we can talk on the CAN bus
   rt_task_shadow(new RT_TASK, NULL, 10, 0);

   printf("Waking hand pucks ...\n");
//   bus = wam.wam.wambot->bus;
   for(i = 11; i <= 14; i++)
	   bt_bus_set_property(bus, i, 5, 2); // Set STAT to STATUS_READY

   usleep((long)1e6);

   for(i = 11; i <= 14; i++)
   	   bt_bus_set_property(bus, i, 78, 50); // Set TSTOP to 50 ms

   printf("Initializing hand ...\n");
   /* Initialize the hand
   bt_bus_set_property(bus, 11, 29, 13); // Set CMD to CMD_HI
   bt_bus_set_property(bus, 12, 29, 13); // Set CMD to CMD_HI
   bt_bus_set_property(bus, 13, 29, 13); // Set CMD to CMD_HI
   bt_bus_set_property(bus, 14, 29, 13); // Set CMD to CMD_HI
*/
   printf(" ... done.\n");

   /* Start listening for packets ... */
   old_bits = 0;
   num_missed = 1000;
   *going = true;
   realtime = 1;
   while (*going)
   {
	  unsigned char bits;

	  /* Get any packets in the buffer */
	  num_missed++;
	  while (recv(sock, &bits, sizeof(unsigned char), 0) == sizeof(unsigned char))
		 num_missed = 0;

	  /* If things havn't changed, just wait and loop */
	  if ((num_missed) || (bits == old_bits))
	  {
		 /* If it's been more than 50 without a message,
		  * set 0 velocity */
		 if (num_missed == 50 && realtime)
		 {
			printf("Sending stop command to hand.\n");
			bt_bus_set_property(bus, 11, 8, 0); // Set MODE to MODE_IDLE
			bt_bus_set_property(bus, 12, 8, 0); // Set MODE to MODE_IDLE
			bt_bus_set_property(bus, 13, 8, 0); // Set MODE to MODE_IDLE
			bt_bus_set_property(bus, 14, 8, 3); // Set MODE to MODE_PID
		 }

		 usleep(10000); /* ~ 100 Hz */
		 continue;
	  }

	  /* OK, we've gotten a new value. */

	  /* Spread (maybe breaking realtime) */
	  if ( !b1(old_bits) && !b2(old_bits) &&  b1(bits) && !b2(bits) )
	  {
		 // SO
	  }
	  if ( !realtime && !b1(old_bits) && !b2(old_bits) && !b1(bits) &&  b2(bits) )
	  {
		 // SC
	  }

	  if (realtime)
	  {
		 /* Spread */
		 if      (  b1(bits) && !b2(bits) ) bt_bus_set_property(bus, 14, 44, -spreadspeed); // Open
		 else if ( !b1(bits) &&  b2(bits) ) bt_bus_set_property(bus, 14, 44,  spreadspeed); // Close
		 else                               bt_bus_set_property(bus, 14, 44,      0); // Stop
		 bt_bus_set_property(bus, 14, 8, 4);

		 /* Finger 1 */
		 if      (  b3(bits) && !b4(bits) ) bt_bus_set_property(bus, 12, 44, -speed); // Open
		 else if ( !b3(bits) &&  b4(bits) ) bt_bus_set_property(bus, 12, 44,  speed); // Close
		 else                               bt_bus_set_property(bus, 12, 44,      0); // Stop
		 bt_bus_set_property(bus, 12, 8, 4);

		 /* Finger 2 */
		 if      (  b5(bits) && !b6(bits) ) bt_bus_set_property(bus, 13, 44, -speed); // Open
		  else if ( !b5(bits) &&  b6(bits) ) bt_bus_set_property(bus, 13, 44,  speed); // Close
		  else                               bt_bus_set_property(bus, 13, 44,      0); // Stop
		  bt_bus_set_property(bus, 13, 8, 4);

		 /* Finger 3 */
		  if      (  b7(bits) && !b8(bits) ) bt_bus_set_property(bus, 11, 44, -speed); // Open
		   else if ( !b7(bits) &&  b8(bits) ) bt_bus_set_property(bus, 11, 44,  speed); // Close
		   else                               bt_bus_set_property(bus, 11, 44,      0); // Stop
		   bt_bus_set_property(bus, 11, 8, 4);

		 //bhand_RTUpdate(bhand,1,1);
	  }

	  printf("Bits are now: %d %d %d %d %d %d %d %d\n",
				(bits >> 7) & 1,
				(bits >> 6) & 1,
				(bits >> 5) & 1,
				(bits >> 4) & 1,
				(bits >> 3) & 1,
				(bits >> 2) & 1,
				(bits >> 1) & 1,
				(bits >> 0) & 1 );

	  old_bits = bits;
	  usleep(10000); /* ~ 100 Hz */
   }


   printf(" ... done.\n");
}
