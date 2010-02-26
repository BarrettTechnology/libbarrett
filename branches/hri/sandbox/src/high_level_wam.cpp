/*
 * high_level_wam.cpp
 *
 *  Created on: Feb 2, 2010
 *      Author: dc
 */

#include <stdlib.h>
#include <stdio.h>

#include <sys/socket.h> /* For sockets */
#include <fcntl.h>      /* To change socket to nonblocking mode */
#include <arpa/inet.h>  /* For inet_aton() */

#include <iostream>
#include <string>

#include <unistd.h>  // usleep

#include <boost/ref.hpp>
#include <boost/bind.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/tuple/tuple_io.hpp>
#include <libconfig.h++>
#include <Eigen/Geometry>

#include <barrett/exception.h>
#include <barrett/detail/debug.h>
#include <barrett/thread/abstract/mutex.h>
#include <barrett/math.h>
#include <barrett/units.h>
#include <barrett/log.h>
#include <barrett/systems.h>
#include <barrett/wam.h>

//namespace log = barrett::log;
namespace math = barrett::math;
namespace systems = barrett::systems;
namespace units = barrett::units;
using barrett::Wam;
using systems::connect;
using systems::reconnect;
using systems::disconnect;


//using boost::bind;
using boost::ref;

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


const size_t DOF = 7;
const double T_s = 0.002;

typedef Wam<DOF>::jt_type jt_type;
typedef Wam<DOF>::jp_type jp_type;
typedef Wam<DOF>::jv_type jv_type;


void waitForEnter() {
	static std::string line;
	std::getline(std::cin, line);
}


int main() {
	barrett::installExceptionHandler();  // give us pretty stack traces when things die

	libconfig::Config config;
	config.readFile("/etc/wam/wamg-new.config");

	systems::RealTimeExecutionManager rtem(T_s);
	systems::System::defaultExecutionManager = &rtem;


    // instantiate Systems
	Wam<DOF> wam(config.lookup("wam"));

	// start the main loop!
	rtem.start();

	std::cout << "Press [Enter] to compensate for gravity.\n";
	waitForEnter();
	wam.gravityCompensate();

	/* Do stuff */
	{
		int err;
	   int sock;
	   long flags;
	   int buflen;
	   int buflenlen;
	   struct sockaddr_in bind_addr;
	   //struct bhand * bhand;
	   int going;
	   unsigned char old_bits;
	   int num_missed;
	   //int i;
	   int realtime;
	   struct bt_bus *bus;

	   printf("Starting bhand-remote-server on port %d ...\n", port);
	   /* Create socket */
	   sock = socket(PF_INET, SOCK_DGRAM, 0);
	   if (sock == -1)
	   {
		  printf("Could not create socket.\n");
		  return -1;
	   }

	   /* Set socket to non-blocking, set flag associated with open file */
	   flags = fcntl(sock, F_GETFL, 0);
	   if (flags < 0)
	   {
		  printf("Could not get socket flags.\n");
		  return -1;
	   }
	   flags |= O_NONBLOCK;
	   err = fcntl(sock, F_SETFL, flags);
	   if (err < 0)
	   {
		  printf("Could not set socket flags.\n");
		  return -1;
	   }

	   /* Set socket buffer size */
	   buflenlen = sizeof(buflen);
	   buflen = 10 * sizeof(unsigned char);
	   err = setsockopt(sock, SOL_SOCKET, SO_SNDBUF, (char *)&buflen, buflenlen);
	   if (err)
	   {
		  printf("Could not set output buffer size.\n");
		  return -1;
	   }

	   /* Set up the bind address */
	   bind_addr.sin_family = AF_INET;
	   bind_addr.sin_port = htons(port);
	   bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	   err = bind(sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr));
	   if (err == -1)
	   {
		  printf("Could not bind to socket on port %d.", port);
		  return -1;
	   }

	   printf("Waking hand pucks ...\n");
	   bus = wam.wam.wambot->bus;

	   bt_bus_set_property(bus, 11, 5, 0, 2); // Set STAT to STATUS_READY
	   bt_bus_set_property(bus, 12, 5, 0, 2); // Set STAT to STATUS_READY
	   bt_bus_set_property(bus, 13, 5, 0, 2); // Set STAT to STATUS_READY
	   bt_bus_set_property(bus, 14, 5, 0, 2); // Set STAT to STATUS_READY
	   usleep((long)1e6);

	   printf("Initializing hand ...\n");
	   /* Initialize the hand 
	   bt_bus_set_property(bus, 11, 29, 0, 13); // Set CMD to CMD_HI
	   bt_bus_set_property(bus, 12, 29, 0, 13); // Set CMD to CMD_HI
	   bt_bus_set_property(bus, 13, 29, 0, 13); // Set CMD to CMD_HI
	   bt_bus_set_property(bus, 14, 29, 0, 13); // Set CMD to CMD_HI
*/
	   printf(" ... done.\n");

	   /* Start listening for packets ... */
	   old_bits = 0;
	   num_missed = 1000;
	   going = 1;
	   realtime = 1;
	   while (going)
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
				bt_bus_set_property(bus, 11, 8, 0, 0); // Set MODE to MODE_IDLE
				bt_bus_set_property(bus, 12, 8, 0, 0); // Set MODE to MODE_IDLE
				bt_bus_set_property(bus, 13, 8, 0, 0); // Set MODE to MODE_IDLE
				bt_bus_set_property(bus, 14, 8, 0, 3); // Set MODE to MODE_PID
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
			 if      (  b1(bits) && !b2(bits) ) bt_bus_set_property(bus, 14, 44, 0, -spreadspeed); // Open
			 else if ( !b1(bits) &&  b2(bits) ) bt_bus_set_property(bus, 14, 44, 0,  spreadspeed); // Close
			 else                               bt_bus_set_property(bus, 14, 44, 0,      0); // Stop
			 bt_bus_set_property(bus, 14, 8, 0, 4);

			 /* Finger 1 */
			 if      (  b3(bits) && !b4(bits) ) bt_bus_set_property(bus, 12, 44, 0, -speed); // Open
			 else if ( !b3(bits) &&  b4(bits) ) bt_bus_set_property(bus, 12, 44, 0,  speed); // Close
			 else                               bt_bus_set_property(bus, 12, 44, 0,      0); // Stop
			 bt_bus_set_property(bus, 12, 8, 0, 4);

			 /* Finger 2 */
			 if      (  b5(bits) && !b6(bits) ) bt_bus_set_property(bus, 13, 44, 0, -speed); // Open
			  else if ( !b5(bits) &&  b6(bits) ) bt_bus_set_property(bus, 13, 44, 0,  speed); // Close
			  else                               bt_bus_set_property(bus, 13, 44, 0,      0); // Stop
			  bt_bus_set_property(bus, 13, 8, 0, 4);

			 /* Finger 3 */
			  if      (  b7(bits) && !b8(bits) ) bt_bus_set_property(bus, 11, 44, 0, -speed); // Open
			   else if ( !b7(bits) &&  b8(bits) ) bt_bus_set_property(bus, 11, 44, 0,  speed); // Close
			   else                               bt_bus_set_property(bus, 11, 44, 0,      0); // Stop
			   bt_bus_set_property(bus, 11, 8, 0, 4);

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
	/* End stuff */

	std::cout << "Press [Enter] to idle.\n";
	waitForEnter();
	wam.idle();
	wam.gravityCompensate(false);

	std::cout << "Shift-idle, then press [Enter].\n";
	waitForEnter();
	rtem.stop();

	return 0;
}
