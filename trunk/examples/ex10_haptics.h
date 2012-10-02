/*
	Copyright 2010, 2011, 2012 Barrett Technology <support@barrett.com>

	This file is part of libbarrett.

	This version of libbarrett is free software: you can redistribute it
	and/or modify it under the terms of the GNU General Public License as
	published by the Free Software Foundation, either version 3 of the
	License, or (at your option) any later version.

	This version of libbarrett is distributed in the hope that it will be
	useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License along
	with this version of libbarrett.  If not, see
	<http://www.gnu.org/licenses/>.

	Further, non-binding information about licensing is available at:
	<http://wiki.barrett.com/libbarrett/wiki/LicenseNotes>
*/

/*
 * ex10_haptics.h
 *
 *  Created on: Apr 14, 2010
 *      Author: cd
 *      Author: dc
 */

#ifndef NETWORK_HAPTICS_H_
#define NETWORK_HAPTICS_H_


#include <stdexcept>

#include <unistd.h> /* for close() */
#include <sys/socket.h> /* For sockets */
#include <fcntl.h>      /* To change socket to nonblocking mode */
#include <arpa/inet.h>  /* For inet_pton() */

#include <barrett/os.h>
#include <barrett/detail/ca_macro.h>
#include <barrett/units.h>
#include <barrett/systems/abstract/single_io.h>
#include <barrett/thread/disable_secondary_mode_warning.h>

class NetworkHaptics : public barrett::systems::SingleIO<barrett::units::CartesianPosition::type, barrett::units::CartesianForce::type> {
	BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;

public:
	static const int SIZE_OF_MSG = 3 * sizeof(double);

	explicit NetworkHaptics(barrett::systems::ExecutionManager* em, char* remoteHost, int port = 5555, const std::string& sysName = "NetworkHaptics") :
		barrett::systems::SingleIO<cp_type, cf_type>(sysName), sock(-1), cp(0.0), numMissed(0), cf(0.0)
	{
		int err;
		long flags;
		int buflen;
		unsigned int buflenlen;
		struct sockaddr_in bind_addr;
		struct sockaddr_in their_addr;

		/* Create socket */
		sock = socket(PF_INET, SOCK_DGRAM, 0);
		if (sock == -1)
		{
			(barrett::logMessage("(NetworkHaptics::NetworkHaptics): Ctor failed %s: Could not create socket.") % __func__ ).raise<std::runtime_error>();
		}

		/* Set socket to non-blocking, set flag associated with open file */
		flags = fcntl(sock, F_GETFL, 0);
		if (flags < 0)
		{
			(barrett::logMessage("(NetworkHaptics::NetworkHaptics): Ctor failed  %s: Could not get socket flags.") % __func__).raise<std::runtime_error>();
		}
		flags |= O_NONBLOCK;
		err = fcntl(sock, F_SETFL, flags);
		if (err < 0)
		{
			(barrett::logMessage("(NetworkHaptics::NetworkHaptics): Ctor failed %s: Could not set socket flags.") % __func__ ).raise<std::runtime_error>();
		}

		/* Maybe set UDP buffer size? */
		buflenlen = sizeof(buflen);
		err = getsockopt(sock, SOL_SOCKET, SO_SNDBUF, (char *)&buflen, &buflenlen);
		if (err)
		{
			(barrett::logMessage("(NetworkHaptics::NetworkHaptics): Ctor failed %s: Could not get output buffer size.") % __func__ ).raise<std::runtime_error>();
		}
		barrett::logMessage("%s: Note, output buffer is %d bytes.") % __func__ % buflen;

		buflenlen = sizeof(buflen);
		buflen = 5 * SIZE_OF_MSG;
		err = setsockopt(sock, SOL_SOCKET, SO_SNDBUF, (char *)&buflen, buflenlen);
		if (err)
		{
			(barrett::logMessage("(NetworkHaptics::NetworkHaptics): Ctor failed  %s: Could not set output buffer size.") % __func__ ).raise<std::runtime_error>();
		}

		buflenlen = sizeof(buflen);
		err = getsockopt(sock, SOL_SOCKET, SO_SNDBUF, (char *)&buflen, &buflenlen);
		if (err)
		{
			(barrett::logMessage("(NetworkHaptics::NetworkHaptics): Ctor failed  %s: Could not get output buffer size.") % __func__ ).raise<std::runtime_error>();
		}
		barrett::logMessage("%s: Note, output buffer is %d bytes.") % __func__ % buflen;

		/* Set up the bind address */
		bind_addr.sin_family = AF_INET;
		bind_addr.sin_port = htons(port);
		bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
		err = bind(sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr));
		if (err == -1)
		{
			(barrett::logMessage("(NetworkHaptics::NetworkHaptics): Ctor failed %s: Could not bind to socket on port %d") % __func__ % port).raise<std::runtime_error>();
		}

		/* Set up the other guy's address */
		their_addr.sin_family = AF_INET;
		their_addr.sin_port = htons(port);
		err = ! inet_pton(AF_INET, remoteHost, &their_addr.sin_addr);
		if (err)
		{
			(barrett::logMessage("(NetworkHaptics::NetworkHaptics): Ctor failed %s: Bad IP argument '%s'.") %__func__ % remoteHost).raise<std::runtime_error>();
		}

		/* Call "connect" to set datagram destination */
		err = connect(sock, (struct sockaddr *)&their_addr, sizeof(struct sockaddr));
		if (err)
		{
			(barrett::logMessage("(NetworkHaptics::NetworkHaptics): Ctor failed %s: Could not set datagram destination.") % __func__ ).raise<std::runtime_error>();
		}


		if (em != NULL) {
			em->startManaging(*this);
		}
	}

	virtual ~NetworkHaptics() {
		mandatoryCleanUp();
		close(sock);
	}

protected:
	virtual void operate() {
		cp = input.getValue();

		{
			// send() and recv() cause switches to secondary mode. The socket is
			// non-blocking, so this *probably* won't impact the control-loop
			// timing that much...
			barrett::thread::DisableSecondaryModeWarning dsmw;

			send(sock, cp.data(), SIZE_OF_MSG, 0);

			++numMissed;
			while (recv(sock, cf.data(), SIZE_OF_MSG, 0) == SIZE_OF_MSG) {
				numMissed = 0;
			}
			if (numMissed > 10) {
				cf.setZero();
			}
		}

		outputValue->setData(&cf);
	}

	int sock;
	cp_type cp;
	int numMissed;
	cf_type cf;

private:
	DISALLOW_COPY_AND_ASSIGN(NetworkHaptics);
};

#endif /* NETWORK_HAPTICS_H_ */
