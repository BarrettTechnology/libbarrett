/*
 * network_haptics.h
 *
 *  Created on: Apr 14, 2010
 *      Author: dc
 */

#ifndef NETWORK_HAPTICS_H_
#define NETWORK_HAPTICS_H_


#include <stdexcept>

#include <syslog.h>
#include <unistd.h> /* for close() */
#include <sys/socket.h> /* For sockets */
#include <fcntl.h>      /* To change socket to nonblocking mode */
#include <arpa/inet.h>  /* For inet_pton() */

#include <barrett/detail/ca_macro.h>
#include <barrett/units.h>
#include <barrett/systems/abstract/single_io.h>


class NetworkHaptics : public barrett::systems::SingleIO<barrett::units::CartesianPosition::type, barrett::units::CartesianForce::type> {
	typedef barrett::units::CartesianPosition::type cp_type;
	typedef barrett::units::CartesianForce::type cf_type;

public:
	static const int SIZE_OF_MSG = 3 * sizeof(double);

	explicit NetworkHaptics(char* remoteHost, int port = 5555) :
		barrett::systems::SingleIO<cp_type, cf_type>(true), sock(-1), numMissed(0), cf(0.0)
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
			syslog(LOG_ERR,"%s: Could not create socket.",__func__);
			throw std::runtime_error("(systems::NetworkHaptics::NetworkHaptics): Ctor failed. Check /var/log/syslog.");
		}

		/* Set socket to non-blocking, set flag associated with open file */
		flags = fcntl(sock, F_GETFL, 0);
		if (flags < 0)
		{
			syslog(LOG_ERR,"%s: Could not get socket flags.",__func__);
			throw std::runtime_error("(systems::NetworkHaptics::NetworkHaptics): Ctor failed. Check /var/log/syslog.");
		}
		flags |= O_NONBLOCK;
		err = fcntl(sock, F_SETFL, flags);
		if (err < 0)
		{
			syslog(LOG_ERR,"%s: Could not set socket flags.",__func__);
			throw std::runtime_error("(systems::NetworkHaptics::NetworkHaptics): Ctor failed. Check /var/log/syslog.");
		}

		/* Maybe set UDP buffer size? */
		buflenlen = sizeof(buflen);
		err = getsockopt(sock, SOL_SOCKET, SO_SNDBUF, (char *)&buflen, &buflenlen);
		if (err)
		{
			syslog(LOG_ERR,"%s: Could not get output buffer size.",__func__);
			throw std::runtime_error("(systems::NetworkHaptics::NetworkHaptics): Ctor failed. Check /var/log/syslog.");
		}
		syslog(LOG_ERR,"%s: Note, output buffer is %d bytes.",__func__,buflen);

		buflenlen = sizeof(buflen);
		buflen = 5 * SIZE_OF_MSG;
		err = setsockopt(sock, SOL_SOCKET, SO_SNDBUF, (char *)&buflen, buflenlen);
		if (err)
		{
			syslog(LOG_ERR,"%s: Could not set output buffer size.",__func__);
			throw std::runtime_error("(systems::NetworkHaptics::NetworkHaptics): Ctor failed. Check /var/log/syslog.");
		}

		buflenlen = sizeof(buflen);
		err = getsockopt(sock, SOL_SOCKET, SO_SNDBUF, (char *)&buflen, &buflenlen);
		if (err)
		{
			syslog(LOG_ERR,"%s: Could not get output buffer size.",__func__);
			throw std::runtime_error("(systems::NetworkHaptics::NetworkHaptics): Ctor failed. Check /var/log/syslog.");
		}
		syslog(LOG_ERR,"%s: Note, output buffer is %d bytes.",__func__,buflen);

		/* Set up the bind address */
		bind_addr.sin_family = AF_INET;
		bind_addr.sin_port = htons(port);
		bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
		err = bind(sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr));
		if (err == -1)
		{
			syslog(LOG_ERR,"%s: Could not bind to socket on port %d.",__func__,port);
			throw std::runtime_error("(systems::NetworkHaptics::NetworkHaptics): Ctor failed. Check /var/log/syslog.");
		}

		/* Set up the other guy's address */
		their_addr.sin_family = AF_INET;
		their_addr.sin_port = htons(port);
		err = ! inet_pton(AF_INET, remoteHost, &their_addr.sin_addr);
		if (err)
		{
			syslog(LOG_ERR,"%s: Bad IP argument '%s'.",__func__,remoteHost);
			throw std::runtime_error("(systems::NetworkHaptics::NetworkHaptics): Ctor failed. Check /var/log/syslog.");
		}

		/* Call "connect" to set datagram destination */
		err = connect(sock, (struct sockaddr *)&their_addr, sizeof(struct sockaddr));
		if (err)
		{
			syslog(LOG_ERR,"%s: Could not set datagram destination.",__func__);
			throw std::runtime_error("(systems::NetworkHaptics::NetworkHaptics): Ctor failed. Check /var/log/syslog.");
		}
	}

	virtual ~NetworkHaptics() {
		close(sock);
	}

protected:
	virtual void operate() {
		send(sock, input.getValue().data(), SIZE_OF_MSG, 0);

		++numMissed;
		while (recv(sock, cf.data(), SIZE_OF_MSG, 0) == SIZE_OF_MSG) {
			numMissed = 0;
		}
		if (numMissed > 10) {
			cf.setConstant(0.0);
		}

		outputValue->setValue(cf);
	}

	int sock;
	int numMissed;
	cf_type cf;

private:
	DISALLOW_COPY_AND_ASSIGN(NetworkHaptics);
};

#endif /* NETWORK_HAPTICS_H_ */
