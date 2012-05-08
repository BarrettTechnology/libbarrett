/*
 * ex11_master_master.h
 *
 *  Created on: Feb 22, 2010
 *      Author: Christopher Dellin
 *      Author: Dan Cody
 */

#ifndef MASTER_MASTER_H_
#define MASTER_MASTER_H_


#include <stdexcept>

#include <syslog.h>
#include <unistd.h> /* for close() */
#include <sys/socket.h> /* For sockets */
#include <fcntl.h>      /* To change socket to nonblocking mode */
#include <arpa/inet.h>  /* For inet_pton() */

#include <barrett/detail/ca_macro.h>
#include <barrett/thread/abstract/mutex.h>
#include <barrett/thread/disable_secondary_mode_warning.h>
#include <barrett/units.h>
#include <barrett/systems/abstract/single_io.h>


template <size_t DOF>
class MasterMaster : public barrett::systems::SingleIO<typename barrett::units::JointPositions<DOF>::type, typename barrett::units::JointPositions<DOF>::type> {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

public:
	explicit MasterMaster(barrett::systems::ExecutionManager* em, char* remoteHost, int port = 5555, const std::string& sysName = "MasterMaster") :
		barrett::systems::SingleIO<jp_type,jp_type>(sysName), sock(-1), linked(false), numMissed(NUM_MISSED_LIMIT), theirJp(0.0)
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
			throw std::runtime_error("(MasterMaster::MasterMaster): Ctor failed. Check /var/log/syslog.");
		}

		/* Set socket to non-blocking, set flag associated with open file */
		flags = fcntl(sock, F_GETFL, 0);
		if (flags < 0)
		{
			syslog(LOG_ERR,"%s: Could not get socket flags.",__func__);
			throw std::runtime_error("(MasterMaster::MasterMaster): Ctor failed. Check /var/log/syslog.");
		}
		flags |= O_NONBLOCK;
		err = fcntl(sock, F_SETFL, flags);
		if (err < 0)
		{
			syslog(LOG_ERR,"%s: Could not set socket flags.",__func__);
			throw std::runtime_error("(MasterMaster::MasterMaster): Ctor failed. Check /var/log/syslog.");
		}

		/* Maybe set UDP buffer size? */
		buflenlen = sizeof(buflen);
		err = getsockopt(sock, SOL_SOCKET, SO_SNDBUF, (char *)&buflen, &buflenlen);
		if (err)
		{
			syslog(LOG_ERR,"%s: Could not get output buffer size.",__func__);
			throw std::runtime_error("(MasterMaster::MasterMaster): Ctor failed. Check /var/log/syslog.");
		}
		syslog(LOG_ERR,"%s: Note, output buffer is %d bytes.",__func__,buflen);

		buflenlen = sizeof(buflen);
		buflen = 5 * SIZE_OF_MSG;
		err = setsockopt(sock, SOL_SOCKET, SO_SNDBUF, (char *)&buflen, buflenlen);
		if (err)
		{
			syslog(LOG_ERR,"%s: Could not set output buffer size.",__func__);
			throw std::runtime_error("(MasterMaster::MasterMaster): Ctor failed. Check /var/log/syslog.");
		}

		buflenlen = sizeof(buflen);
		err = getsockopt(sock, SOL_SOCKET, SO_SNDBUF, (char *)&buflen, &buflenlen);
		if (err)
		{
			syslog(LOG_ERR,"%s: Could not get output buffer size.",__func__);
			throw std::runtime_error("(MasterMaster::MasterMaster): Ctor failed. Check /var/log/syslog.");
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
			throw std::runtime_error("(MasterMaster::MasterMaster): Ctor failed. Check /var/log/syslog.");
		}

		/* Set up the other guy's address */
		their_addr.sin_family = AF_INET;
		their_addr.sin_port = htons(port);
		err = ! inet_pton(AF_INET, remoteHost, &their_addr.sin_addr);
		if (err)
		{
			syslog(LOG_ERR,"%s: Bad IP argument '%s'.",__func__,remoteHost);
			throw std::runtime_error("(MasterMaster::MasterMaster): Ctor failed. Check /var/log/syslog.");
		}

		/* Call "connect" to set datagram destination */
		err = connect(sock, (struct sockaddr *)&their_addr, sizeof(struct sockaddr));
		if (err)
		{
			syslog(LOG_ERR,"%s: Could not set datagram destination.",__func__);
			throw std::runtime_error("(MasterMaster::MasterMaster): Ctor failed. Check /var/log/syslog.");
		}


		if (em != NULL) {
			em->startManaging(*this);
		}
	}

	virtual ~MasterMaster() {
		this->mandatoryCleanUp();

		close(sock);
	}

	bool isLinked() const { return linked; }
	void tryLink() {
		BARRETT_SCOPED_LOCK(this->getEmMutex());

		if (numMissed < NUM_MISSED_LIMIT) {
			linked = true;
		}
	}
	void unlink() { linked = false; }

protected:
	static const int SIZE_OF_MSG = DOF*sizeof(double);
	static const int NUM_MISSED_LIMIT = 10;

	virtual void operate() {

		{
			// send() and recv() cause switches to secondary mode. The socket is
			// non-blocking, so this *probably* won't impact the control-loop
			// timing that much...
			barrett::thread::DisableSecondaryModeWarning dsmw;


			send(sock, this->input.getValue().data(), SIZE_OF_MSG, 0);

			if (numMissed < NUM_MISSED_LIMIT) {  // prevent numMissed from wrapping
				++numMissed;
			}
			while (recv(sock, theirJp.data(), SIZE_OF_MSG, 0) == SIZE_OF_MSG) {
				numMissed = 0;
			}
		}

		if ( !linked  ||  numMissed >= NUM_MISSED_LIMIT) {
			linked = false;
			theirJp = this->input.getValue();
		}

		this->outputValue->setData(&theirJp);
	}

	int sock;
	bool linked;
	int numMissed;
	jp_type theirJp;

private:
	DISALLOW_COPY_AND_ASSIGN(MasterMaster);
};


#endif /* MASTER_MASTER_H_ */
