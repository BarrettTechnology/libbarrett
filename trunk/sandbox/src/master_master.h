/*
 * master_master.h
 *
 *  Created on: Feb 22, 2010
 *      Author: Christopher Dellin
 *      Author: Dan Cody
 */

#ifndef MASTER_MASTER_H_
#define MASTER_MASTER_H_


#include <stdexcept>

#include <syslog.h>

#include <boost/thread.hpp>

#include <unistd.h> /* for close() */
#include <sys/socket.h> /* For sockets */
#include <fcntl.h>      /* To change socket to nonblocking mode */
#include <arpa/inet.h>  /* For inet_pton() */

#include <barrett/detail/ca_macro.h>
#include <barrett/units.h>
#include <barrett/systems/abstract/single_io.h>


namespace barrett {
namespace systems {


template<size_t DOF>
class MasterMaster : public SingleIO<typename units::JointPositions<DOF>::type, typename units::JointPositions<DOF>::type> {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

public:
	explicit MasterMaster(char* remoteHost, int port = 5555) :
		sock(-1), init(true), locked(false), numMissed(), theirJp(), stopRunning(false), thread() {
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
			throw std::runtime_error("(systems::MasterMaster::MasterMaster): Ctor failed. Check /var/log/syslog.");
		}

		/* Set socket to non-blocking, set flag associated with open file */
		flags = fcntl(sock, F_GETFL, 0);
		if (flags < 0)
		{
			syslog(LOG_ERR,"%s: Could not get socket flags.",__func__);
			throw std::runtime_error("(systems::MasterMaster::MasterMaster): Ctor failed. Check /var/log/syslog.");
		}
		flags |= O_NONBLOCK;
		err = fcntl(sock, F_SETFL, flags);
		if (err < 0)
		{
			syslog(LOG_ERR,"%s: Could not set socket flags.",__func__);
			throw std::runtime_error("(systems::MasterMaster::MasterMaster): Ctor failed. Check /var/log/syslog.");
		}

		/* Maybe set UDP buffer size? */
		buflenlen = sizeof(buflen);
		err = getsockopt(sock, SOL_SOCKET, SO_SNDBUF, (char *)&buflen, &buflenlen);
		if (err)
		{
			syslog(LOG_ERR,"%s: Could not get output buffer size.",__func__);
			throw std::runtime_error("(systems::MasterMaster::MasterMaster): Ctor failed. Check /var/log/syslog.");
		}
		syslog(LOG_ERR,"%s: Note, output buffer is %d bytes.",__func__,buflen);

		buflenlen = sizeof(buflen);
		buflen = 5 * DOF * sizeof(double);
		err = setsockopt(sock, SOL_SOCKET, SO_SNDBUF, (char *)&buflen, buflenlen);
		if (err)
		{
			syslog(LOG_ERR,"%s: Could not set output buffer size.",__func__);
			throw std::runtime_error("(systems::MasterMaster::MasterMaster): Ctor failed. Check /var/log/syslog.");
		}

		buflenlen = sizeof(buflen);
		err = getsockopt(sock, SOL_SOCKET, SO_SNDBUF, (char *)&buflen, &buflenlen);
		if (err)
		{
			syslog(LOG_ERR,"%s: Could not get output buffer size.",__func__);
			throw std::runtime_error("(systems::MasterMaster::MasterMaster): Ctor failed. Check /var/log/syslog.");
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
			throw std::runtime_error("(systems::MasterMaster::MasterMaster): Ctor failed. Check /var/log/syslog.");
		}

		/* Set up the other guy's address */
		their_addr.sin_family = AF_INET;
		their_addr.sin_port = htons(port);
		err = ! inet_pton(AF_INET, remoteHost, &their_addr.sin_addr);
		if (err)
		{
			syslog(LOG_ERR,"%s: Bad IP argument '%s'.",__func__,remoteHost);
			throw std::runtime_error("(systems::MasterMaster::MasterMaster): Ctor failed. Check /var/log/syslog.");
		}

		/* Call "connect" to set datagram destination */
		err = connect(sock, (struct sockaddr *)&their_addr, sizeof(struct sockaddr));
		if (err)
		{
			syslog(LOG_ERR,"%s: Could not set datagram destination.",__func__);
			throw std::runtime_error("(systems::MasterMaster::MasterMaster): Ctor failed. Check /var/log/syslog.");
		}


		// start comm thread
		boost::thread tmpThread(Runnable(this));
		thread.swap(tmpThread);
	}

	virtual ~MasterMaster() {
		stopRunning = true;
		thread.join();

		close(sock);
	}

	bool isLocked() {  return locked;  }
	void tryToLock() {
		theirJp.setConstant(0.0);
		init = false;
		locked = true;
	}
	void unlock() {
		theirJp.setConstant(0.0);
		init = true;
		locked = false;
	}

protected:
	class Runnable {
	public:
		Runnable(MasterMaster<DOF>* mm) :
			mm(mm) {}
		void operator() () {
			while ( !mm->stopRunning ) {
//				usleep(580); // give other threads a turn...
//				usleep(600); // give other threads a turn...
//				usleep(615); // give other threads a turn...

				usleep(750); // give other threads a turn...

//				usleep(2000); // about 500Hz
				usleep(1000000); // about 1Hz


//				send(mm->sock, mm->input.getValue().data(), DOF*sizeof(double), 0);
//
//				++mm->numMissed;
//				while (recv(mm->sock, mm->theirJp.data(), DOF*sizeof(double), 0) == DOF*sizeof(double)) {
//					mm->numMissed = 0;
//				}
			}
		}

	private:
		MasterMaster<DOF>* mm;
	};


	virtual void operate() {
		send(sock, this->input.getValue().data(), DOF*sizeof(double), 0);


		if (init) {
			this->outputValue->setValue(jp_type(0.0));
			return;
		}


		++numMissed;
		while (recv(sock, theirJp.data(), DOF*sizeof(double), 0) == DOF*sizeof(double)) {
			numMissed = 0;
		}
		if (numMissed > 10) {
			locked = false;
			theirJp.setConstant(0.0);
		}

		if (locked) {
//			this->outputValue->setValue(0.5*theirJp + 0.5*this->input.getValue());
			this->outputValue->setValue(theirJp);
		} else {
			this->outputValue->setValue(this->input.getValue());
		}
	}

	int sock;
	bool init;
	bool locked;
	int numMissed;
	jp_type theirJp;

	bool stopRunning;
	boost::thread thread;

private:
	DISALLOW_COPY_AND_ASSIGN(MasterMaster);
};


}
}


#endif /* MASTER_MASTER_H_ */
