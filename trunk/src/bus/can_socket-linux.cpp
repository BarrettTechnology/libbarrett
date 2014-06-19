/**
 *	Copyright 2009-2014 Barrett Technology <support@barrett.com>
 *
 *	This file is part of libbarrett.
 *
 *	This version of libbarrett is free software: you can redistribute it
 *	and/or modify it under the terms of the GNU General Public License as
 *	published by the Free Software Foundation, either version 3 of the
 *	License, or (at your option) any later version.
 *
 *	This version of libbarrett is distributed in the hope that it will be
 *	useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License along
 *	with this version of libbarrett.  If not, see
 *	<http://www.gnu.org/licenses/>.
 *
 *
 *	Barrett Technology Inc.
 *	73 Chapel Street
 *	Newton, MA 02458
 *
 */

/*
 * can_socket-linux.cpp
 *
 *  Created on: Jul 10, 2013
 *      Author: dc
 */

#include <stdexcept>
#include <cstdio>
#include <cstring>

#include <unistd.h>
#include <errno.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/uio.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include <barrett/os.h>
#include <barrett/thread/real_time_mutex.h>
#include <barrett/products/puck.h>
#include <barrett/bus/can_socket.h>


namespace barrett {
namespace bus {


namespace detail {
struct can_handle {
	typedef int handle_type;
	static const handle_type NULL_HANDLE = -1;
	handle_type h;

	can_handle() : h(NULL_HANDLE) {}
	bool isValid() const { return h != NULL_HANDLE; }
};
}


CANSocket::CANSocket() :
	mutex(), handle(new detail::can_handle)
{
}

CANSocket::CANSocket(int port) throw(std::runtime_error) :
	mutex(), handle(new detail::can_handle)
{
	open(port);
}

CANSocket::~CANSocket()
{
	close();
	delete handle;
	handle = NULL;
}


void CANSocket::open(int port) throw(std::logic_error, std::runtime_error)
{
	if (isOpen()) {
		(logMessage("CANSocket::%s(): This object is already associated with a CAN port.")
				% __func__).raise<std::logic_error>();
	}

	logMessage("CANSocket::open(%d) using Linux SocketCAN driver (NON-REALTIME!)") % port;

	int ret;

	ret = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	if (ret < 0) {
		close();
		(logMessage("CANSocket::%s(): Could not open CAN port. socket(): (%d) %s")
				% __func__ % -ret % strerror(-ret)).raise<std::runtime_error>();
	}
	handle->h = ret;

	struct ifreq ifr;
	char devname[10];
	sprintf(devname, "can%d", port);
	strncpy(ifr.ifr_name, devname, IFNAMSIZ);

	struct can_filter recvFilter[1];
	recvFilter[0].can_id = (Puck::HOST_ID << Puck::NODE_ID_WIDTH) | CAN_INV_FILTER;
	recvFilter[0].can_mask = Puck::FROM_MASK;
	ret = setsockopt(handle->h, SOL_CAN_RAW, CAN_RAW_FILTER, &recvFilter, sizeof(recvFilter));
	if (ret != 0) {
		close();
		(logMessage("CANSocket::%s(): Could not open CAN port. setsockopt(CAN_RAW_FILTER): (%d) %s")
				% __func__ % -ret % strerror(-ret)).raise<std::runtime_error>();
	}

	// Note: This must be done after the ioctl(SIOCGCANSTATE) call above,
	// otherwise send() will fail with ret = -6 (No such device or
	// address). The ifr.ifr_index gets overwritten because it is actually a
	// member of the ifr.ifr_ifru union.
	ret = ioctl(handle->h, SIOCGIFINDEX, &ifr);
	if (ret != 0) {
		close();
		(logMessage("CANSocket::%s(): Could not open CAN port. ioctl(SIOCGIFINDEX): (%d) %s")
				% __func__ % -ret % strerror(-ret)).raise<std::runtime_error>();
	}

	struct sockaddr_can toAddr;
	memset(&toAddr, 0, sizeof(toAddr));
	toAddr.can_ifindex = ifr.ifr_ifindex;
	toAddr.can_family = AF_CAN;
	ret = bind(handle->h, (struct sockaddr *) &toAddr, sizeof(toAddr));
	if (ret != 0) {
		close();
		(logMessage("CANSocket::%s(): Could not open CAN port. bind(): (%d) %s")
				% __func__ % -ret % strerror(-ret)).raise<std::runtime_error>();
	}

//	nanosecs_rel_t timeout = (nanosecs_rel_t) 1e9 * CommunicationsBus::TIMEOUT;
//	ret = ioctl(handle->h, RTCAN_RTIOC_RCV_TIMEOUT, &timeout);
//	if (ret != 0) {
//		close();
//		(logMessage("CANSocket::%s(): Could not open CAN port. ioctl(RCV_TIMEOUT): (%d) %s")
//				% __func__ % -ret % strerror(-ret)).raise<std::runtime_error>();
//	}
//	ret = ioctl(handle->h, RTCAN_RTIOC_SND_TIMEOUT, &timeout);
//	if (ret != 0) {
//		close();
//		(logMessage("CANSocket::%s(): Could not open CAN port. ioctl(SND_TIMEOUT): (%d) %s")
//				% __func__ % -ret % strerror(-ret)).raise<std::runtime_error>();
//	}
}

void CANSocket::close()
{
	if (isOpen()) {
		::close(handle->h);
		handle->h = detail::can_handle::NULL_HANDLE;
	}
}

bool CANSocket::isOpen() const
{
	return handle->isValid();
}

int CANSocket::send(int busId, const unsigned char* data, size_t len) const
{
	BARRETT_SCOPED_LOCK(mutex);

	struct can_frame frame;
	frame.can_id = busId;
	frame.can_dlc = len;
	memcpy(frame.data, data, len);

	int ret = ::send(handle->h, (void *) &frame, sizeof(struct can_frame), 0);
	if (ret < 0) {
		ret = -errno;  // Specific error info is in errno. Save a copy.

		switch (ret) {
		case -EAGAIN: // -EWOULDBLOCK
			logMessage("CANSocket::%s: "
					"send(): data would block during non-blocking send (output buffer full)")
					% __func__;
			return 1;
			break;
		case -ETIMEDOUT:
			logMessage("CANSocket::%s: "
					"send(): timed out")
					% __func__;
			return 2;
			break;
		case -EBADF:
			logMessage("CANSocket::%s: "
					"send(): aborted because socket was closed")
					% __func__;
			return 2;
		default:
			logMessage("CANSocket::%s: "
					"send(): (%d) %s")
					% __func__ % -ret % strerror(-ret);
			return 2;
		}
	} else if (ret != sizeof(struct can_frame)) {
		logMessage("CANSocket::%s: sent incomplete CAN frame (ret = %d")
				% __func__ % ret;
		return 2;
	}

	return 0;
}

int CANSocket::receiveRaw(int& busId, unsigned char* data, size_t& len, bool blocking) const
{
	BARRETT_SCOPED_LOCK(mutex);

	struct can_frame frame;
	int ret = recv(handle->h, (void *) &frame, sizeof(struct can_frame), blocking ? 0 : MSG_DONTWAIT);

	if (ret < 0) {
		ret = -errno;  // Specific error info is in errno. Save a copy.

		switch (ret) {
		case -EAGAIN: // -EWOULDBLOCK
			//logMessage("CANSocket::%s: "
			//		"recv(): no data available during non-blocking read")
			//		% __func__;
			return 1;
			break;
		case -ETIMEDOUT:
			logMessage("CANSocket::%s: "
					"recv(): timed out")
					% __func__;
			return 2;
			break;
		case -EBADF:
			logMessage("CANSocket::%s: "
					"recv(): aborted because socket was closed")
					% __func__;
			return 2;
			break;
		default:
			logMessage("CANSocket::%s: "
					"recv(): (%d) %s")
					% __func__ % -ret % strerror(-ret);
			return 2;
			break;
		}
	} else if (ret != sizeof(struct can_frame)) {
		logMessage("CANSocket::%s: received incomplete CAN frame (ret = %d")
				% __func__ % ret;
		return 2;
	} else if (frame.can_id & CAN_ERR_FLAG) {
		logMessage("CANSocket::%s: CAN_ERR_FLAG was set") % __func__;
		return 2;
	}

	busId = frame.can_id;
	len = frame.can_dlc;
	memcpy(data, frame.data, len);

	return 0;
}


}
}
