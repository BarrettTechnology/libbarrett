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
 * can_socket-rtcan.cpp
 *
 *  Created on: Aug 18, 2010
 *      Author: dc
 */

#include <stdexcept>
#include <cstdio>
#include <cstring>

#include <errno.h>

#include <rtdm/rtcan.h>

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

	logMessage("CANSocket::open(%d) using RTCAN driver") % port;

	int ret;

	ret = rt_dev_socket(PF_CAN, SOCK_RAW, CAN_RAW);
	if (ret < 0) {
		close();
		(logMessage("CANSocket::%s(): Could not open CAN port. rt_dev_socket(): (%d) %s")
				% __func__ % -ret % strerror(-ret)).raise<std::runtime_error>();
	}
	handle->h = ret;

	struct ifreq ifr;
	char devname[10];
	sprintf(devname, "rtcan%d", port);
	strncpy(ifr.ifr_name, devname, IFNAMSIZ);

	ret = rt_dev_ioctl(handle->h, SIOCGCANSTATE, &ifr);
	if (ret != 0) {
		close();
		(logMessage("CANSocket::%s(): Could not open CAN port. rt_dev_ioctl(SIOCGCANSTATE): (%d) %s")
				% __func__ % -ret % strerror(-ret)).raise<std::runtime_error>();
	} else {
		// Note: The Xenomai documentation says that "ifr_ifru will be filled
		// with an instance of can_mode_t." This is a documentation bug. In
		// ksrc/drivers/can/rtcan_raw_dev.c ifr_ifru actually gets filled with
		// in instance of can_state_t, which makes a lot more sense.
		can_state_t* state = (can_state_t*) &ifr.ifr_ifru;
		if (*state != CAN_STATE_ACTIVE) {
			const int NUM_STATES = 8;
			const char canStateStrs[NUM_STATES][18] = {
					"active",
					"warning",
					"passive",
					"bus-off",
					"scanning-baudrate",
					"stopped",
					"sleeping",
					"unknown state"
			};

			int index = *state;
			if (index < 0  ||  index >= NUM_STATES) {
				index = NUM_STATES - 1;
			}
			logMessage("  WARNING: CAN_STATE is %s (%d)") % canStateStrs[index] % *state;

			if (*state == CAN_STATE_BUS_OFF) {
				logMessage("  Setting CAN_MODE = CAN_MODE_START");

				can_mode_t* mode = (can_mode_t*) &ifr.ifr_ifru;
				*mode = CAN_MODE_START;
				ret = rt_dev_ioctl(handle->h, SIOCSCANMODE, &ifr);
				if (ret != 0) {
					close();
					(logMessage("CANSocket::%s(): Could not open CAN port. rt_dev_ioctl(SIOCSCANMODE): (%d) %s")
							% __func__ % -ret % strerror(-ret)).raise<std::runtime_error>();
				}

				// According to Xenomai documentation, the transition from
				// bus-off to active takes at least 128 * 11 * 1e-6 = 0.0014
				// seconds and there's no way to detect when it's done :(
				btsleep(0.1);
			}
		}
	}

	struct can_filter recvFilter[1];
	recvFilter[0].can_id = (Puck::HOST_ID << Puck::NODE_ID_WIDTH) | CAN_INV_FILTER;
	recvFilter[0].can_mask = Puck::FROM_MASK;
	ret = rt_dev_setsockopt(handle->h, SOL_CAN_RAW, CAN_RAW_FILTER, &recvFilter, sizeof(recvFilter));
	if (ret != 0) {
		close();
		(logMessage("CANSocket::%s(): Could not open CAN port. rt_dev_setsockopt(CAN_RAW_FILTER): (%d) %s")
				% __func__ % -ret % strerror(-ret)).raise<std::runtime_error>();
	}

	// Note: This must be done after the rt_dev_ioctl(SIOCGCANSTATE) call above,
	// otherwise rt_dev_send() will fail with ret = -6 (No such device or
	// address). The ifr.ifr_index gets overwritten because it is actually a
	// member of the ifr.ifr_ifru union.
	ret = rt_dev_ioctl(handle->h, SIOCGIFINDEX, &ifr);
	if (ret != 0) {
		close();
		(logMessage("CANSocket::%s(): Could not open CAN port. rt_dev_ioctl(SIOCGIFINDEX): (%d) %s")
				% __func__ % -ret % strerror(-ret)).raise<std::runtime_error>();
	}

	struct sockaddr_can toAddr;
	memset(&toAddr, 0, sizeof(toAddr));
	toAddr.can_ifindex = ifr.ifr_ifindex;
	toAddr.can_family = AF_CAN;
	ret = rt_dev_bind(handle->h, (struct sockaddr *) &toAddr, sizeof(toAddr));
	if (ret != 0) {
		close();
		(logMessage("CANSocket::%s(): Could not open CAN port. rt_dev_bind(): (%d) %s")
				% __func__ % -ret % strerror(-ret)).raise<std::runtime_error>();
	}

	nanosecs_rel_t timeout = (nanosecs_rel_t) 1e9 * CommunicationsBus::TIMEOUT;
	ret = rt_dev_ioctl(handle->h, RTCAN_RTIOC_RCV_TIMEOUT, &timeout);
	if (ret != 0) {
		close();
		(logMessage("CANSocket::%s(): Could not open CAN port. rt_dev_ioctl(RCV_TIMEOUT): (%d) %s")
				% __func__ % -ret % strerror(-ret)).raise<std::runtime_error>();
	}
	ret = rt_dev_ioctl(handle->h, RTCAN_RTIOC_SND_TIMEOUT, &timeout);
	if (ret != 0) {
		close();
		(logMessage("CANSocket::%s(): Could not open CAN port. rt_dev_ioctl(SND_TIMEOUT): (%d) %s")
				% __func__ % -ret % strerror(-ret)).raise<std::runtime_error>();
	}
}

void CANSocket::close()
{
	if (isOpen()) {
		rt_dev_close(handle->h);
		handle->h = detail::can_handle::NULL_HANDLE;
	}
}

bool CANSocket::isOpen() const
{
	return handle->isValid();
}

int CANSocket::send(int busId, const unsigned char* data, size_t len) const
{
	boost::unique_lock<thread::RealTimeMutex> ul(mutex);

	struct can_frame frame;
	frame.can_id = busId;
	frame.can_dlc = len;
	memcpy(frame.data, data, len);

	int ret = rt_dev_send(handle->h, (void *) &frame, sizeof(can_frame_t), 0);
	if (ret < 0) {
		ul.unlock();

		switch (ret) {
		case -EAGAIN: // -EWOULDBLOCK
			logMessage("CANSocket::%s: "
					"rt_dev_send(): data would block during non-blocking send (output buffer full)")
					% __func__;
			return 1;
			break;
		case -ETIMEDOUT:
			logMessage("CANSocket::%s: "
					"rt_dev_send(): timed out")
					% __func__;
			return 2;
			break;
		case -EBADF:
			logMessage("CANSocket::%s: "
					"rt_dev_send(): aborted because socket was closed")
					% __func__;
			return 2;
		default:
			logMessage("CANSocket::%s: "
					"rt_dev_send(): (%d) %s")
					% __func__ % -ret % strerror(-ret);
			return 2;
		}
	}

	return 0;
}

int CANSocket::receiveRaw(int& busId, unsigned char* data, size_t& len, bool blocking) const
{
	BARRETT_SCOPED_LOCK(mutex);

	struct can_frame frame;
	int ret = rt_dev_recv(handle->h, (void *) &frame, sizeof(can_frame_t), blocking ? 0 : MSG_DONTWAIT);

	if (ret < 0) {
		switch (ret) {
		case -EAGAIN: // -EWOULDBLOCK
			//logMessage("CANSocket::%s: "
			//		"rt_dev_recv(): no data available during non-blocking read")
			//		% __func__;
			return 1;
			break;
		case -ETIMEDOUT:
			logMessage("CANSocket::%s: "
					"rt_dev_recv(): timed out")
					% __func__;
			return 2;
			break;
		case -EBADF:
			logMessage("CANSocket::%s: "
					"rt_dev_recv(): aborted because socket was closed")
					% __func__;
			return 2;
			break;
		default:
			logMessage("CANSocket::%s: "
					"rt_dev_recv(): (%d) %s")
					% __func__ % -ret % strerror(-ret);
			return 2;
			break;
		}
	}

	busId = frame.can_id;
	len = frame.can_dlc;
	memcpy(data, frame.data, len);

	if (frame.can_id & CAN_ERR_FLAG) {
		if (frame.can_id & CAN_ERR_BUSOFF) {
			logMessage("CANSocket::%s: bus-off") % __func__;
		}
		if (frame.can_id & CAN_ERR_CRTL) {
			logMessage("CANSocket::%s: controller problem") % __func__;
		}
		return 2;
	}

	return 0;
}


}
}
