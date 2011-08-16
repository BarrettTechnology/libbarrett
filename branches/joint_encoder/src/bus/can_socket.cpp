/*
 * can_socket.cpp
 *
 *  Created on: Aug 18, 2010
 *      Author: dc
 */

#include <stdexcept>
#include <cstdio>
#include <cstring>

#include <syslog.h>
#include <errno.h>
#include <sys/mman.h>

#include <rtdm/rtcan.h>

#include <barrett/thread/real_time_mutex.h>
#include <barrett/products/puck.h>
#include <barrett/bus/can_socket.h>


namespace barrett {
namespace bus {


CANSocket::CANSocket() :
	mutex(), handle(NULL_HANDLE)
{
	init();
}

CANSocket::CANSocket(int port) throw(std::runtime_error) :
	mutex(), handle(NULL_HANDLE)
{
	init();
	open(port);
}

CANSocket::~CANSocket()
{
	close();
}


void CANSocket::open(int port) throw(std::logic_error, std::runtime_error)
{
	if (isOpen()) {
		throw std::logic_error("CANSocket::open(): This object is already associated with a CAN port.");
	}

	syslog(LOG_ERR, "CANSocket::open(%d)", port);

	int ret;

	ret = rt_dev_socket(PF_CAN, SOCK_RAW, CAN_RAW);
	if (ret < 0) {
		syslog(LOG_ERR, "  rt_dev_socket(): (%d) %s\n", -ret, strerror(-ret));
		fail();
	}
	handle = ret;

	struct ifreq ifr;
	char devname[10];
	sprintf(devname, "rtcan%d", port);
	strncpy(ifr.ifr_name, devname, IFNAMSIZ);

	ret = rt_dev_ioctl(handle, SIOCGIFINDEX, &ifr);
	if (ret < 0) {
		syslog(LOG_ERR, "  rt_dev_ioctl(SIOCGIFINDEX): (%d) %s\n", -ret, strerror(-ret));
		fail();
	}

	struct can_filter recvFilter[1];
	recvFilter[0].can_id = (Puck::HOST_ID << Puck::NODE_ID_WIDTH) | CAN_INV_FILTER;
	recvFilter[0].can_mask = Puck::FROM_MASK;
	ret = rt_dev_setsockopt(handle, SOL_CAN_RAW, CAN_RAW_FILTER, &recvFilter, sizeof(recvFilter));
	if (ret < 0) {
		syslog(LOG_ERR, "  rt_dev_setsockopt(CAN_RAW_FILTER): (%d) %s\n", -ret, strerror(-ret));
		fail();
	}

	struct sockaddr_can toAddr;
	memset(&toAddr, 0, sizeof(toAddr));
	toAddr.can_ifindex = ifr.ifr_ifindex;
	toAddr.can_family = AF_CAN;
	ret = rt_dev_bind(handle, (struct sockaddr *) &toAddr,
			sizeof(toAddr));
	if (ret < 0) {
		syslog(LOG_ERR, "  rt_dev_bind(): (%d) %s\n", -ret, strerror(-ret));
		fail();
	}

	nanosecs_rel_t timeout = (nanosecs_rel_t) CommunicationsBus::TIMEOUT;
	ret = rt_dev_ioctl(handle, RTCAN_RTIOC_RCV_TIMEOUT, &timeout);
	if (ret) {
		syslog(LOG_ERR, "  rt_dev_ioctl(RCV_TIMEOUT): (%d) %s\n", -ret, strerror(-ret));
		fail();
	}
	ret = rt_dev_ioctl(handle, RTCAN_RTIOC_SND_TIMEOUT, &timeout);
	if (ret) {
		syslog(LOG_ERR, "  rt_dev_ioctl(SND_TIMEOUT): (%d) %s\n", -ret, strerror(-ret));
		fail();
	}
}

void CANSocket::close()
{
	if (isOpen()) {
		struct ifreq ifr;
		can_mode_t* mode = (can_mode_t *)&ifr.ifr_ifru;
		*mode = CAN_MODE_STOP;
		rt_dev_ioctl(handle, SIOCSCANMODE, &ifr);
		rt_dev_close(handle);

		handle = NULL_HANDLE;
	}
}

int CANSocket::send(int busId, const unsigned char* data, size_t len) const
{
	BARRETT_SCOPED_LOCK(mutex);

	int ret;

	struct can_frame frame;
	frame.can_id = busId;
	frame.can_dlc = len;
	memcpy(frame.data, data, len);

	ret = rt_dev_send(handle, (void *) &frame, sizeof(can_frame_t), 0);
	if (ret < 0) {
		switch (ret) {
		case -EAGAIN: // -EWOULDBLOCK
			syslog(LOG_ERR,
					"%s: rt_dev_send(): data would block during non-blocking send (output buffer full)",
					__func__);
			return 1;
			break;
		case -ETIMEDOUT:
			syslog(LOG_ERR, "%s: rt_dev_send(): timed out", __func__);
			return 2;
			break;
		case -EBADF:
			syslog(LOG_ERR,
					"%s: rt_dev_send(): aborted because socket was closed",
					__func__);
			return 2;
		default:
			syslog(LOG_ERR, "%s: rt_dev_send(): (%d) %s\n", __func__, -ret, strerror(-ret));
			return 2;
		}
	}

	return 0;
}

int CANSocket::receiveRaw(int& busId, unsigned char* data, size_t& len, bool blocking) const
{
	BARRETT_SCOPED_LOCK(mutex);

	struct can_frame frame;
	int ret = rt_dev_recv(handle, (void *) &frame, sizeof(can_frame_t), blocking ? 0 : MSG_DONTWAIT);

	if (ret < 0) {
		switch (ret) {
		case -EAGAIN: // -EWOULDBLOCK
			//syslog(LOG_ERR, "%s: rt_dev_recv(): no data available during non-blocking read", __func__);
			return 1;
			break;
		case -ETIMEDOUT:
			syslog(LOG_ERR, "%s: rt_dev_recv(): timed out", __func__);
			return 2;
			break;
		case -EBADF:
			syslog(LOG_ERR,
					"%s: rt_dev_recv(): aborted because socket was closed",
					__func__);
			return 2;
			break;
		default:
			syslog(LOG_ERR, "%s: rt_dev_recv(): (%d) %s\n", __func__, -ret, strerror(-ret));
			return 2;
			break;
		}
	}

	busId = frame.can_id;
	len = frame.can_dlc;
	memcpy(data, frame.data, len);

	if (frame.can_id & CAN_ERR_FLAG) {
		if (frame.can_id & CAN_ERR_BUSOFF) {
			syslog(LOG_ERR, "%s: bus-off", __func__);
		}
		if (frame.can_id & CAN_ERR_CRTL) {
			syslog(LOG_ERR, "%s: controller problem", __func__);
		}
		return 2;
	}

	return 0;
}

void CANSocket::fail() throw(std::runtime_error)
{
	close();
	syslog(LOG_ERR, "CANSocket::open() failed.");
	throw std::runtime_error("CANSocket::open(): Could not open CAN port. Check /var/log/syslog for details.");
}

void CANSocket::init() {
	// Avoids memory swapping for this program
	mlockall(MCL_CURRENT|MCL_FUTURE);
}


}
}
