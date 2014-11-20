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
 * @file communications_bus.h
 * @date 08/26/2010
 * @author Dan Cody
 *
 */

#ifndef BARRETT_BUS_ABSTRACT_COMMUNICATIONS_BUS_H_
#define BARRETT_BUS_ABSTRACT_COMMUNICATIONS_BUS_H_


#include <barrett/thread/abstract/mutex.h>


namespace barrett {
namespace bus {


class CommunicationsBus {
public:
	static const size_t MAX_MESSAGE_LEN = 8;  /** The maximum of any of the available communications buses */
	static const double TIMEOUT = 1.0;  /** Bus connection timeout limit in seconds */

	virtual ~CommunicationsBus() {} /** Destructor */

	virtual thread::Mutex& getMutex() const = 0;

	virtual void open(int port) = 0;
	virtual void close() = 0;
	virtual bool isOpen() const = 0;

	virtual int send(int busId, const unsigned char* data, size_t len) const = 0;
	virtual int receive(int expectedBusId, unsigned char* data, size_t& len, bool blocking = true, bool realtime = false) const;
	virtual int receiveRaw(int& busId, unsigned char* data, size_t& len, bool blocking = true) const = 0;
};


}
}


#endif /* BARRETT_BUS_ABSTRACT_COMMUNICATIONS_BUS_H_ */
