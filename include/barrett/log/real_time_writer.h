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
 */

/**
 * @file real_time_writer.h
 * @date 12/30/2009
 * @author Dan Cody
 * 
 */

#ifndef BARRETT_LOG_REAL_TIME_WRITER_H_
#define BARRETT_LOG_REAL_TIME_WRITER_H_


#include <boost/thread.hpp>

#include <barrett/detail/ca_macro.h>
#include <barrett/log/traits.h>
#include <barrett/log/writer.h>


namespace barrett {
namespace log {


// A log writer that is real-time safe. The data is double-buffered and is written to disk in a separate thread.
template<typename T, typename Traits = Traits<T> >
class RealTimeWriter : public Writer<T, Traits> {
public:
	typedef typename Writer<T, Traits>::parameter_type parameter_type;
	static const int DEFAULT_PRIORITY = 20;

	RealTimeWriter(const char* fileName, double recordPeriod_s, int priority_ = DEFAULT_PRIORITY);
	RealTimeWriter(const char* fileName, double approxPeriod_s, size_t recordsInSingleBuffer, int priority_ = DEFAULT_PRIORITY);
	~RealTimeWriter();

	void putRecord(parameter_type data);
	void close();

protected:
	void init(size_t recordsInSingleBuffer);
	void writeToDiskEntryPoint(const char *);

	double period;
	size_t singleBufferSize;
	char* inBuff;
	char* outBuff;
	char* endInBuff;
	char* endOutBuff;
	char* currentPos;
	bool writeToDisk;

	boost::thread thread;
	int priority;

private:
	DISALLOW_COPY_AND_ASSIGN(RealTimeWriter);
};


}
}


// include template definitions
#include <barrett/log/detail/real_time_writer-inl.h>


#endif /* BARRETT_LOG_REAL_TIME_WRITER_H_ */
