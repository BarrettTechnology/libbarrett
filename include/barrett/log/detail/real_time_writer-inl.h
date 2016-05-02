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
 * @file real_time_writer-inl.h
 * @date 12/30/2009
 * @author Dan Cody
 *  
 */


#include <stdexcept>
#include <algorithm>
#include <fstream>
#include <string>

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/lexical_cast.hpp>

#include <barrett/os.h>


namespace barrett {
namespace log {


template<typename T, typename Traits>
RealTimeWriter<T, Traits>::RealTimeWriter(const char* fileName, double recordPeriod_s, int priority_) :
	Writer<T, Traits>(fileName), period(0.0), singleBufferSize(0),
	inBuff(NULL), outBuff(NULL), endInBuff(NULL), endOutBuff(NULL), currentPos(NULL), writeToDisk(false),
	thread(), priority(priority_)
{
	if (this->recordLength > 1024) {
		throw(std::logic_error("(log::RealTimeWriter::RealTimeWriter()): This constructor was not designed for records this big."));
	}

	// keep the single buffer size below 16KB
	size_t recordsInSingleBuffer = 16384 / this->recordLength;

	// with a factor of safety of 5, how many seconds to fill a single buffer?
	period = (recordsInSingleBuffer * recordPeriod_s) / 5.0;
	if (period < 0.003) {
		throw(std::logic_error("(log::RealTimeWriter::RealTimeWriter()): This constructor was not designed for data rates this high."));
	}
	period = std::min(period, 1.0);  // limit period to a maximum of 1 second

	init(recordsInSingleBuffer);
}

template<typename T, typename Traits>
RealTimeWriter<T, Traits>::RealTimeWriter(const char* fileName, double approxPeriod_s, size_t recordsInSingleBuffer, int priority_) :
	Writer<T, Traits>(fileName), period(approxPeriod_s),
	inBuff(NULL), outBuff(NULL), endInBuff(NULL), endOutBuff(NULL), currentPos(NULL), writeToDisk(false),
	thread(), priority(priority_)
{
	init(recordsInSingleBuffer);
}

template<typename T, typename Traits>
void RealTimeWriter<T, Traits>::init(size_t recordsInSingleBuffer) {
	singleBufferSize = this->recordLength * recordsInSingleBuffer;
        const char *thread_name = "Real Time Writer";

	delete[] this->buffer;
	this->buffer = new char[singleBufferSize * 2];  // log::Writer's dtor will delete this for us.

	inBuff = this->buffer;
	outBuff = inBuff + singleBufferSize;
	endInBuff = inBuff + singleBufferSize;
	endOutBuff = outBuff + singleBufferSize;
	currentPos = inBuff;
	writeToDisk = false;

	// start writing thread
	boost::thread tmpThread(boost::bind(&RealTimeWriter<T, Traits>::writeToDiskEntryPoint, this, thread_name));
	thread.swap(tmpThread);
}

template<typename T, typename Traits>
RealTimeWriter<T, Traits>::~RealTimeWriter()
{
	if (this->file.is_open()) {
		close();
	}
}

template<typename T, typename Traits>
void RealTimeWriter<T, Traits>::putRecord(parameter_type data)
{
	Traits::serialize(data, currentPos);
	currentPos += this->recordLength;

	if (currentPos >= endInBuff) {
		if (writeToDisk) {
			throw(std::overflow_error("(log::RealTimeWriter::putRecord()): The input buffer filled up before the output buffer was written to disk."));
		}

		std::swap(inBuff, outBuff);
		std::swap(endInBuff, endOutBuff);
		currentPos = inBuff;
		writeToDisk = true;
	}
}

template<typename T, typename Traits>
void RealTimeWriter<T, Traits>::close()
{
	thread.interrupt();
	thread.join();

	if (writeToDisk) {
		this->file.write(outBuff, singleBufferSize);
		writeToDisk = false;
	}
	if (currentPos != inBuff) {
		this->file.write(inBuff, currentPos - inBuff);
	}

	this->Writer<T, Traits>::close();
}

template<typename T, typename Traits>
void RealTimeWriter<T, Traits>::writeToDiskEntryPoint(const char *thread_name)
{
        PeriodicLoopTimer loopTimer(period, thread_name, priority);
	while ( !boost::this_thread::interruption_requested() ) {
		loopTimer.wait();
		if (writeToDisk) {
			this->file.write(outBuff, singleBufferSize);
			writeToDisk = false;
		}
	}
}


}
}
