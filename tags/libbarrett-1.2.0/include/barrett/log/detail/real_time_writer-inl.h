/*
	Copyright 2009, 2010, 2011, 2012 Barrett Technology <support@barrett.com>

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
 * real_time_writer-inl.h
 *
 *  Created on: Dec 30, 2009
 *      Author: dc
 */


#include <stdexcept>
#include <algorithm>
#include <fstream>
#include <string>

#include <native/task.h>

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
	stopRunning(false), thread(), priority(priority_)
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
	stopRunning(false), thread(), priority(priority_)
{
	init(recordsInSingleBuffer);
}

template<typename T, typename Traits>
void RealTimeWriter<T, Traits>::init(size_t recordsInSingleBuffer) {
	singleBufferSize = this->recordLength * recordsInSingleBuffer;

	delete[] this->buffer;
	this->buffer = new char[singleBufferSize * 2];  // log::Writer's dtor will delete this for us.

	inBuff = this->buffer;
	outBuff = inBuff + singleBufferSize;
	endInBuff = inBuff + singleBufferSize;
	endOutBuff = outBuff + singleBufferSize;
	currentPos = inBuff;
	writeToDisk = false;

	// start writing thread
	boost::thread tmpThread(boost::bind(&RealTimeWriter<T, Traits>::writeToDiskEntryPoint, this));
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
	stopRunning = true;
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
void RealTimeWriter<T, Traits>::writeToDiskEntryPoint()
{
	std::string name = "RTW: " + boost::lexical_cast<std::string>(this);

	int ret;

	ret = rt_task_shadow(NULL, name.c_str(), priority, 0);
	if (ret != 0) {
		logMessage("log::RealTimeWriter::%s: rt_task_shadow(): (%d) %s")
				% __func__ % -ret % strerror(-ret);
		exit(2);
	}
	ret = rt_task_set_periodic(NULL, TM_NOW, static_cast<RTIME>(period * 1e9 + 0.5));
	if (ret != 0) {
		logMessage("log::RealTimeWriter::%s: rt_task_set_periodic(): (%d) %s")
				% __func__ % -ret % strerror(-ret);
		exit(2);
	}


	while ( !stopRunning ) {
		ret = rt_task_wait_period(NULL);
		if (ret != 0  &&  ret != -ETIMEDOUT) {  // ETIMEDOUT means that we missed a release point
			logMessage("log::RealTimeWriter::%s: rt_task_wait_period(): (%d) %s")
					% __func__ % -ret % strerror(-ret);
			exit(2);
		}

		if (writeToDisk) {
			this->file.write(outBuff, singleBufferSize);
			writeToDisk = false;
		}
	}
}


}
}
