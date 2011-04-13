/*
 * real_time_writer-inl.h
 *
 *  Created on: Dec 30, 2009
 *      Author: dc
 */


#include <stdexcept>
#include <algorithm>
#include <fstream>

#include <unistd.h>  // usleep

#include <boost/bind.hpp>
#include <boost/thread.hpp>


namespace barrett {
namespace log {


template<typename T, typename Traits>
RealTimeWriter<T, Traits>::RealTimeWriter(const char* fileName, double recordPeriod_s) :
	Writer<T, Traits>(fileName), period(0), singleBufferSize(0),
	inBuff(NULL), outBuff(NULL), endInBuff(NULL), endOutBuff(NULL), currentPos(NULL), writeToDisk(false),
	stopRunning(false), thread()
{
	if (this->recordLength > 1024) {
		throw(std::logic_error("(log::RealTimeWriter::RealTimeWriter()): This constructor was not designed for records this big."));
	}

	// keep the single buffer size below 16KB
	size_t recordsInSingleBuffer = 16384 / this->recordLength;

	// with a factor of safety of 5, how many microseconds to fill a single buffer?
	period = static_cast<size_t>( (1e6 * recordsInSingleBuffer * recordPeriod_s) / 5.0 );
	if (period < 3000) {
		throw(std::logic_error("(log::RealTimeWriter::RealTimeWriter()): This constructor was not designed for data rates this high."));
	}
	period = std::min(period, (size_t)1000000u);  // limit period to a maximum of 1 second

	init(recordsInSingleBuffer);
}

template<typename T, typename Traits>
RealTimeWriter<T, Traits>::RealTimeWriter(const char* fileName, size_t approxPeriod_us, size_t recordsInSingleBuffer) :
	Writer<T, Traits>(fileName), period(approxPeriod_us),
	inBuff(NULL), outBuff(NULL), endInBuff(NULL), endOutBuff(NULL), currentPos(NULL), writeToDisk(false),
	stopRunning(false), thread()
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
	while ( !stopRunning ) {
		usleep(period);

		if (writeToDisk) {
			this->file.write(outBuff, singleBufferSize);
			writeToDisk = false;
		}
	}
}


}
}
