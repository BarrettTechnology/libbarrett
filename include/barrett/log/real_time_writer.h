/*
 * real_time_writer.h
 *
 *  Created on: Dec 30, 2009
 *      Author: dc
 */

#ifndef REAL_TIME_WRITER_H_
#define REAL_TIME_WRITER_H_


#include <boost/thread.hpp>

#include "../detail/ca_macro.h"
#include "./traits.h"
#include "./writer.h"


namespace barrett {
namespace log {


// A log writer that is real-time safe. The data is double-buffered and is written to disk in a separate thread.
template<typename T, typename Traits = Traits<T> >
class RealTimeWriter : public Writer<T, Traits> {
public:
	typedef typename Writer<T, Traits>::parameter_type parameter_type;

	RealTimeWriter(const char* fileName, double recordPeriod_s);
	RealTimeWriter(const char* fileName, size_t approxPeriod_us, size_t recordsInSingleBuffer);
	~RealTimeWriter();

	void putRecord(parameter_type data);
	void close();

protected:
	class Runnable {
	public:
		Runnable(RealTimeWriter<T, Traits>* writer) :
			writer(writer) {}
		void operator() ();

	private:
		RealTimeWriter<T, Traits>* writer;
	};


	void init(size_t recordsInSingleBuffer);

	size_t period, singleBufferSize;
	char* inBuff;
	char* outBuff;
	char* endInBuff;
	char* endOutBuff;
	char* currentPos;
	bool writeToDisk;

	bool stopRunning;
	boost::thread thread;

private:
	DISALLOW_COPY_AND_ASSIGN(RealTimeWriter);
};


}
}


// include template definitions
#include "./detail/real_time_writer-inl.h"


#endif /* REAL_TIME_WRITER_H_ */
