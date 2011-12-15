/*
 * writer.h
 *
 *  Created on: Dec 29, 2009
 *      Author: dc
 */

#ifndef BARRETT_LOG_WRITER_H_
#define BARRETT_LOG_WRITER_H_


#include <fstream>

#include <barrett/detail/ca_macro.h>
#include <barrett/log/traits.h>


namespace barrett {
namespace log {


template<typename T, typename Traits = Traits<T> >
class Writer {
public:
	typedef typename Traits::parameter_type parameter_type;

	explicit Writer(const char* fileName);
	~Writer();

	void putRecord(parameter_type data);
	void close();

protected:
	std::ofstream file;
	size_t recordLength;
	char* buffer;

private:
	DISALLOW_COPY_AND_ASSIGN(Writer);
};


}
}


// include template definitions
#include <barrett/log/detail/writer-inl.h>


#endif /* BARRETT_LOG_WRITER_H_ */
