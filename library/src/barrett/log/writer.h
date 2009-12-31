/*
 * writer.h
 *
 *  Created on: Dec 29, 2009
 *      Author: dc
 */

#ifndef WRITER_H_
#define WRITER_H_


#include <fstream>

#include "../detail/ca_macro.h"
#include "./detail/traits.h"


namespace barrett {
namespace log {


template<typename T, typename Traits = detail::Traits<T> >
class Writer {
public:
	typedef typename Traits::parameter_type parameter_type;
	typedef typename Traits::pointer_type pointer_type;


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
#include "./detail/writer-inl.h"


#endif /* WRITER_H_ */
