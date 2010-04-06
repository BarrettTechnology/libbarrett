/*
 * reader.h
 *
 *  Created on: Dec 29, 2009
 *      Author: dc
 */

#ifndef READER_H_
#define READER_H_


#include <fstream>
#include "../detail/ca_macro.h"
#include "./traits.h"


namespace barrett {
namespace log {


template<typename T, typename Traits = Traits<T> >
class Reader {
public:
	typedef typename Traits::parameter_type parameter_type;

	Reader(const char* fileName);
	~Reader();

	size_t numRecords() const;
	T getRecord();
	void close();

protected:
	std::ifstream file;
	size_t recordLength, recordCount;
	char* buffer;

private:
	DISALLOW_COPY_AND_ASSIGN(Reader);
};


}
}


// include template definitions
#include "./detail/reader-inl.h"


#endif /* READER_H_ */
