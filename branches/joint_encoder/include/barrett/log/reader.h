/*
 * reader.h
 *
 *  Created on: Dec 29, 2009
 *      Author: dc
 */

#ifndef BARRETT_LOG_READER_H_
#define BARRETT_LOG_READER_H_


#include <fstream>
#include <barrett/detail/ca_macro.h>
#include <barrett/log/traits.h>


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
	void exportCSV(const char* outputFileName);
	void exportCSV(std::ostream& os);

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
#include <barrett/log/detail/reader-inl.h>


#endif /* BARRETT_LOG_READER_H_ */
