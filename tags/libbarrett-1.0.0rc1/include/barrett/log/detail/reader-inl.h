/*
 * reader-inl.h
 *
 *  Created on: Dec 31, 2009
 *      Author: dc
 */


#include <stdexcept>
#include <sstream>
#include <fstream>


namespace barrett {
namespace log {


template<typename T, typename Traits>
Reader<T, Traits>::Reader(const char* fileName) :
	file(fileName, std::ios_base::binary), recordLength(Traits::serializedLength()), recordCount(0)
{
	if (recordLength == 0) {
		throw(std::logic_error("(log::Reader::Reader): The record length "
				"(Traits::serializedLength()) cannot be zero."));
	}

	file.seekg(0, std::ifstream::end);
	long size = file.tellg();

	if (size % recordLength != 0) {
		std::stringstream ss;
		ss << "(log::Reader::Reader): The file '" << fileName
				<< "' is corrupted or does not contain this type of data. Its "
				"size is not evenly divisible by the record length ("
				<< recordLength << " bytes).";
		throw(std::runtime_error(ss.str()));
	}
	recordCount = size / recordLength;
	file.seekg(0);

	buffer = new char[recordLength];
}

template<typename T, typename Traits>
Reader<T, Traits>::~Reader()
{
	if (file.is_open()) {
		close();
	}

	delete[] buffer;
	buffer = NULL;
}

template<typename T, typename Traits>
inline size_t Reader<T, Traits>::numRecords() const
{
	return recordCount;
}

template<typename T, typename Traits>
inline T Reader<T, Traits>::getRecord()
{
	file.read(buffer, recordLength);

	if (file.eof()) {
		throw(std::underflow_error("(log::Reader::getRecord()): The end of the file was reached. There are no more records to read."));
	}

	return Traits::unserialize(buffer);
}

template<typename T, typename Traits>
inline void Reader<T, Traits>::exportCSV(const char* outputFileName)
{
	std::ofstream ofs(outputFileName);
	exportCSV(ofs);
	ofs.close();
}

template<typename T, typename Traits>
void Reader<T, Traits>::exportCSV(std::ostream& os)
{
	try {
		for (size_t i = 0; i < numRecords(); ++i) {
			Traits::asCSV(getRecord(), os);
			os << std::endl;
		}
	} catch (std::underflow_error) {}
}

template<typename T, typename Traits>
inline void Reader<T, Traits>::close()
{
	file.close();
}


}
}
