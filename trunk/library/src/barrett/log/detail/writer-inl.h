/*
 * writer-inl.h
 *
 *  Created on: Dec 29, 2009
 *      Author: dc
 */


#include <fstream>


namespace barrett {
namespace log {


template<typename T, typename Traits>
Writer<T, Traits>::Writer(const char* fileName) :
	file(fileName, std::ios_base::binary), recordLength(Traits::serializedLength())
{
	buffer = new char[recordLength];
}

template<typename T, typename Traits>
Writer<T, Traits>::~Writer()
{
	if (file.is_open()) {
		close();
	}

	delete[] buffer;
	buffer = NULL;
}

template<typename T, typename Traits>
inline void Writer<T, Traits>::putRecord(parameter_type data)
{
	Traits::serialize(data, reinterpret_cast<pointer_type>(buffer));
	file.write(buffer, recordLength);
}

template<typename T, typename Traits>
inline void Writer<T, Traits>::close()
{
	file.close();
}


}
}
