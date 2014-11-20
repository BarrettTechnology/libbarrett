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
 * @file reader-inl.h
 * @date 12/31/2009
 * @author Dan Cody
 *
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
