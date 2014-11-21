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
 * @file writer-inl.h
 * @date 12/29/2009
 * @author Dan Cody
 * 
 */


#include <fstream>


namespace barrett {
namespace log {


template<typename T, typename Traits>
Writer<T, Traits>::Writer(const char* fileName) :
	file(fileName, std::ios_base::binary), recordLength(Traits::serializedLength())
{
	if (recordLength == 0) {
		throw(std::logic_error("(log::Writer::Writer): The record length "
				"(Traits::serializedLength()) cannot be zero."));
	}

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
	Traits::serialize(data, buffer);
	file.write(buffer, recordLength);
}

template<typename T, typename Traits>
inline void Writer<T, Traits>::close()
{
	file.close();
}


}
}
