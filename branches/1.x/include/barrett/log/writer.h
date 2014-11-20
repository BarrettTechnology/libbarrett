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
 * @file writer.h
 * @date 12/29/2009
 * @author Dan Cody
 *
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
