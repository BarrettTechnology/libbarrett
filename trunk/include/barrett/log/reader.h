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
 * @file reader.h
 * @date 12/29/2009
 * @author Dan Cody
 *  
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
	/** Constructor and Destructors for Reader.
	 *
	 */
	Reader(const char* fileName);
	~Reader();

/** numRecords Method returns the number of lines in the file being processed.
 *
 */
	size_t numRecords() const;
/** getRecord Method returns the line of the file currently being processed.
 *
 */
	T getRecord();
/** exportCSV method writes binary data to comma separated text file.
 *
 */
	void exportCSV(const char* outputFileName);
/** exportCSV method writes binary data to comma separated text file.
 *
 */
	void exportCSV(std::ostream& os);
/** close Method destroys file being written to. 
 *
 */
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
