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
 *
 */

/**
 *	@file ca_macro.h
 *  @date 12/2009
 *	copied from: http://greeness2008.blogspot.com/2008/10/google-c-coding-style.html
 */
#ifndef BARRETT_DETAIL_CA_MACRO_H_
#define BARRETT_DETAIL_CA_MACRO_H_

/** Disallow Copy and Assign Macro
 * 
 *	This macro disallows the copy ctor and operator= functions
 * 	and should be used in the private: declarations for a class 
 */
#define DISALLOW_COPY_AND_ASSIGN(TypeName)		\
		TypeName(const TypeName&);				\
		void operator=(const TypeName&)


#endif /* BARRETT_DETAIL_CA_MACRO_H_ */
