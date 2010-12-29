#ifndef BARRETT_DETAIL_CA_MACRO_H_
#define BARRETT_DETAIL_CA_MACRO_H_

// copied from: http://greeness2008.blogspot.com/2008/10/google-c-coding-style.html
// originally from Google?
//
// A macro to disallow the copy ctor and operator= functions
// This should be used in the private: declarations for a class
#define DISALLOW_COPY_AND_ASSIGN(TypeName)		\
		TypeName(const TypeName&);				\
		void operator=(const TypeName&)


#endif /* BARRETT_DETAIL_CA_MACRO_H_ */
