#ifndef WAM_EXCEPTION_HPP
#define WAM_EXCEPTION_HPP

#include <exception>

using namespace std;
   
enum WamExceptionType
{
   CANT_OPEN_WAM,
   TEACH_ERROR,
   PLAYBACK_ERROR,
   THREAD_ERROR
   
}; 

class WamException: public exception
{
public:
   WamException(WamExceptionType type, char * message) throw (): 
                                 type(type), message(message)
   {}
   virtual ~WamException() throw()
   {}
      
   WamExceptionType getType()
   {
      return type;
   }
   
   virtual const char * what() const throw()
   {
      return message;
   }


private:
   WamExceptionType type;
   char * message;
   
   
};

#endif

