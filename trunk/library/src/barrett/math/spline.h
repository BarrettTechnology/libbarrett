/*
 * spline.h
 *
 *  Created on: Dec 17, 2009
 *      Author: dc
 */


#ifndef SPLINE_H_
#define SPLINE_H_


#include "../detail/ca_macro.h"
//#include <barrett/spline/spline.h>


// forward declarations from <barrett/spline/spline.h>
struct bt_spline;
enum bt_spline_mode
{
   BT_SPLINE_MODE_ARCLEN, /* The spline uses computed arc-length */
   BT_SPLINE_MODE_EXTERNAL /* The spline uses an external parameter */
};



namespace barrett {
namespace math {


template<typename T>
class Spline {
public:
	Spline() {}


protected:
	struct bt_spline* impl;

private:
	// TODO(dc): write a real copy constructor and assignment operator?
	DISALLOW_COPY_AND_ASSIGN(Spline);
};


}
}


#endif /* SPLINE_H_ */
