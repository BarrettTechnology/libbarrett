/*
 * system-inl.h
 *
 *  Created on: Nov 17, 2009
 *      Author: dc
 */


namespace barrett {
namespace systems {


// TODO(dc): test!

// FIXME(dc): should this be inline?
inline bool System::inputsValid()
{
	std::vector<AbstractInput*>::const_iterator i;
	for (i = inputs.begin(); i != inputs.end(); ++i) {
		if ((*i) != NULL  &&  !(*i)->valueDefined() ) {
			return false;
		}
	}

	return true;
}


}
}
