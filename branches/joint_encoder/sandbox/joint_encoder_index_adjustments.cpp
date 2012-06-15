/*
 * joint_encoder_index_adjustments.cpp
 *
 *  Created on: Jun 15, 2012
 *      Author: robot
 */

#include <cstdio>
#include <cstring>

#include <barrett/products/product_manager.h>


using namespace barrett;


int main() {
	ProductManager pm;
	pm.wakeAllPucks();

	for (size_t i = 0; i < pm.getWamPucks().size(); ++i) {
		if (pm.getWamPucks()[i] != NULL) {
			printf("Puck %d: ECMIN=%d\t\tECMAX=%d\n",
					pm.getWamPucks()[i]->getId(),
					pm.getWamPucks()[i]->getProperty(Puck::ECMIN),
					pm.getWamPucks()[i]->getProperty(Puck::ECMAX));
		}
	}

	return 0;
}
