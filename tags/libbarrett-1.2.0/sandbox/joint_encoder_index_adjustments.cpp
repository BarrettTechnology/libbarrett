/*
 * joint_encoder_index_adjustments.cpp
 *
 *  Created on: Jun 15, 2012
 *      Author: dc
 */

#include <cstdio>
#include <cstring>

#include <barrett/products/product_manager.h>


using namespace barrett;


int main(int argc, char** argv) {
	ProductManager pm;
	pm.waitForWam(false);
	pm.wakeAllPucks();

	if (argc == 2  &&  strcmp(argv[1], "--reset") == 0) {
		for (size_t i = 0; i < pm.getWamPucks().size(); ++i) {
			if (pm.getWamPucks()[i] != NULL) {
				pm.getWamPucks()[i]->setProperty(Puck::ECMIN, 0);
				pm.getWamPucks()[i]->setProperty(Puck::ECMAX, 0);
			}
		}
	} else {
		for (size_t i = 0; i < pm.getWamPucks().size(); ++i) {
			if (pm.getWamPucks()[i] != NULL) {
				printf("Puck %d: ECMIN=%d\t\tECMAX=%d\n",
						pm.getWamPucks()[i]->getId(),
						pm.getWamPucks()[i]->getProperty(Puck::ECMIN),
						pm.getWamPucks()[i]->getProperty(Puck::ECMAX));
			}
		}
	}

	return 0;
}
