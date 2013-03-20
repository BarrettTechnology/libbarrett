/*
 * joint_encoder_set_offsets.cpp
 *
 *  Created on: Jun 19, 2012
 *      Author: dc
 */

#include <cstdio>
#include <cmath>
#include <cassert>

#include <boost/lexical_cast.hpp>

#include <barrett/math.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>

#define BARRETT_SMF_VALIDATE_ARGS
#define BARRETT_SMF_DONT_WAIT_FOR_SHIFT_ACTIVATE
#include <barrett/standard_main_function.h>


using namespace barrett;


bool validate_args(int argc, char** argv) {
	if (argc == 1) {
		printf("Usage:\n");
		printf("  %s <zero angle 1> <zero angle 2> ...\n", argv[0]);
		printf("  %s --reset\n", argv[0]);
		return false;
	}

	return true;
}

template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
	assert(wam.getLowLevelWam().hasJointEncoders());


	if (argc == 2  &&  strcmp(argv[1], "--reset") == 0) {
		for (size_t i = 0; i < pm.getWamPucks().size(); ++i) {
			if (pm.getWamPucks()[i] != NULL  &&  pm.getWamPucks()[i]->hasOption(Puck::RO_OpticalEncOnEnc)) {
				pm.getWamPucks()[i]->setProperty(Puck::JOFST, 0);
				pm.getWamPucks()[i]->saveProperty(Puck::JOFST);
			}
		}

		return 0;
	}


	int angleIdx = 0;
	char* zeroStr = NULL;
	const v_type& je2jp = wam.getLowLevelWam().getJointEncoderToJointPositionTransform();
	try {
		for (size_t i = 0; i < pm.getWamPucks().size(); ++i) {
			if (pm.getWamPucks()[i] != NULL  &&  pm.getWamPucks()[i]->hasOption(Puck::RO_OpticalEncOnEnc)) {
				assert(je2jp[i] > 0.0);

				zeroStr = argv[angleIdx + 1];
				if (zeroStr[strlen(zeroStr) - 1] == ',') {
					zeroStr[strlen(zeroStr) - 1] = '\0';
				}
				double zeroAngle = boost::lexical_cast<double>(zeroStr);
				int zeroCounts = floor(zeroAngle / je2jp[i] + 0.5);
				int offset = pm.getWamPucks()[i]->getProperty(Puck::JOFST);

				printf("%f, %f, %d, %d\n", zeroAngle, je2jp[i], zeroCounts, offset);

				pm.getWamPucks()[i]->setProperty(Puck::JOFST, offset - zeroCounts);
				pm.getWamPucks()[i]->saveProperty(Puck::JOFST);

				angleIdx++;
			}
		}
	} catch (boost::bad_lexical_cast& e) {
		printf("ERROR: \"%s\" is not a number\n", zeroStr);
		return 1;
	}

	return 0;
}
