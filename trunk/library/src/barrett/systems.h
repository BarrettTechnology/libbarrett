/*
 * systems.h
 *
 *  Created on: Sep 4, 2009
 *      Author: dc
 */

#ifndef SYSTEMS_H_
#define SYSTEMS_H_


// other -- these operate on Systems, but are not Systems themselves
#include "systems/helpers.h"
#include "systems/converter.h"
#include "systems/io_conversion.h"

// sources
#include "systems/constant.h"
#include "systems/exposed_output.h"

// sinks
#include "systems/print_to_stream.h"

// operators
#include "systems/summer.h"
#include "systems/gain.h"
#include "systems/array_editor.h"
#include "systems/pid_controller.h"
#include "systems/callback.h"

// #include "systems/"


#endif /* SYSTEMS_H_ */
