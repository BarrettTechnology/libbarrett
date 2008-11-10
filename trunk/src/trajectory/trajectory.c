/* ======================================================================== *
 *  Module ............. libbt
 *  File ............... trajectory.c
 *  Author ............. Traveler Hauptman
 *                       Brian Zenowich
 *                       Christopher Dellin
 *  Creation Date ...... 2005 Mar 30
 *                                                                          *
 *  **********************************************************************  *
 *                                                                          *
 * Copyright (C) 2008   Barrett Technology <support@barrett.com>
 *
 *  NOTES:
 *
 *  REVISION HISTORY:
 *    2008 Sept 15 - CD
 *      Ported from btsystem to libbt; merged from btstatecontrol and btpath
 *
 * ======================================================================== */

#include "trajectory.h"
#include "interp.h"

#include <gsl/gsl_blas.h>

#include <math.h> /* For sqrt() */

#include <syslog.h>


