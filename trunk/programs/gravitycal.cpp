/*
	Copyright 2009, 2010, 2011, 2012 Barrett Technology <support@barrett.com>

	This file is part of libbarrett.

	This version of libbarrett is free software: you can redistribute it
	and/or modify it under the terms of the GNU General Public License as
	published by the Free Software Foundation, either version 3 of the
	License, or (at your option) any later version.

	This version of libbarrett is distributed in the hope that it will be
	useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License along
	with this version of libbarrett.  If not, see
	<http://www.gnu.org/licenses/>.

	Further, non-binding information about licensing is available at:
	<http://wiki.barrett.com/libbarrett/wiki/LicenseNotes>
*/

/*
 * gravitycal.cpp
 *
 *  Created on: Apr 9, 2010
 *      Author: cd
 *      Author: dc
 */

#include <math.h> /* For sqrt() */

#include <signal.h>
#include <syslog.h>
#include <unistd.h>

#include <curses.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_linalg.h>
#include <boost/tuple/tuple.hpp>

#include <barrett/exception.h>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>

#include <barrett/os.h>
#include <barrett/cdlbt/gsl.h>
#include <barrett/cdlbt/kinematics.h>
#include <barrett/cdlbt/calgrav.h>

#define BARRETT_SMF_VALIDATE_ARGS
#include <barrett/standard_main_function.h>

#include "utils.h"


using namespace barrett;

const char CAL_CONFIG_FILE[] = "/etc/barrett/calibration.conf";
const char DATA_CONFIG_FILE[] = "/etc/barrett/calibration_data/%s/gravitycal.conf";


// Print this before the WAM is activated.
bool validate_args(int argc, char** argv) {
	printf(
"\n"
"                *** Barrett WAM Gravity Calibration Utility ***\n"
"\n"
"This utility will calculate cumulative first moment of mass data for each link\n"
"of your WAM Arm. This data is used by the gravity compensation routine to\n"
"support WAM's weight in gravity. The program will move the WAM to several\n"
"predefined positions and take torque measurements at each location.\n"
"\n"
"The calculations rely on having accurate kinematic information. Consider\n"
"performing the zero-calibration procedure (bt-wam-zerocal) before proceeding. It\n"
"is also necessary to know how the WAM is oriented relative to gravity, so be\n"
"sure to update the \"world_to_base\" transform if your WAM is not mounted in the\n"
"standard orientation.\n"
"\n"
"\n"
"IMPORTANT: DO NOT TOUCH the WAM during the measurement process, or the\n"
"calibration computations will be significantly wrong, and any subsequent gravity\n"
"compensation will fail spectacularly!\n"
"\n"
"\n"
	);

	return true;
}


/* ------------------------------------------------------------------------ *
 * Mode Mu-Calibrate                                                        */

#define ANGLE_DIFF (0.03)
#define SLOW_VEL (0.05)
#define SLOW_ACCEL (0.05)
#define NUM_POINTS (2000)
#define CALC_LAMBDA (0.000001)
int mu_n;
/* Big arrays of joint torques and joint positions, for a given pose */
double ** mu_jts; /*[NUM_DOFS][NUM_POINTS];*/
double ** mu_jps; /*[NUM_DOFS][NUM_POINTS];*/

int done;
void sigint(int) {
	done = -1;
}

template <size_t DOF>
int mu_callback(const boost::tuple<typename units::JointTorques<DOF>::type, typename units::JointPositions<DOF>::type>& t) {
	size_t j;
	/* Save the current joint torques for each joint */
	if (mu_n < NUM_POINTS) {
		for (j = 0; j < DOF; j++) {
			mu_jts[j][mu_n] = boost::get<0>(t)[j];
			mu_jps[j][mu_n] = boost::get<1>(t)[j];
		}
		mu_n++;
	}
	return 0;
}

/* Do the averaving */
int mu_stats(gsl_vector * torques, gsl_vector * positions) {
	int n;
	int j;
	gsl_vector * torque_vars;
	gsl_vector * position_vars;
	int dof;

	dof = torques->size;

	torque_vars = gsl_vector_alloc(dof);
	position_vars = gsl_vector_alloc(dof);

	/* Zero the values */
	gsl_vector_set_zero(torques);
	gsl_vector_set_zero(positions);
	gsl_vector_set_zero(torque_vars);
	gsl_vector_set_zero(position_vars);

	/* Calculate the average */
	for (n = 0; n < NUM_POINTS; n++) {
		for (j = 0; j < dof; j++) {
			*(gsl_vector_ptr(torques, j)) += mu_jts[j][n];
			*(gsl_vector_ptr(positions, j)) += mu_jps[j][n];
		}
	}
	gsl_blas_dscal(1.0 / NUM_POINTS, torques);
	gsl_blas_dscal(1.0 / NUM_POINTS, positions);

	/* Calculate the variance */
	for (n = 0; n < NUM_POINTS; n++) {
		for (j = 0; j < dof; j++) {
			*(gsl_vector_ptr(torque_vars, j)) +=
					(mu_jts[j][n] - gsl_vector_get(torques, j)) *
					(mu_jts[j][n] - gsl_vector_get(torques, j));
			*(gsl_vector_ptr(position_vars, j)) +=
					(mu_jps[j][n] - gsl_vector_get(positions, j)) *
					(mu_jps[j][n] - gsl_vector_get(positions, j));
		}
	}
	gsl_blas_dscal(1.0 / NUM_POINTS, torque_vars);
	gsl_blas_dscal(1.0 / NUM_POINTS, position_vars);

	/* Print positions */
	mvprintw(12, 0, "  Positions:");
	mvprintw(13, 0, "  (std-dev):");
	mvprintw(14, 0, "    Torques:");
	mvprintw(15, 0, "  (std-dev):");
	for (j = 0; j < dof; j++) {
		mvprintw(12, 13 + 9 * j, "% 08.5f ", gsl_vector_get(positions, j));
		mvprintw(13, 13 + 9 * j, "% 08.5f ", sqrt(gsl_vector_get(position_vars, j)));
		mvprintw(14, 13 + 9 * j, "% 08.5f ", gsl_vector_get(torques, j));
		mvprintw(15, 13 + 9 * j, "% 08.5f ", sqrt(gsl_vector_get(torque_vars, j)));
	}

	gsl_vector_free(torque_vars);
	gsl_vector_free(position_vars);
	return 0;
}

template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	Hand* hand = pm.getHand();

	int err; /* Generic error variable for function calls */
	int j;
	int n;

	/* Stuff that's filled in once */
	gsl_vector ** poses;
	int num_poses;
	int pose;

	/* Stuff that's used once for each pose in measure mode */
	gsl_vector * angle_diff;
	gsl_vector * moveto_vec;
	gsl_vector * torques_top;
	gsl_vector * positions_top;
	gsl_vector * torques_bottom;
	gsl_vector * positions_bottom;

	/* For storing the results from measure mode, one vector per pose  */
	gsl_vector ** torques;
	gsl_vector ** positions;
	/* For storing the results from computation mode, one vector per joint */
	gsl_vector ** mus;

	/* Make a list of phases for each pose*/
	enum PHASE {
		MU_P_START,
		MU_P_TO_TOP,
		MU_P_FROM_TOP,
		MU_P_MEAS_TOP,
		MU_P_TO_BOT,
		MU_P_FROM_BOT,
		MU_P_MEAS_BOT,
		MU_P_DONE
	} phase;

	// The longest string below is 9 characters (including the NULL termination).
	char phasenm[][9] = { "START", "TO_TOP", "FROM_TOP", "MEAS_TOP", "TO_BOT",
			"FROM_BOT", "MEAS_BOT", "DONE" };

	/* Initialize the ncurses screen library */
	initscr();
	cbreak();
	noecho();
	timeout(0);
	clear();

	mvprintw(0, 0, "Starting Gravity Calibration Mode");


	n = DOF;

	/* Grab poses from the configuration file
	 * (fills poses, sets num_poses) */
	{
		char key[256];
		config_t cfg;
		config_setting_t * poses_array;

		config_init(&cfg);
		err = config_read_file(&cfg, CAL_CONFIG_FILE);
		if (err != CONFIG_TRUE) {
			syslog(LOG_ERR, "Calibration configuration file %s not found.", CAL_CONFIG_FILE);
			printf("Calibration configuration file %s not found.\n", CAL_CONFIG_FILE);
			config_destroy(&cfg);
			endwin();
			return -1;
		}

		sprintf(key, "gravitycal.%s", pm.getWamDefaultConfigPath());
		poses_array = config_lookup(&cfg, key);
		if (!poses_array) {
			syslog(LOG_ERR, "Configuration group %s not found.", key);
			printf("Configuration group %s not found.\n", key);
			config_destroy(&cfg);
			endwin();
			return -1;
		}

		num_poses = config_setting_length(poses_array);
		poses = (gsl_vector **) malloc(num_poses * sizeof(gsl_vector *));
		for (pose = 0; pose < num_poses; pose++) {
			poses[pose] = gsl_vector_alloc(n);
			err = bt_gsl_fill_vector_cfgarray(poses[pose],
					config_setting_get_elem(poses_array, pose));
			if (err) {
				syslog(LOG_ERR, "Pose %d not formatted correctly.", pose);
				printf("Pose %d not formatted correctly.\n", pose);
				config_destroy(&cfg);
				endwin();
				return -1;
			}
		}

		config_destroy(&cfg);
	}

	/* Allocate the global variables */
	mu_jts = (double **) malloc(n * sizeof(double *));
	mu_jps = (double **) malloc(n * sizeof(double *));
	for (j = 0; j < n; j++) {
		mu_jts[j] = (double *) malloc(NUM_POINTS * sizeof(double));
		mu_jps[j] = (double *) malloc(NUM_POINTS * sizeof(double));
	}

	mu_n = NUM_POINTS; /* So we're presently not collecting data */

	/* Allocate some vectors */
	angle_diff = gsl_vector_alloc(n);
	gsl_vector_set_all(angle_diff, ANGLE_DIFF);
	moveto_vec = gsl_vector_alloc(n);
	torques_top = gsl_vector_alloc(n);
	positions_top = gsl_vector_alloc(n);
	torques_bottom = gsl_vector_alloc(n);
	positions_bottom = gsl_vector_alloc(n);
	torques = (gsl_vector **) malloc(num_poses * sizeof(gsl_vector *));
	positions = (gsl_vector **) malloc(num_poses * sizeof(gsl_vector *));
	for (pose = 0; pose < num_poses; pose++) {
		torques[pose] = gsl_vector_alloc(n);
		positions[pose] = gsl_vector_alloc(n);
	}
	mus = (gsl_vector **) malloc(n * sizeof(gsl_vector *));
	for (j = 0; j < n; j++)
		mus[j] = gsl_vector_alloc(3);

	// Install callback
	systems::TupleGrouper<jt_type, jp_type> tg;
	systems::Callback<boost::tuple<jt_type, jp_type>, int> muCallback(mu_callback<DOF>);
	pm.getExecutionManager()->startManaging(muCallback);  // Make sure mu_callback() is called every execution cycle

	systems::connect(wam.jtSum.output, tg.template getInput<0>());
	systems::connect(wam.jpOutput, tg.template getInput<1>());
	systems::connect(tg.output, muCallback.input);

	wam.jpController.setControlSignalLimit(jp_type()); // Disable torque saturation because gravity comp isn't on


	/* Start the GUI! */
	pose = 0;
	phase = MU_P_START;

	clear();
	mvprintw(1, 0, "Note: Press [Control+C] at any time to cancel the calibration.");
	mvprintw(2, 0, "DO NOT TOUCH the WAM during the calibration process!");
	done = 0;

	signal(SIGINT, sigint);
	while (!done) {
		char buf[256];

		/* Print current state */
		mvprintw(0,  0, "Current Pose: %d of %d.  ", pose + 1, num_poses);
		mvprintw(0, 30, "Current Phase: %s        ", phasenm[phase]);

		/* Print current joint position and torque */
		mvprintw(4, 0, "      Joint:");
		mvprintw(5, 0, "   Position:");
		mvprintw(6, 0, "     Torque:");
		for (j = 0; j < n; j++) {
			mvprintw(4, 13 + 9 * j, " Joint %d ", j + 1);
			mvprintw(5, 13 + 9 * j, "% 08.5f ", gsl_vector_get(wam.getJointPositions().asGslType(), j));
			mvprintw(6, 13 + 9 * j, "% 08.4f ", gsl_vector_get(wam.llww.input.getValue().asGslType(), j));
		}

		/* Line 9 - Status Updates */
		mvprintw(8, 0, "Current Status:");

		/* Note - lines 12-?? reserved for printing statistics */
		mvprintw(11, 0, "Recent Statistics:");

		refresh();
		btsleep(0.05);

		/* Move through the state machine */
		switch (phase) {
		case MU_P_START:
			mvprintw(9, 3, "Moving to above position ...           ");
			syslog(LOG_ERR, "Pose is: %s", bt_gsl_vector_sprintf(buf, poses[pose]));
			gsl_blas_dcopy(poses[pose], moveto_vec);
			gsl_blas_daxpy(1.0, angle_diff, moveto_vec);
			syslog(LOG_ERR, "Moving to: %s", bt_gsl_vector_sprintf(buf, moveto_vec));
			wam.moveTo(jp_type(moveto_vec), false);

			phase = (enum PHASE) ((int) phase + 1);
			break;
		case MU_P_TO_TOP:
			if (!wam.moveIsDone())
				break;

			if (hand != NULL  &&  pose == 0) {
				hand->initialize();
				hand->close(Hand::GRASP);
			}

			mvprintw(9, 3, "Moving to position (from above) ...    ");
			wam.moveTo(jp_type(poses[pose]), false, SLOW_VEL, SLOW_ACCEL);

			phase = (enum PHASE) ((int) phase + 1);
		case MU_P_FROM_TOP:
			if (!wam.moveIsDone())
				break;
			mvprintw(9, 3, "Starting a measurement ...             ");
			mu_n = 0;
			phase = (enum PHASE) ((int) phase + 1);
			break;
		case MU_P_MEAS_TOP:
			if (mu_n < NUM_POINTS)
				break;
			mu_stats(torques_top, positions_top);
			mvprintw(9, 3, "Moving to below position ...           ");
			gsl_blas_dcopy(poses[pose], moveto_vec);
			gsl_blas_daxpy(-1.0, angle_diff, moveto_vec);
			wam.moveTo(jp_type(moveto_vec), false);

			phase = (enum PHASE) ((int) phase + 1);
			break;
		case MU_P_TO_BOT:
			if (!wam.moveIsDone())
				break;
			mvprintw(9, 3, "Moving to position (from below) ...    ");
			wam.moveTo(jp_type(poses[pose]), false, SLOW_VEL, SLOW_ACCEL);

			phase = (enum PHASE) ((int) phase + 1);
			break;
		case MU_P_FROM_BOT:
			if (!wam.moveIsDone())
				break;
			mvprintw(9, 3, "Starting a measurement ...             ");
			mu_n = 0;
			phase = (enum PHASE) ((int) phase + 1);
			break;
		case MU_P_MEAS_BOT:
			if (mu_n < NUM_POINTS)
				break;
			mu_stats(torques_bottom, positions_bottom);
			phase = (enum PHASE) ((int) phase + 1);
			break;
		case MU_P_DONE:
			/* Get the midpoint position and torque ... */
			gsl_vector_set_zero(torques[pose]);
			gsl_blas_daxpy(0.5, torques_top, torques[pose]);
			gsl_blas_daxpy(0.5, torques_bottom, torques[pose]);
			gsl_vector_set_zero(positions[pose]);
			gsl_blas_daxpy(0.5, positions_top, positions[pose]);
			gsl_blas_daxpy(0.5, positions_bottom, positions[pose]);
			pose++;
			phase = MU_P_START;
			if (pose == num_poses)
				done = 1;
			break;
		}
	}

	/* Stop ncurses ... */
	clear();
	endwin();
	if (freopen(NULL, "w", stdout) == NULL) {  // restore stdout's line buffering
		syslog(LOG_ERR, "%s:%d freopen(stdout) failed.", __FILE__, __LINE__);
	}


	/* Free unneeded variables */
	for (j = 0; j < n; j++) {
		free(mu_jts[j]);
		free(mu_jps[j]);
	}
	free(mu_jts);
	free(mu_jps);
	gsl_vector_free(angle_diff);
	gsl_vector_free(moveto_vec);
	gsl_vector_free(torques_top);
	gsl_vector_free(positions_top);
	gsl_vector_free(torques_bottom);
	gsl_vector_free(positions_bottom);


	if (done == 1) {
		struct bt_kinematics * kin;
		struct bt_calgrav * grav;

		/* Here we have the "Iterative Algorithm"
		 * described in the Chris Dellin document entitled
		 * "Newton-Euler First-Moment Gravity Compensation" */

		/* Here we have the matrix composed of L matrices (negative) */
		gsl_matrix * nLL;

		/* We have a GT matrix and a Y vector for each link */
		gsl_matrix ** GT;
		gsl_vector ** Y;

		/* We have a solution vector P for each link */
		gsl_vector ** P;

		printf(">>> Calibration completed!\n");

		/* Start calculating ...
		 * We have vectors of torque and position
		 * in torques[] and positions[] */

		libconfig::Setting& wamSetting = pm.getConfig().lookup(pm.getWamDefaultConfigPath());
		bt_kinematics_create(&kin, wamSetting["kinematics"].getCSetting(), n);
		bt_calgrav_create(&grav, wamSetting["gravity_compensation"].getCSetting(), n);

		/* Make the nLL matrix */
		nLL = gsl_matrix_calloc(3 * num_poses, 3 + 2 * num_poses);
		for (pose = 0; pose < num_poses; pose++) {
			gsl_matrix_set(nLL, 3 * pose + 0, 3 + 2 * pose + 0, -1.0);
			gsl_matrix_set(nLL, 3 * pose + 1, 3 + 2 * pose + 1, -1.0);
		}

		/* Make each link's GT matrix */
		/* Make each link's Y torque solution matrix */
		Y = (gsl_vector **) malloc(n * sizeof(gsl_vector *));
		GT = (gsl_matrix **) malloc(n * sizeof(gsl_matrix *));
		for (j = 0; j < n; j++) {
			Y[j] = gsl_vector_alloc(3 * num_poses);
			GT[j] = gsl_matrix_calloc(3 * num_poses, 3 + 2 * num_poses);
		}

		for (pose = 0; pose < num_poses; pose++) {
			/* Put the robot in the pose (by position) */
			bt_kinematics_eval(kin, positions[pose], 0);
			bt_calgrav_eval(grav, kin, 0); /* find g vectors */
			for (j = 0; j < n; j++) {
				/* GT: Gravity skew matrix */
				gsl_matrix_set(GT[j], 3 * pose + 0, 1, -gsl_vector_get(grav->g[j], 2));
				gsl_matrix_set(GT[j], 3 * pose + 0, 2, gsl_vector_get(grav->g[j], 1));
				gsl_matrix_set(GT[j], 3 * pose + 1, 0, gsl_vector_get(grav->g[j], 2));
				gsl_matrix_set(GT[j], 3 * pose + 1, 2, -gsl_vector_get(grav->g[j], 0));
				gsl_matrix_set(GT[j], 3 * pose + 2, 0, -gsl_vector_get(grav->g[j], 1));
				gsl_matrix_set(GT[j], 3 * pose + 2, 1, gsl_vector_get(grav->g[j], 0));
				/* GT: -R*L */
				gsl_matrix_set(GT[j], 3 * pose + 0, 3 + 2 * pose + 0, -gsl_matrix_get(kin->link[j]->rot_to_prev, 0, 0));
				gsl_matrix_set(GT[j], 3 * pose + 0, 3 + 2 * pose + 1, -gsl_matrix_get(kin->link[j]->rot_to_prev, 1, 0));
				gsl_matrix_set(GT[j], 3 * pose + 1, 3 + 2 * pose + 0, -gsl_matrix_get(kin->link[j]->rot_to_prev, 0, 1));
				gsl_matrix_set(GT[j], 3 * pose + 1, 3 + 2 * pose + 1, -gsl_matrix_get(kin->link[j]->rot_to_prev, 1, 1));
				gsl_matrix_set(GT[j], 3 * pose + 2, 3 + 2 * pose + 0, -gsl_matrix_get(kin->link[j]->rot_to_prev, 0, 2));
				gsl_matrix_set(GT[j], 3 * pose + 2, 3 + 2 * pose + 1, -gsl_matrix_get(kin->link[j]->rot_to_prev, 1, 2));
				/* Y */
				gsl_vector_set(Y[j], 3 * pose + 0,
						gsl_vector_get(torques[pose], j) * gsl_matrix_get(kin->link[j]->rot_to_prev, 2, 0));
				gsl_vector_set(Y[j], 3 * pose + 1,
						gsl_vector_get(torques[pose], j) * gsl_matrix_get(kin->link[j]->rot_to_prev, 2, 1));
				gsl_vector_set(Y[j], 3 * pose + 2,
						gsl_vector_get(torques[pose], j) * gsl_matrix_get(kin->link[j]->rot_to_prev, 2, 2));
				if (j < n - 1)
					gsl_vector_set(Y[j], 3 * pose + 2,
							gsl_vector_get(Y[j], 3 * pose + 2) - gsl_vector_get(torques[pose], j + 1));
			}
		}

		/* Make a space for each link's P solution vector */
		P = (gsl_vector **) malloc(n * sizeof(gsl_vector *));
		for (j = 0; j < n; j++)
			P[j] = gsl_vector_alloc(3 + 2 * num_poses);

		/* Do the regression for each link */
		{
			int i;
			gsl_vector * b;
			gsl_matrix * m2x2;
			gsl_matrix * m2x2inverse;
			gsl_matrix * m2x3;
			gsl_permutation * permutation;

			b = gsl_vector_alloc(3 * num_poses);
			m2x2 = gsl_matrix_alloc(3 + 2 * num_poses, 3 + 2 * num_poses);
			m2x2inverse = gsl_matrix_alloc(3 + 2 * num_poses, 3 + 2 * num_poses);
			permutation = gsl_permutation_alloc(3 + 2 * num_poses);
			m2x3 = gsl_matrix_alloc(3 + 2 * num_poses, 3 * num_poses);

			for (j = n - 1; j >= 0; j--) {
				int signum; /* For the inverse */

				/* b = Y - nLL P(j+1) */
				gsl_blas_dcopy(Y[j], b);
				if (j < n - 1)
					gsl_blas_dgemv(CblasNoTrans, 1.0, nLL, P[j + 1], 1.0, b);

				/* m2x2 = GT[j]' * GT[j] */
				gsl_blas_dgemm(CblasTrans, CblasNoTrans, 1.0, GT[j], GT[j], 0.0, m2x2);

				/* Increment the diagonal of m2x2 by CALC_LAMBDA */
				for (i = 0; i < 3 + 2 * num_poses; i++)
					*(gsl_matrix_ptr(m2x2, i, i)) += CALC_LAMBDA;

				/* Calculate the inverse of m2x2 */
				gsl_linalg_LU_decomp(m2x2, permutation, &signum);
				gsl_linalg_LU_invert(m2x2, permutation, m2x2inverse);

				/* m2x3 = m2x2inverse * GT[j]' */
				gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0, m2x2inverse, GT[j], 0.0, m2x3);

				/* P[j] = m2x3 * b */
				gsl_blas_dgemv(CblasNoTrans, 1.0, m2x3, b, 0.0, P[j]);
			}

			gsl_vector_free(b);
			gsl_matrix_free(m2x2);
			gsl_matrix_free(m2x2inverse);
			gsl_permutation_free(permutation);
			gsl_matrix_free(m2x3);

			/* Copy the results */
			for (j = 0; j < n; j++) {
				gsl_vector_set(mus[j], 0, gsl_vector_get(P[j], 0));
				gsl_vector_set(mus[j], 1, gsl_vector_get(P[j], 1));
				gsl_vector_set(mus[j], 2, gsl_vector_get(P[j], 2));
			}

			/* Clean up */
			gsl_matrix_free(nLL);
			for (j = 0; j < n; j++) {
				gsl_matrix_free(GT[j]);
				gsl_vector_free(Y[j]);
				gsl_vector_free(P[j]);
			}
			free(GT);
			free(Y);
			free(P);
		}


		char* dataConfigFile = new char[strlen(DATA_CONFIG_FILE) + strlen(pm.getWamDefaultConfigPath()) - 2 + 1];
		sprintf(dataConfigFile, DATA_CONFIG_FILE, pm.getWamDefaultConfigPath());
		manageBackups(dataConfigFile);  // Backup old calibration data

		// Save to the data config file
		libconfig::Config dataConfig;
		libconfig::Setting& musSetting = dataConfig.getRoot()
				.add("gravity_compensation", libconfig::Setting::TypeGroup)
				.add("mus", libconfig::Setting::TypeList);
		for (size_t i = 0; i < DOF; ++i) {
			libconfig::Setting& rowSetting = musSetting.add(libconfig::Setting::TypeList);
			rowSetting.add(libconfig::Setting::TypeFloat) = gsl_vector_get(mus[i], 0);
			rowSetting.add(libconfig::Setting::TypeFloat) = gsl_vector_get(mus[i], 1);
			rowSetting.add(libconfig::Setting::TypeFloat) = gsl_vector_get(mus[i], 2);
		}

		dataConfig.writeFile(dataConfigFile);
		printf(">>> Data written to: %s\n", dataConfigFile);

		delete[] dataConfigFile;
	} else {
		printf(">>> ERROR: Calibration canceled.\n");
	}


	/* Re-fold and exit */
	printf(">>> Moving back to home position.\n");
	if (hand != NULL) {
		hand->open(Hand::GRASP);
		hand->close(Hand::SPREAD);
		hand->trapezoidalMove(Hand::jp_type(M_PI/2.0), Hand::GRASP);
	}
	wam.moveHome();
	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);

	/* Free the variables */
	for (pose = 0; pose < num_poses; pose++) {
		gsl_vector_free(torques[pose]);
		gsl_vector_free(positions[pose]);
	}
	free(torques);
	free(positions);
	for (j = 0; j < n; j++)
		gsl_vector_free(mus[j]);
	free(mus);

	return 0;
}
