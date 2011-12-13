/*
 * zerocal.cpp
 *
 *  Created on: Apr 8, 2010
 *      Author: cd
 *      Author: dc
 */

#include <vector>
#include <string.h>

#include <syslog.h>
#include <unistd.h>

#include <curses.h>

#include <boost/thread.hpp>
#include <boost/ref.hpp>

#include <barrett/exception.h>
#include <barrett/detail/stl_utils.h>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>


using namespace barrett;
using detail::waitForEnter;


enum btkey {
	BTKEY_UNKNOWN = -2,
	BTKEY_NOKEY = -1,
	BTKEY_TAB = 9,
	BTKEY_ENTER = 10,
	BTKEY_ESCAPE = 27,
	BTKEY_BACKSPACE = 127,
	BTKEY_UP = 256,
	BTKEY_DOWN = 257,
	BTKEY_LEFT = 258,
	BTKEY_RIGHT = 259
};
enum btkey btkey_get() {
	int c1, c2, c3;

	/* Get the key from ncurses */
	c1 = getch();
	if (c1 == ERR)
		return BTKEY_NOKEY;

	/* Get all keyboard characters */
	if (32 <= c1 && c1 <= 126)
		return (enum btkey) c1;

	/* Get special keys */
	switch (c1) {
	case BTKEY_TAB:
	case BTKEY_ENTER:
	case BTKEY_BACKSPACE:
		return (enum btkey) c1;
		/* Get extended keyboard chars (eg arrow keys) */
	case 27:
		c2 = getch();
		if (c2 == ERR)
			return BTKEY_ESCAPE;
		if (c2 != 91)
			return BTKEY_UNKNOWN;
		c3 = getch();
		switch (c3) {
		case 65:
			return BTKEY_UP;
		case 66:
			return BTKEY_DOWN;
		case 67:
			return BTKEY_RIGHT;
		case 68:
			return BTKEY_LEFT;
		default:
			return BTKEY_UNKNOWN;
		}
	default:
		return BTKEY_UNKNOWN;
	}
}


int * mz_mechisset;
int * mz_counts;
int * mz_magvals;
double * mz_angles;
int mz_magvals_get;

void magenc_thd_function(bool* going , const std::vector<MotorPuck>& motorPucks) {
	int i;
	const int n = motorPucks.size();

	while (*going) {

		if (mz_magvals_get) {
			for (i = 0; i < n; i++)
				if (mz_mechisset[i]) {
					mz_magvals[i] = motorPucks[i].getPuck()->getProperty(Puck::MECH);
					mz_angles[i] = motorPucks[i].counts2rad(mz_magvals[i]);
				}
			mz_magvals_get = 0;
		}
		usleep(100000);
	}

	return;
}


template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	int n = DOF;
	int done;

	/* GUI stuff */
	enum MODE {
		MODE_TOZERO, MODE_CANCEL, MODE_PRINTVALS, MODE_JSELECT, MODE_EDIT
	} mode;
	int joint;
	int decplace;
	gsl_vector * jangle;

	char newhome[80];
	char zeromag[80];


	wam.jpController.setControlSignalLimit(jp_type()); // disable torque saturation because gravity comp isn't on


	/* Spin off the magenc thread, which also detects Puck versions into mechset */
	mz_mechisset = (int *) malloc(n * sizeof(int));
	mz_counts = (int *) malloc(n * sizeof(int));
	mz_magvals = (int *) calloc(n, sizeof(int));
	mz_angles = (double *) calloc(n, sizeof(double));
	mz_magvals_get = 0;
	bool magencGoing = true;

	/* Detect puck versions */
	const std::vector<MotorPuck>& motorPucks = wam.llww.getLowLevelWam().getMotorPucks();
	for (size_t i = 0; i < motorPucks.size(); i++) {
		long vers = motorPucks[i].getPuck()->getVers();
		long role = motorPucks[i].getPuck()->getRole();

		mz_mechisset[i] = ((vers >= 118) && (role & 256)) ? 1 : 0;
		mz_counts[i] = motorPucks[i].getCts();
	}

	boost::thread magenc_thd(magenc_thd_function, &magencGoing, boost::ref(wam.llww.getLowLevelWam().getMotorPucks()));


	/* Initialize the ncurses screen library */
	initscr();
	cbreak();
	noecho();
	timeout(0);
	clear();


	/* Start the user interface */
	mode = MODE_TOZERO;
	joint = 0;
	decplace = 4;
	jangle = gsl_vector_alloc(n);
	wam.getJointPositions().copyTo(jangle);

	done = 0;
	while (!done) {
		int j;
		int line;
		enum btkey key;


		/* Display the Zeroing calibration stuff ... */
		mz_magvals_get = 1;

		line = 2;
		if (mode == MODE_TOZERO) {
			attron( A_BOLD);
			mvprintw(line++, 0, "--> ] Move To Current Zero");
			attroff(A_BOLD);
		} else
			mvprintw(line++, 0, "    ] Move To Current Zero");
		if (mode == MODE_CANCEL) {
			attron( A_BOLD);
			mvprintw(line++, 0, "--> ] Cancel Calibration");
			attroff(A_BOLD);
		} else
			mvprintw(line++, 0, "    ] Cancel Calibration");
		if (mode == MODE_PRINTVALS) {
			attron( A_BOLD);
			mvprintw(line++, 0, "--> ] Print Calibrated Values and Exit");
			attroff(A_BOLD);
		} else
			mvprintw(line++, 0, "    ] Print Calibrated Values and Exit");
		if (mode == MODE_JSELECT) {
			attron( A_BOLD);
			mvprintw(line, 0, "--> ] Joint:");
			attroff(A_BOLD);
		} else
			mvprintw(line, 0, "    ] Joint:");
		mvprintw(line + 1, 5, "-------");
		if (mode == MODE_EDIT)
			attron( A_BOLD);
		mvprintw(line + 2, 5, "   Set:");
		if (mode == MODE_EDIT)
			attroff( A_BOLD);
		mvprintw(line + 3, 5, "Actual:");
		mvprintw(line + 5, 5, " Motor:");
		mvprintw(line + 6, 5, "MagEnc:");
		for (j = 0; j < n; j++) {
			if ((mode == MODE_JSELECT || mode == MODE_EDIT) && j == joint) {
				attron( A_BOLD);
				mvprintw(line + 0, 13 + 9 * j, "[Joint %d]", j + 1);
				attroff(A_BOLD);
			} else
				mvprintw(line + 0, 13 + 9 * j, " Joint %d ", j + 1);
			/* total with 8, 5 decimal points (+0.12345) */
			if (mode == MODE_EDIT && j == joint) {
				int boldplace;
				mvprintw(line + 1, 13 + 9 * j, " _._____ ", j + 1);
				mvprintw(line + 2, 13 + 9 * j, "% 08.5f ", gsl_vector_get(
						jangle, j));
				boldplace = decplace + 1;
				if (decplace)
					boldplace++;
				mvprintw(line + 1, 13 + 9 * j + boldplace, "x");
				mvchgat(line + 2, 13 + 9 * j + boldplace, 1, A_BOLD, 0, NULL );
			} else {
				mvprintw(line + 1, 13 + 9 * j, " ------- ", j + 1);
				mvprintw(line + 2, 13 + 9 * j, "% 08.5f ", gsl_vector_get(
						jangle, j));
			}
			mvprintw(line + 3, 13 + 9 * j, "% 08.5f ", wam.getJointPositions()[j]);
			mvprintw(line + 5, 13 + 9 * j, " Motor %d", j + 1);
			if (mz_mechisset[j]) {
				mvprintw(line + 6, 13 + 9 * j, "   %05.3f", mz_angles[j]);
				mvprintw(line + 7, 13 + 9 * j, "    %04d", mz_magvals[j]);
			} else
				mvprintw(line + 6, 13 + 9 * j, "   (None)", mz_magvals[j]);

		}
		refresh();

		/* Wait a bit ... */
		usleep(50000);
		key = btkey_get();

		/* If the user is in menu mode, work the menu ... */
		if (mode != MODE_EDIT)
			switch (key) {
			case BTKEY_UP:
				mode = (enum MODE) ((int) mode - 1);
				if ((signed int) mode < 0)
					mode = (enum MODE) 0;
				break;
			case BTKEY_DOWN:
				mode = (enum MODE) ((int) mode + 1);
				if (mode > MODE_JSELECT)
					mode = MODE_JSELECT;
				break;
			case BTKEY_ENTER:
				switch (mode) {
				case MODE_TOZERO:
					gsl_vector_set_zero(jangle);
					wam.moveTo(jp_type(jangle), false);
					break;
				case MODE_CANCEL:
					done = -1;
					break;
				case MODE_PRINTVALS:
					done = 1;
					break;
				case MODE_JSELECT:
					mode = MODE_EDIT;
					decplace = 4;
					break;
				case MODE_EDIT:
					break;
				}
				break;
			default:
				if (mode == MODE_JSELECT)
					switch (key) {
					case BTKEY_LEFT:
						joint--;
						if (joint < 0)
							joint = 0;
						break;
					case BTKEY_RIGHT:
						joint++;
						if (joint >= n)
							joint = n - 1;
						break;
					default:
						break;
					}
				break;
			}
		/* We're in joint edit mode */
		else
			switch (key) {
			case BTKEY_LEFT:
				decplace--;
				if (decplace < 0)
					decplace = 0;
				break;
			case BTKEY_RIGHT:
				decplace++;
				if (decplace > 5)
					decplace = 5;
				break;
			case BTKEY_BACKSPACE:
				mode = MODE_JSELECT;
				break;
				/* Actually do the moves */
			case BTKEY_UP:
				*(gsl_vector_ptr(jangle, joint)) += pow(10, -decplace);
				wam.moveTo(jp_type(jangle), false);
				break;
			case BTKEY_DOWN:
				*(gsl_vector_ptr(jangle, joint)) -= pow(10, -decplace);
				wam.moveTo(jp_type(jangle), false);
				break;
			default:
				break;
			}
	}

	if (done == 1) {
		gsl_vector * vec;
		/* Save the new home location */
		vec = gsl_vector_alloc(n);
		gsl_vector_memcpy(vec, wam.getHomePosition().asGslType());
		gsl_vector_sub(vec, jangle);
		sprintf(newhome, "( %05.3f", gsl_vector_get(vec, 0));
		for (int i = 1; i < n; i++)
			sprintf(newhome + strlen(newhome), ", %05.3f", gsl_vector_get(vec,
					i));
		sprintf(newhome + strlen(newhome), " );");
		gsl_vector_free(vec);

		/* Save the zeromag values */
		for (int i = 0; i < n; i++)
			if (!mz_mechisset[i])
				mz_angles[i] = -1.0;
		sprintf(zeromag, "( %05.3f", mz_angles[0]);
		for (int i = 1; i < n; i++)
			sprintf(zeromag + strlen(zeromag), ", %05.3f", mz_angles[i]);
		sprintf(zeromag + strlen(zeromag), " );");
	}

	/* Stop ncurses ... */
	clear();
	endwin();
	if (freopen(NULL, "w", stdout) == NULL) {  // restore stdout's line buffering
		syslog(LOG_ERR, "%s:%d freopen(stdout) failed.", __FILE__, __LINE__);
	}

	/* Re-fold, print, and exit */
	printf("Beginning move back to the home location...\n");
	wam.moveHome(false);

	if (done == 1) {
		/* Print the results */
		printf("\n");
		printf("Zeroing calibration ended.\n");
		printf("\n");
		for (int i = 0; i < n; i++)
			if (!mz_mechisset[i]) {
				printf("Note: Some (or all) of your pucks do not support absolute\n");
				printf("position measurement, either because they do not use magnetic\n");
				printf("encoders, or because they have not been updated to firmware r118.\n");
				printf("\n");
				break;
			}
		printf("Copy the following lines into your wam*.conf file,\n");
//      printf("near the top, in the %s{} group.\n",wamname);
		printf("Make sure it replaces the old home = ( ... ); definition.\n");
		printf("--------\n");
		printf("      # Calibrated zero values ...\n");
		printf("      home = %s\n", newhome);
		for (int i = 0; i < n; i++)
			if (mz_mechisset[i]) {
				printf("      zeroangle = %s\n", zeromag);
				break;
			}
		printf("--------\n");
//      {
//         FILE * logfile;
//         logfile = fopen("cal-zero.log","w");
//         if (logfile)
//         {
//            fprintf(logfile,"      # Calibrated zero values ...\n");
//            fprintf(logfile,"      home = %s\n",newhome);
//            for (i=0; i<n; i++) if (mz_mechisset[i])
//            { fprintf(logfile,"      zeromag = %s\n",zeromag); break; }
//            fclose(logfile);
//            printf("This text has been saved to cal-zero.log.\n");
//            printf("\n");
//         }
//         else
//         {
//            syslog(LOG_ERR,"Could not write to cal-zero.log.");
//            printf("Error: Could not write to cal-zero.log.\n");
//            printf("\n");
//         }
//      }

//		printf("Note that you must E-Stop (or power-cycle) your WAM\n");
//		printf("for the calibrated settings to take effect!\n");
		printf("\n");
	}


	magencGoing = false;
	magenc_thd.join();

	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
	pm.getSafetyModule()->setWamZeroed(false);  // Make sure the user applies their new calibration

	free(mz_mechisset);
	free(mz_counts);
	free(mz_magvals);
	free(mz_angles);
	gsl_vector_free(jangle);

	return 0;
}


int main(int argc, char** argv) {
	// Give us pretty stack-traces when things die
	installExceptionHandler();

	ProductManager pm;
	pm.waitForWam(false);


	SafetyModule* sm = pm.getSafetyModule();
	if (sm == NULL) {
		printf("ERROR: No SafetyModule found.\n");
		return 1;
	} else {
		sm->setWamZeroed(false);
	}

	// Remove existing zerocal information, if present
	libconfig::Setting& llSetting = pm.getConfig().lookup(pm.getWamDefaultConfigPath())["low_level"];
	if (llSetting.exists("zeroangle")) {
		llSetting.remove(llSetting["zeroangle"].getIndex());
		syslog(LOG_ERR, "** Ignoring previous \"zeroangle\" vector **");
	}


	printf(">>> Please *carefully* place the WAM in its new home position, then press [Enter].");
	waitForEnter();


	if (pm.foundWam4()) {
		return wam_main(argc, argv, pm, *pm.getWam4());
	} else if (pm.foundWam7()) {
		return wam_main(argc, argv, pm, *pm.getWam7());
	} else {
		printf(">>> ERROR: No WAM was found. Perhaps you have found a bug in ProductManager::waitForWam().\n");
		return 1;
	}
}


