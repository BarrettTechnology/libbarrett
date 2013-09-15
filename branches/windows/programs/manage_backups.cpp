/*
	Copyright 2011, 2012 Barrett Technology <support@barrett.com>

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
 * manage_backups.cpp
 *
 *  Created on: Dec 22, 2011
 *      Author: dc
 */

#include <fstream>
#include <cstdio>
#include <cstring>
#include <cassert>

#include "utils.h"


bool fileExists(const char* file)
{
	std::ifstream fs(file);
	return fs;
}

void backupFileName(char* str, const char* baseFileName, int backupNumer)
{
	sprintf(str, "%s.%d", baseFileName, backupNumer);
}


void manageBackups(const char* file, int numBackups) {
	assert(numBackups <= 9);  // Otherwise our strings won't be long enough
	char* backupFile1 = new char[strlen(file) + 2 + 1];
	char* backupFile2 = new char[strlen(file) + 2 + 1];

	backupFileName(backupFile2, file, numBackups);
	for (int i = numBackups - 1; i >= 0; --i) {
		if (i == 0) {
			strcpy(backupFile1, file);
		} else {
			backupFileName(backupFile1, file, i);
		}

		if (fileExists(backupFile1)) {
			if (fileExists(backupFile2)) {
				remove(backupFile2);
			}
			rename(backupFile1, backupFile2);
		}
		strcpy(backupFile2, backupFile1);
	}

	backupFileName(backupFile1, file, 1);
	if (fileExists(backupFile1)) {
		printf(">>> Old data saved: %s\n", backupFile1);
	}

	delete[] backupFile1;
	delete[] backupFile2;
}
