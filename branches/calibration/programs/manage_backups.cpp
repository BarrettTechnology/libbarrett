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


bool fileExists(const char *file)
{
	std::ifstream fs(file);
	return fs;
}


void manageBackups(const char* file, int numBackups) {
	assert(numBackups <= 9);  // Otherwise our strings won't be long enough
	char* backupFile1 = new char[strlen(file) + 2 + 1];
	char* backupFile2 = new char[strlen(file) + 2 + 1];

	sprintf(backupFile2, "%s.%d", file, numBackups);
	for (int i = numBackups - 1; i >= 0; --i) {
		if (i != 0) {
			sprintf(backupFile1, "%s.%d", file, i);
		} else {
			strcpy(backupFile1, file);
		}

		if (fileExists(backupFile1)) {
			if (fileExists(backupFile2)) {
				remove(backupFile2);
			}
			rename(backupFile1, backupFile2);
		}
		strcpy(backupFile2, backupFile1);
	}

	delete[] backupFile1;
	delete[] backupFile2;
}
