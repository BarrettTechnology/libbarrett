/*
 * verify_file_contents.cpp
 *
 *  Created on: Dec 31, 2009
 *      Author: dc
 */


#include <cstring>
#include <fstream>

#include <gtest/gtest.h>
#include "./verify_file_contents.h"


void verifyFileContents(char* fileName, char* expectedContents, long expectedSize)
{
	std::ifstream fh(fileName, std::fstream::binary);

	// get size of file
	fh.seekg(0, std::ifstream::end);
	long size = fh.tellg();
	fh.seekg(0);
	ASSERT_EQ(expectedSize, size);

	// read file
	char* buffer = new char[expectedSize];
	fh.read(buffer, expectedSize);
	EXPECT_EQ(0, std::memcmp(expectedContents, buffer, size));

	delete[] buffer;
	fh.close();
}
