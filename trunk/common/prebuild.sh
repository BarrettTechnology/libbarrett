#!/bin/bash

# This script is designed to be run by Eclipse from a project's build directory

scriptpath=`dirname $0`

# run cpplint on the project
if [ "$1" != "--no-cpplint" ]
then
	$scriptpath/cpplint.py \
		--filter=-build/header_guard,-build/include_order,-legal,-readability/streams,-whitespace/blank_line,-whitespace/braces,-whitespace/tab,-whitespace/labels \
		--recursive ../src/
else
	echo "Skipping cpplint."
fi

