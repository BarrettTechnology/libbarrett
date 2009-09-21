#!/bin/sh

../../common/cpplint.py --filter=-build/header_guard,-build/include_order,-legal,-readability/streams,-whitespace/blank_line,-whitespace/braces,-whitespace/tab,-whitespace/labels  --recursive ../src/
