#!/usr/bin/python

import sys, re, subprocess, string

FILE_NAME = "PARSE.H"
MAIN_URL = "http://web.barrett.com/svn/puck2/trunk/source/"
MON_URL = "http://web.barrett.com/svn/puck2mon/source/"
FT_URL = "http://web.barrett.com/svn/forcetorque/trunk/source/"

WC_DIR = "__property_list_tmp_dir/"
OUTPUT_H_FILE = "../include/barrett/detail/property_list.h"
OUTPUT_CPP_FILE = "property_list.cpp"

# puck types
PT_MONITOR = 0
PT_SAFETY = 1
PT_MOTOR = 2
PT_FORCETORQUE = 3

# must be valid C++ identifiers
PUCK_TYPE_NAMES = ["Monitor", "Safety", "Motor", "ForceTorque"]

p = [{}, {}, {}, {}]


# find all revisions where fileNameOrUrl changed
def revisionList(fileNameOrUrl, minVer = 1):
	vers = []
	output = subprocess.Popen(("svn -q -r HEAD:%d log" % minVer).split() + [fileNameOrUrl], stdout=subprocess.PIPE).communicate()[0]
	for line in output.split("\n"):
		m = re.match("r(\d+)", line)
		if m != None:
			vers.append(int(m.group(1)))
	return vers

def matchAndAppendProp(propList, line):
	# the comma excludes the last element in the enum, which is good
	m = re.match("(\w+)(?:\s*=\s*.+)?,", line)
	if m != None:
		prop = m.group(1)
		
		# if prop is the 2nd half of a 32-bit property
		if (len(propList) >= 1) and (propList[-1] != None) and (propList[-1] + "2" == prop):
			propList.append(None)  # insert a place holder to keep the index correct
		else:
			propList.append(prop)  # else record the property

# for main firmware revisions 1 through 60
def parseMainPropList1(fileName):
	inComment = False
	inPropList = False
	propList = []
	
	f = open(fileName, 'r')
	for line in f:
		line = line.strip()
		
		if inComment and line == "#endif":
			inComment = False
		elif line == "#if 0":
			inComment = True
		elif not inComment:
			if line.find("VERS") != -1:
				inPropList = True
			elif line == "};":
				inPropList = False
			
			if inPropList:
				matchAndAppendProp(propList, line)
	return propList

# for main firmware revisions 61 and up
def parseMainPropList61(fileName, sections):
	# parsing machine states
	ST_BEGIN = 0
	ST_WAIT = 1
	ST_PROPS = 2

	state = ST_BEGIN
	propList = []
	ret = [[] for i in range(len(sections))]

	f = open(fileName, 'r')
	for line in f:
		line = line.strip()

		if state == ST_BEGIN:
			if line in sections:
				state = ST_WAIT
				propList = ret[sections.index(line)]
		elif state == ST_WAIT:
			if line == "enum {":
				state = ST_PROPS
		elif state == ST_PROPS:
			if line == "};":
				state = ST_BEGIN
			else:
				matchAndAppendProp(propList, line)
	return ret

def mainParseFunc(fileName, v):	
	if v < 61:
		c = parseMainPropList1(fileName)
		p[PT_SAFETY][v] = c
		p[PT_MOTOR][v] = c
	else:
		(c, s, t) = parseMainPropList61(fileName, ["/* Common */", "/* Safety */", "/* Tater */"])
		p[PT_SAFETY][v] = c + s
		p[PT_MOTOR][v] = c + t

# for monitor firmware
def monParseFunc(fileName, v):
	# same as early versions of main firmware
	c = parseMainPropList1(fileName)
	p[PT_SAFETY][v] = c
	p[PT_MOTOR][v] = c

# for FT firmware
def ftParseFunc(fileName, v):
	(c, f) = parseMainPropList61(fileName, ["/* Common */", "/* FT */"])
	p[PT_FORCETORQUE][v] = c + f

def mungeRepo(url, parseFunc, minVer = 1):
	print url + FILE_NAME + ":",
	sys.stdout.flush()

	# get list of revisions
	vers = revisionList(url + FILE_NAME, minVer)
	print vers

	# parse the property list out of each of the revisions
	print ".",
	sys.stdout.flush()
	subprocess.call("rm -Rf".split() + [WC_DIR])  # in case WC_DIR already containts some other working copy
	subprocess.call("svn co".split() + [url, WC_DIR], stdout=subprocess.PIPE)
	for v in vers:
		print ".",
		sys.stdout.flush()

		subprocess.call("svn up -r".split() + [str(v), WC_DIR], stdout=subprocess.PIPE)
		parseFunc(WC_DIR + FILE_NAME, v)
	subprocess.call("rm -Rf".split() + [WC_DIR])
	print "\n"


# collect all property informationfrom the various repositories
print "### Collecting property information from 3 repositories..."
mungeRepo(MAIN_URL, mainParseFunc)
mungeRepo(MON_URL, monParseFunc)

# the FT's VERS command isn't yet implemented... not sure how to handle this robustly
mungeRepo(FT_URL, ftParseFunc, 5)

# remove version-adjacent, duplicate porperty lists
# (the cause of the revision change might have been outside the property enum(s))
for pt in p:
	v_1 = 0
	for v in sorted(pt.keys()):
		if v_1 != 0 and pt[v] == pt[v_1]:
			del pt[v]
		else:
			v_1 = v

# generate sorted list of unique property names
props = set()
for pt in p:
	props.update(*pt.values())
props.discard(None)  # remove the 32-bit property placeholder, if any
props = sorted(props)


print "\n### Writing output to %s and %s ..." % (OUTPUT_H_FILE, OUTPUT_CPP_FILE)

# output header file
f = open(OUTPUT_H_FILE, 'w')
print >> f, "const int NUM_PROPERTIES = %d;" % len(props)
print >> f, "enum Property {\n%s\n};" % str(props).translate(None, "'[]")
print >> f, "enum PuckType {\n%s\n};" % str(PUCK_TYPE_NAMES).translate(None, "'[]")
print >> f, "\n"
f.close()

# output source file
f = open(OUTPUT_CPP_FILE, 'w')

for pt in range(len(p)):
	vers = sorted(p[pt].keys())
	if len(vers) != 0:
		print >> f, "const int PL_%s[][NUM_PROPERTIES] = {" % PUCK_TYPE_NAMES[pt]
		for v in vers:
			reversePropList = dict(zip(p[pt][v], range(len(p[pt][v]))))
			print >> f, "{", str([reversePropList.get(prop, -1) for prop in props])[1:-1], "}",
			if v != vers[-1]:
				print >> f, ","
			else:
				print >> f
		print >> f, "};\n"


def binarySearchHelper(pt, vers, indentation, low, high):
	prefix = "\t" * indentation
	if low == high:
		print >> f, prefix + "return PL_%s[%d][prop];" % (PUCK_TYPE_NAMES[pt], low)
	else:
		mid = int((high-low+1) / 2) + low
		
		print >> f, prefix + "if (version >= %d) {" % vers[mid]
		binarySearchHelper(pt, vers, indentation + 1, mid, high)
		print >> f, prefix + "} else {"
		binarySearchHelper(pt, vers, indentation + 1, low, mid-1)
		print >> f, prefix + "}"
	
print >> f, "int propertyNum(enum PuckType type, int version, enum Property prop) {"
print >> f, "\tif (prop < 0  ||  prop >= NUM_PROPERTIES) {"
print >> f, "\t\treturn -1;"
print >> f, "\t}"
print >> f, "\tswitch (type) {"
for pt in range(len(p)):
	vers = sorted(p[pt].keys())
	if len(vers) != 0:
		print >> f, "\tcase %s:" % PUCK_TYPE_NAMES[pt]
		binarySearchHelper(pt, vers, 2, 0, len(vers)-1)
		print >> f, "\t\tbreak;"
print >> f, "\t}"
print >> f, "\treturn -1;"
print >> f, "}"

f.close()

print "### Done!"

