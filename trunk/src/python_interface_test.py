#!/usr/bin/python

from libbarrett import *


#c = bus.CANSocket(0)
p = Puck(bus.CANSocket(0), 1)

print p.getProperty(Puck.STAT)
