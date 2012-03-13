#!/usr/bin/python

"""
These tests assume that there is a correctly configured WAM attached to the
CANbus on CAN_PORT.
"""


from libbarrett import *


CAN_PORT = 0


def assertHasattrs(obj, attrs):
	if isinstance(attrs, str):
		attrs = attrs.split()
	for a in attrs:
		assert hasattr(obj, a), "Object \"%s\" doesn't have attr \"%s\"" % (str(obj), a)


### bus
assertHasattrs(bus, "CommunicationsBus CANSocket")

# CommunicationsBus
assert bus.CommunicationsBus.MAX_MESSAGE_LEN == 8
assert bus.CommunicationsBus.TIMEOUT > 0
assertHasattrs(bus.CommunicationsBus, "MAX_MESSAGE_LEN TIMEOUT open close isOpen send send receiveRaw")

try:
	cb = bus.CommunicationsBus()
except RuntimeError:
	pass
else:
	assert False, "Shouldn't be able to instantiate bus.CommunicationsBus"

# CANSocket
assert issubclass(bus.CANSocket, bus.CommunicationsBus)
c = bus.CANSocket()
assert not c.isOpen()
c.open(CAN_PORT)
assert c.isOpen()
c.close()
assert not c.isOpen()

c = bus.CANSocket(CAN_PORT)
assert c.isOpen()
c.close()
assert not c.isOpen()

# BusManager
assert issubclass(bus.BusManager, bus.CommunicationsBus)
b = bus.BusManager()
assert not b.isOpen()
assert isinstance(b.getUnderlyingBus(), bus.CANSocket)

bus.CANSocket(CAN_PORT)
b = bus.BusManager(c)
# TODO(dc): Is there a way to access the pointer of the C++ object?
#assert b.getUnderlyingBus() is c
b.close()

b = bus.BusManager(CAN_PORT)
assert b.isOpen()
assert b.getUnderlyingBus().isOpen()
b.close()






p = Puck(bus.CANSocket(CAN_PORT), 1)

print p.getProperty(Puck.STAT)
