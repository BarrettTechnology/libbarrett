#!/usr/bin/python

"""
These tests assume that there is a well-behaved, Shift-idled WAM attached to the
CANbus on CAN_PORT.
"""


from libbarrett import *


CAN_PORT = 0


def assertHasattrs(obj, attrs):
	if isinstance(attrs, str):
		attrs = attrs.split()
	for a in attrs:
		assert hasattr(obj, a), "Object \"%s\" doesn't have attr \"%s\"" % (str(obj), a)

def shouldRaise(errorType, func, *args, **kwargs):
	try:
		func(*args, **kwargs)
	except errorType:
		pass
	else:
		assert False

### bus
assertHasattrs(bus, "CommunicationsBus CANSocket")

# CommunicationsBus
assert bus.CommunicationsBus.MAX_MESSAGE_LEN == 8
assert bus.CommunicationsBus.TIMEOUT > 0
assertHasattrs(bus.CommunicationsBus, "MAX_MESSAGE_LEN TIMEOUT open close isOpen send send receiveRaw")
#Shouldn't be able to instantiate bus.CommunicationsBus
shouldRaise(RuntimeError, bus.CommunicationsBus)

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

# Message is too long
shouldRaise(RuntimeError, bus.BusManager.send, b, 0x001, range(bus.CommunicationsBus.MAX_MESSAGE_LEN + 1))
# 256 doesn't fit in a char
shouldRaise(OverflowError, bus.BusManager.send, b, 0x001, [5,0,256,0,0,0,0,0])
b.send(0x001, [0,0,0,0,0,0,0,0])  # Can we send an 8-byte message?

b.send(0x001, [3])
assert b.receive(0x426) == [0x83,0,1,0]
assert b.receive(0x426, False) == []  # We don't expect any messages
b.send(0x001, [3])
assert b.receiveRaw() == (0x426, [0x83,0,1,0])
assert b.receiveRaw(False) == (0, [])  # We don't expect any messages
b.close()


### products

# Puck
assertHasattrs(Puck, "RO_MagEncOnSerial RO_Strain RO_OpticalEncOnEnc")

assertHasattrs(Puck, "PT_Monitor PT_Motor PT_Unknown")
assert Puck.getPuckTypeStr(Puck.PT_Safety) == "Safety"
assert Puck.getPuckTypeStr(Puck.PT_Motor) == "Motor"

assertHasattrs(Puck, "CMD ID JP MODE SAVE TSTOP X6")
assert Puck.getPropertyStr(Puck.STAT) == "STAT"
assert Puck.getPropertyStr(Puck.FET0) == "FET0"
# Puck.getPropertyStr() should reject invalid properties
#shouldRaise(RuntimeError, Puck.getPropertyStr, Puck.Property(-1))

assert Puck.getPropertyEnum("MODE") == Puck.MODE
assert Puck.getPropertyEnum("TStoP") == Puck.TSTOP
shouldRaise(ValueError, Puck.getPropertyEnum, "")
shouldRaise(ValueError, Puck.getPropertyEnum, "myprop")

assert Puck.getPropertyEnumNoThrow("vl1") == Puck.VL1
assert Puck.getPropertyEnumNoThrow("asdfasdf") == -1


#p = Puck(bus.CANSocket(CAN_PORT), 1)

#print p.getProperty(Puck.STAT)

print "Success! All tests passed."
