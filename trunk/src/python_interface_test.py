#!/usr/bin/python

"""
These tests assume that there is a well-behaved, Shift-idled WAM attached to the
CANbus on CAN_PORT.
"""


from time import sleep
from libbarrett import *


CAN_PORT = 0
P_ID = 1
BAD_P_ID = 9  # There is no Puck 9 on the bus


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
b.send(0x000, [0,0,0,0,0,0,0,0])  # Can we send an 8-byte message?

b.send(0x001, [3])
assert b.receive(0x426) == [0x83,0,1,0]
assert b.receive(0x426, False) == []  # We don't expect any messages
b.send(0x001, [3])
assert b.receiveRaw() == (0x426, [0x83,0,1,0])
assert b.receiveRaw(False) == (0, [])  # We don't expect any messages
b.close()


### products

# ProductManager
assertHasattrs(ProductManager, "DEFAULT_CONFIG_FILE MAX_WAM_DOF SAFETY_MODULE_ID FORCE_TORQUE_SENSOR_ID")
assertHasattrs(ProductManager, "enumerate cleanUpAfterEstop waitForWam startExecutionManager")

pm = ProductManager()
pm = ProductManager("../config/default.conf")
bm = bus.BusManager(CAN_PORT)
pm = ProductManager("../config/default.conf", bm)

pm.getPuck(1).setProperty(Puck.STAT, 0)
pm.getPuck(2).setProperty(Puck.STAT, 0)
sleep(1)
assert pm.getPuck(1).getProperty(Puck.STAT) == 0
assert pm.getPuck(2).getProperty(Puck.STAT) == 0
pm.getPuck(1).updateStatus()
pm.getPuck(2).updateStatus()
pm.wakeAllPucks()
assert pm.getPuck(1).getProperty(Puck.STAT) == 2
assert pm.getPuck(2).getProperty(Puck.STAT) == 2

assert pm.foundSafetyModule()

assert pm.foundWam()
if pm.foundWam4():
	assert not pm.foundWam7()
	assert not pm.foundWam7Wrist()
	assert not pm.foundWam7Gimbals()
	assert pm.getWamDefaultConfigPath() == "wam4"
elif pm.foundWam7Wrist():
	assert not pm.foundWam4()
	assert pm.foundWam7()
	assert not pm.foundWam7Gimbals()
	assert pm.getWamDefaultConfigPath() == "wam7w"
elif pm.foundWam7Gimbals():
	assert not pm.foundWam4()
	assert pm.foundWam7()
	assert not pm.foundWam7Wrist()
	assert pm.getWamDefaultConfigPath() == "wam7g"
else:
	assert False

assert not pm.foundForceTorqueSensor()

assert not pm.foundHand()

assert not pm.foundGimbalsHandController()

for i in range(1,5):
	assert pm.getPuck(i).getId() == i
assert pm.getPuck(9) is None

# Puck
assertHasattrs(Puck, "RO_MagEncOnSerial RO_Strain RO_OpticalEncOnEnc")

assertHasattrs(Puck, "PT_Monitor PT_Motor PT_Unknown")
assert Puck.getPuckTypeStr(Puck.PT_Safety) == "Safety"
assert Puck.getPuckTypeStr(Puck.PT_Motor) == "Motor"

assertHasattrs(Puck, "CMD ID JP MODE SAVE TSTOP X6")
assert Puck.getPropertyStr(Puck.STAT) == "STAT"
assert Puck.getPropertyStr(Puck.FET0) == "FET0"
# Puck.getPropertyStr() should reject invalid properties
shouldRaise(ValueError, Puck.getPropertyStr, Puck.Property(-1))
shouldRaise(ValueError, Puck.getPropertyStr, Puck.Property(Puck.NUM_PROPERTIES))

assert Puck.getPropertyEnum("MODE") == Puck.MODE
assert Puck.getPropertyEnum("TStoP") == Puck.TSTOP
shouldRaise(ValueError, Puck.getPropertyEnum, "")
shouldRaise(ValueError, Puck.getPropertyEnum, "myprop")

assert Puck.getPropertyEnumNoThrow("vl1") == Puck.VL1
#assert Puck.getPropertyEnumNoThrow("asdfasdf") == -1

bm = bus.BusManager(CAN_PORT)
p = Puck(bm, P_ID)
p.wake()
assert p.respondsToProperty(Puck.STAT)
assert p.getPropertyId(Puck.STAT) == 5
assert p.getProperty(Puck.STAT) == 2

assert p.respondsToProperty(Puck.CTS)
assert not p.respondsToProperty(Puck.VL2)
shouldRaise(RuntimeError, Puck.getPropertyId, p, Puck.VL2)
assert p.getPropertyIdNoThrow(Puck.VL2) == -1

assert isinstance(p.getBus(), bus.CommunicationsBus)
assert p.getId() == P_ID
assert p.getVers() > 100 and p.getVers() < 1000
assert p.getRole() & 0x001f == 0
assert p.hasOption(Puck.RO_MagEncOnSerial)
assert not p.hasOption(Puck.RO_OpticalEncOnEnc)
assert p.getType() == Puck.PT_Motor
assert p.getEffectiveType() == Puck.PT_Motor

oHsg = p.getProperty(Puck.HSG)
if oHsg > 5:
	nHsg = oHsg - 1
else:
	nHsg = oHsg + 1
oLsg = p.getProperty(Puck.LSG)
if oLsg > 5:
	nLsg = oLsg - 1
else:
	nLsg = oLsg + 1
p.setProperty(Puck.HSG, nHsg)
assert p.getProperty(Puck.HSG) == nHsg
p.resetProperty(Puck.HSG)
assert p.getProperty(Puck.HSG) == oHsg

p.setProperty(Puck.HSG, nHsg)
assert p.getProperty(Puck.HSG) == nHsg
p.saveProperty(Puck.HSG)
p.resetProperty(Puck.HSG)
assert p.getProperty(Puck.HSG) == nHsg
p.setProperty(Puck.STAT, 0)
sleep(2)
p.updateStatus()
assert p.getProperty(Puck.STAT) == 0
assert p.getType() == Puck.PT_Motor
assert p.getEffectiveType() == Puck.PT_Monitor
assert not p.respondsToProperty(Puck.CTS)
shouldRaise(RuntimeError, Puck.getPropertyId, p, Puck.CTS)
assert p.getPropertyIdNoThrow(Puck.CTS) == -1
p.wake()
assert p.getProperty(Puck.HSG) == nHsg

p.setProperty(Puck.HSG, oHsg)  # Change two properties
p.setProperty(Puck.LSG, nLsg)
p.saveAllProperties()
p.setProperty(Puck.STAT, 0)
sleep(2)
p.updateStatus()
p.wake()
sleep(1)
assert p.getProperty(Puck.HSG) == oHsg
assert p.getProperty(Puck.LSG) == nLsg

pucks = [Puck(bm, 1), Puck(bm, 3), Puck(bm, 2)]
map(lambda p: p.setProperty(Puck.STAT, 0), pucks)
sleep(2)
map(Puck.updateStatus, pucks)
assert map(Puck.getEffectiveType, pucks) == [Puck.PT_Monitor] * len(pucks)
Puck.wakeList(pucks)
assert map(Puck.getEffectiveType, pucks) == [Puck.PT_Motor] * len(pucks)

assertHasattrs(Puck, "LowLevel")
LL = Puck.LowLevel

assert LL.getProperty(bm, P_ID, 5) == 2
assert LL.tryGetProperty(bm, P_ID, 5) == (0, 2)
assert LL.tryGetProperty(bm, BAD_P_ID, 5) == (1, 0)
assert LL.tryGetProperty(bm, BAD_P_ID, 5, 0.1) == (1, 0)

assert p.getProperty(Puck.HSG) == oHsg
LL.setProperty(bm, P_ID, p.getPropertyId(Puck.HSG), nHsg)
assert p.getProperty(Puck.HSG) == nHsg
LL.setProperty(bm, P_ID, p.getPropertyId(Puck.HSG), oHsg, True)
assert p.getProperty(Puck.HSG) == oHsg

assert LL.sendGetPropertyRequest(bm, P_ID, 5) == 0
assert LL.receiveGetPropertyReply(bm, P_ID, 5, True) == (0, 2)
assert LL.receiveGetPropertyReply(bm, P_ID, 5, False) == (1, 0)

assert LL.respondsToProperty(Puck.STAT, Puck.PT_Unknown, 2)
assert not LL.respondsToProperty(Puck.X2, Puck.PT_Unknown, 150)
assert LL.respondsToProperty(Puck.X2, Puck.PT_Motor, 150)
assert LL.getPropertyId(Puck.FT, Puck.PT_ForceTorque, 5) == 54
shouldRaise(RuntimeError, LL.getPropertyId, Puck.VL2, Puck.PT_ForceTorque, 5)
assert LL.getPropertyIdNoThrow(Puck.TL2, Puck.PT_Safety, 119) == 48
assert LL.getPropertyIdNoThrow(Puck.TL2, Puck.PT_Monitor, 119) == -1

assertHasattrs(LL, "DEFAULT_IPNM MIN_ID MAX_ID HOST_ID NODE_ID_WIDTH NODE_ID_MASK")
assert LL.nodeId2BusId(3) == 0x003
assert LL.busId2NodeId(0x066) == 3
assert LL.encodeBusId(10, 5) == 0x145
assert LL.decodeBusId(0x082) == (4, 2)
assertHasattrs(LL, "GROUP_MASK FROM_MASK TO_MASK SET_MASK PROPERTY_MASK WAKE_UP_TIME TURN_OFF_TIME")

print "Success! All tests passed."
