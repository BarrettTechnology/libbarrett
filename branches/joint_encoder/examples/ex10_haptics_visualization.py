import sys
from socket import *
from time import sleep
import struct
from visual import *
import numpy


# args
if len(sys.argv) == 2:
	remoteHost = sys.argv[1]
else:
	print "Usage: %s <WAM IP address>" % sys.argv[0]
	exit()


# set up socket
PORT = 5555
MSG_FORMAT = "ddd"  # messages contain 3 doubles
MSG_SIZE = struct.calcsize(MSG_FORMAT)

sock = socket(AF_INET, SOCK_DGRAM)
#sock.setblocking(0)
sock.bind(('', PORT))
sock.connect((remoteHost,PORT))


# set up visual
scene.fullscreen = True
#scene.cursor.visible = False
#scene.autocenter = True
#scene.autoscale = True
scene.center = (0,-0.4,1)
scene.scale = (1.25,1.25,1.25)

f = frame()
f.rotate(angle = -pi/2.0, axis = (1,0,0), origin = (0,0,0))
f.rotate(angle = -pi/2.0, axis = (0,1,0), origin = (0,0,0))
f.rotate(angle = pi*0.2, axis = (1,0,0), origin = (0,0,0))

# unit vectors
#arrow(pos = (0,0,0), axis = (1,0,0), color = color.red, frame = f)
#arrow(pos = (0,0,0), axis = (0,1,0), color = color.green, frame = f)
#arrow(pos = (0,0,0), axis = (0,0,1), color = color.blue, frame = f)

# floor
xMin,xMax = -1,1
yMin,yMax = -1,1
z = -0.5
step = 0.1

for x in numpy.linspace(xMin, xMax, (xMax-xMin)/step):
	curve(pos = [(x,yMin,z), (x,yMax,z)], frame = f)
for y in numpy.linspace(yMin, yMax, (yMax-yMin)/step):
	curve(pos = [(xMin,y,z), (xMax,y,z)], frame = f)

# haptic objects
sphere(pos = (0.4,-0.3,0), radius = 0.2, color = color.green, opacity = 0.6, frame = f)
box(pos = (0.35,0.4,0), length = 0.3, height = 0.3, width = 0.3, color = color.blue, opacity = 0.6, frame = f)

# end point
ep = sphere(pos = (0,0,0), radius = 0.03, color = color.red, frame = f)


# loop!
while True:
	ep.pos = struct.unpack(MSG_FORMAT, sock.recv(MSG_SIZE))
	
sock.close()

