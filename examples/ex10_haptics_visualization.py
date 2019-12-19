import sys
from socket import *
from time import sleep
import struct
from vpython import *
import numpy


# args
if len(sys.argv) == 2:
    remoteHost = sys.argv[1]
else:
    print ("Usage: %s <WAM IP address>" % sys.argv[0])
    exit()


# set up socket
PORT_SRC = 5556
PORT_DEST = 5557
MSG_FORMAT = "ddd"  # messages contain 3 doubles
MSG_SIZE = struct.calcsize(MSG_FORMAT)

sock = socket(AF_INET, SOCK_DGRAM)
#sock.setblocking(0)
sock.bind(('', PORT_SRC))
sock.connect((remoteHost,PORT_DEST))


# set up visual
scene.visible = False
scene.fullscreen = True
scene.visible = True
#scene.cursor.visible = False
#scene.autocenter = True
#scene.autoscale = True
scene.center = vector(0,-0.4,1)
scene.scale = (1.25,1.25,1.25)
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
    curve(pos = [(x,yMin,z), (x,yMax,z)])
for y in numpy.linspace(yMin, yMax, (yMax-yMin)/step):
    curve(pos = [(xMin,y,z), (xMax,y,z)])

# haptic objects
sphere(pos = vector(0.4,-0.3,0), radius = 0.2, color = color.green, opacity = 0.6)
box(pos = vector(0.35,0.4,0), length = 0.3, height = 0.3, width = 0.3, color = color.blue, opacity = 0.6)
# end point
scene.userspin = True
scene.userpan = True
ep = sphere(pos = vector(0,0,0), radius = 0.03, color = color.red)
scene.up = vector(0,0,1)
scene.camera.rotate(angle = -pi/2.0, axis = vector(0,0,1), origin = vector(0,0,0))
scene.up = vector(1,0,0)
scene.camera.rotate(angle = -pi/2.0, axis = vector(0,0,1), origin = vector(0,0,0))
scene.camera.axis = vector(6.71314, 0.0176608, -0.882919)
scene.camera.pos = vector(-2.39578, -0.0174547, 0.879487)
# loop!
while True:
    msg = struct.unpack(MSG_FORMAT, sock.recv(MSG_SIZE))
    ep.pos = vector(msg[0], msg[1], msg[2])
    
sock.close()

