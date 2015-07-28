#!/usr/bin/python
import sys
import time
import math
import select
import struct
import numpy
import Image
from socket import *
from visual import *

# args
if len(sys.argv) == 2:
    remoteHost = sys.argv[1]
else:
    print "Usage: %s <WAM IP address>" % sys.argv[0]
    exit()

# Networking
PORT = 5555
MSG_FORMAT = "dddddddd"  # messages contain 8 doubles - 3 Proxy Position - 3 Proxy Velocity - 1 Wall Selection - 1 Button Press
MSG_SIZE = struct.calcsize(MSG_FORMAT)
sock = socket(AF_INET, SOCK_DGRAM)
sock.bind(('', PORT))
sock.connect((remoteHost, PORT))

# Visuals
xMin, xMax = 0.3, 0.7
yMin, yMax = -0.25, 0.25
zMin, zMax = 0.0, 0.5

xSize = xMax - xMin
ySize = yMax - yMin
zSize = zMax - zMin

xMiddle = xMin + (xSize / 2)
yMiddle = yMin + (ySize / 2)
zMiddle = zMin + (zSize / 2)

proxyPos = (xMiddle, yMiddle, zMiddle)
proxySize = 0.03
proxyColor = (0.0, 1.0, 0.0)
proxyVel = (0.0, 0.0, 0.0)

# Setup the scene
scene.fullscreen = True
scene.center = (0.0, yMiddle, zMiddle + 0.15)  # 0.4
scene.fov = math.pi / 2.0;
scene.scale = (1.3, 1.3, 1.3)
scene.forward = (-0.00250513, 0.517794, -0.855502)

f = frame()
f.rotate(angle= -pi / 2.0, axis=(1, 0, 0), origin=(0, 0, 0))
f.rotate(angle= -pi / 2.0, axis=(0, 1, 0), origin=(0, 0, 0))
f.rotate(angle=pi * 0.2, axis=(1, 0, 0), origin=(0, 0, 0))

wall_size = 0.002;

""" Room Definitions """
floor   = box(pos=(xMiddle, yMiddle, zMin), length=xSize, height=ySize, width=wall_size, frame=f, color=color.yellow)
ceiling = box(pos=(xMiddle, yMiddle, zMax), length=xSize, height=ySize, width=wall_size, frame=f, color=color.yellow)
left    = box(pos=(xMiddle, yMin, zMiddle), length=xSize, height=wall_size, width=zSize, frame=f, color=color.yellow)
right   = box(pos=(xMiddle, yMax, zMiddle), length=xSize, height=wall_size, width=zSize, frame=f, color=color.yellow)
back    = box(pos=(xMin, yMiddle, zMiddle), length=wall_size, height=ySize, width=zSize, frame=f, color=color.yellow)
front   = box(pos=(xMax, yMiddle, zMiddle), length=wall_size, height=ySize, width=zSize, frame=f, color=color.yellow, opacity = 0.1)
""" Button Definitions """
left_button    = cylinder(pos=(xMiddle, yMin, zMiddle), axis=(0.0, 0.01, 0.0), radius=0.15, frame=f, color=color.red)
right_button   = cylinder(pos=(xMiddle, yMax, zMiddle), axis=(0.0, -0.01, 0.0), radius=0.15, frame=f, color=color.red)
ceiling_button = cylinder(pos=(xMiddle, yMiddle, zMax), axis=(0.0, 0.0, -0.01), radius=0.15, frame=f, color=color.red)
floor_button   = cylinder(pos=(xMiddle, yMiddle, zMin), axis=(0.0, 0.0, 0.01), radius=0.15, frame=f, color=color.red)
back_button    = cylinder(pos=(xMin, yMiddle, zMiddle), axis=(0.01, 0.0, 0.0), radius=0.15, frame=f, color=color.red)
front_button   = cylinder(pos=(xMax, yMiddle, zMiddle), axis=(-0.01, 0.0, 0.0), radius=0.15, frame=f, color=color.red, opacity = 0.1)

""" Button Function Definitions """

""" Functions to change wall color"""
def nonePressed():
    left_button.color = (color.red)
    left_button.axis = (0.0, 0.01, 0.0)
    right_button.color = (color.red)
    right_button.axis = (0.0, -0.01, 0.0)
    ceiling_button.color = (color.red)
    ceiling_button.axis = (0.0, 0.0, -0.01)
    floor_button.color = (color.red)
    floor_button.axis = (0.0, 0.0, 0.01)
    back_button.color = (color.red)
    back_button.axis = (0.01, 0.0, 0.0)
    front_button.color = (color.red)
    front_button.axis = (-0.01, 0.0, 0.0)
 
     
def leftPressed():
    left_button.axis = (0.0, 0.004, 0.0)
    left.color = (color.blue)
    
def rightPressed():
    right_button.axis = (0.0, -0.004, 0.0)
    right.color = (color.blue)
    
def ceilingPressed():
    ceiling_button.axis = (0.0, 0.0, -0.004)
    ceiling.color = (color.yellow)
    
def floorPressed():
    floor_button.axis = (0.0, 0.0, 0.004)
    floor.color = (color.blue)
    
def backPressed():
    back_button.axis = (0.004, 0.0, 0.0)
    back.color = (color.blue)
    
def frontPressed():
    front_button.axis = (-0.004, 0.0, 0.0)
    front.color = (color.blue)
    front.opacity = 0.1

""" Integer Values of 0,1,2,4,8,16,32 to correspond with List """
buttonPress = 0
buttonOptions = {0 : nonePressed, 1 : leftPressed, 2 : rightPressed,
        4 : ceilingPressed, 8 : floorPressed, 16 : backPressed, 32 : frontPressed, }

""" Selected Wall will turn from Yellow to Green until Button is pressed."""
wallSelect = 0

def noneSelected():
    back.color    = (color.yellow)
    front.color   = (color.yellow)
    left.color    = (color.yellow)
    right.color   = (color.yellow)
    ceiling.color = (color.yellow)
    floor.color   = (color.yellow)

def leftSelected():
    left.color    = (color.green)    
    back.color    = (color.yellow)
    front.color   = (color.yellow)
    right.color   = (color.yellow)
    ceiling.color = (color.yellow)
    floor.color   = (color.yellow)
    
def rightSelected():
    right.color   = (color.green)
    back.color    = (color.yellow)
    front.color   = (color.yellow)
    left.color    = (color.yellow)
    ceiling.color = (color.yellow)
    floor.color   = (color.yellow)
    
    
def ceilingSelected():
    ceiling.color = (color.green)
    back.color    = (color.yellow)
    front.color   = (color.yellow)
    left.color    = (color.yellow)
    right.color   = (color.yellow)
    floor.color   = (color.yellow)
    
def floorSelected():
    floor.color   = (color.green)
    back.color    = (color.yellow)
    front.color   = (color.yellow)
    left.color    = (color.yellow)
    right.color   = (color.yellow)
    ceiling.color = (color.yellow)

    
def backSelected():
    back.color    = (color.green)
    front.color   = (color.yellow)
    left.color    = (color.yellow)
    right.color   = (color.yellow)
    ceiling.color = (color.yellow)
    floor.color   = (color.yellow)
    
def frontSelected():
    front.color   = (color.green)
    back.color    = (color.yellow)
    left.color    = (color.yellow)
    right.color   = (color.yellow)
    ceiling.color = (color.yellow)
    floor.color   = (color.yellow)
    
wallOptions = {0 : noneSelected, 1 : leftSelected, 2 : rightSelected,
        3 : ceilingSelected, 4 : floorSelected, 5 : backSelected, 6 : frontSelected, }

""" Initial Haptic Ball Definition """    
proxy = sphere(pos=proxyPos, radius=proxySize, color=proxyColor, velocity=proxyVel, frame=f)

while True:
   msg = struct.unpack(MSG_FORMAT, sock.recv(MSG_SIZE))
   proxy.pos = (msg[0],msg[1],msg[2])
   proxyVel = (msg[3:5]) # not a true sphere attribute
   wallSelect = int(msg[6])
   buttonPress = int(msg[7])
     
   """ Python version of switch to change desired Wall Color"""
   if wallSelect in wallOptions:      
      wallOptions[wallSelect]()
   else:
      wallOptions[0]()
      
   """ Python version of switch to preform button pressed functions"""
   if buttonPress in buttonOptions:
      buttonOptions[buttonPress]()
   else:
      buttonOptions[0]()
   
  
        
   
sock.close()

