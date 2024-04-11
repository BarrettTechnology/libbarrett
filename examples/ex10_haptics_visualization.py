#!/usr/bin/python3
#
# Usage:
#  python3 ex10_haptics_visualization.py <ip address>
#
# Dependencies:
#  sudo apt install python3 python3-pip freeglut3
#  python3 -m pip install PyOpenGL

import time, sys
from socket import *
import struct
import fcntl, os, errno

# OpenGL (Open Graphics Library) is a cross-language, cross-platform 
# application programming interface (API) for rendering 2D and 3D vector graphics.
import OpenGL
OpenGL.ERROR_ON_COPY = True # Report any performance-reducing numpy data conversions
from OpenGL.GL import *

# GLU is the OpenGL Utility Library. 
# This is a set of functions to create texture mipmaps from a base image, 
# map coordinates between screen and object space, and draw quadric surfaces and NURBS.
from OpenGL.GLU import *

# GLUT is the OpenGL Utility Toolkit, a window system independent toolkit for writing OpenGL programs. 
# It implements a simple windowing application programming interface (API) for OpenGL.
from OpenGL.GLUT import *

_angle = 0.0
_pos = [0.0,0.0,0.0]

def handleResize(w,h):
    glViewport(0, 0, w, h)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45.0, w / h, 1.0, 200.0)

# Draw a checkerboard pattern
def drawFloor():
    Depth = -4.0

    GridSizeX = 8
    GridSizeY = 8
    SizeX = 4
    SizeY = 4

    glBegin(GL_QUADS)
    glNormal3f(0, 1, 0)
    for x in range(-GridSizeX//2,GridSizeX//2):
        for y in range(-GridSizeY//2,GridSizeY//2):
            if ((x+y) & 1): # If odd
                glColor4f(0.2, 0.2, 0.2, 1.0) # Gray
            else:
                glColor4f(0.0,0.0,0.0,0.0) # Black

            glVertex3f(    x*SizeX,Depth,    y*SizeY)
            glVertex3f((x+1)*SizeX,Depth,    y*SizeY)
            glVertex3f((x+1)*SizeX,Depth,(y+1)*SizeY)
            glVertex3f(    x*SizeX,Depth,(y+1)*SizeY)
    glEnd()

# The function called whenever a key is pressed. 
# Note the use of Python tuples to pass in: (key, x, y)
def keyPressed(*args):
    print(args[0])
    # If escape is pressed, kill everything.
    if args[0] == b'\x1b':
        try:
            sys.exit()
        except SystemExit as e:
            glutLeaveMainLoop()
        finally:
            print("Done.")

def drawScene():
    global _angle
    global _pos

    # Clear Z-Buffer
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

    # Define material colors (R,G,B,alpha)
    qaGreen = [0.0, 1.0, 0.0, 0.6]
    qaRed   = [1.0, 0.0, 0.0, 1.0]
    qaBlue  = [0.0, 0.0, 1.0, 0.6]
    
    qaWhite = GLfloat_4(0.0, 1.0, 1.0, 1.0)
    
    # Set the camera orientation
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()

    # Set view distance
    glTranslatef(0.0, 0.0, -25.0)

    # Rotate viewpoint up around X-axis
    glRotatef(20, 1, 0, 0) # Deg, x, y, z

    glMaterialfv(GL_FRONT, GL_SPECULAR, qaWhite)
    glMaterialf(GL_FRONT, GL_SHININESS, 128.0)

    # Draw the cursor (opaque objects first)
    glColor4f(*qaRed)
    glPushMatrix()
    glTranslatef(_pos[1], _pos[2], _pos[0]) # Horizontal, Vertical, Depth
    glutSolidSphere(0.3, 32, 32) # Radius, slices, stacks
    glPopMatrix()

    # Draw a sphere (transparent objects second)
    glColor4f(*qaGreen)
    glPushMatrix()
    glTranslatef(-3, 0, 4) # Horizontal, Vertical, Depth
    glutSolidSphere(2, 32, 32) # Radius, slices, stacks
    glPopMatrix()

    # Draw a cube
    glColor4f(*qaBlue)
    glPushMatrix()
    glTranslatef(4, 0, 3.5) # Horizontal, Vertical, Depth
    glRotatef(_angle, 0.0, 1.0, 0.0)
    glutSolidCube(3) # Length of a side
    glPopMatrix()
    
    # Draw the floor
    drawFloor()
    
    # Display the buffer
    glutSwapBuffers()

def glutInitialize():
    # Set background color
    qaBlack = [0.0, 0.0, 0.0, 1.0]
    glClearColor(*qaBlack) 

    glEnable(GL_DEPTH_TEST)
    
    #glEnable(GL_NORMALIZE)

    glEnable(GL_COLOR_MATERIAL)
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
    
    glShadeModel(GL_SMOOTH)

    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
    #glMatrixMode(GL_PROJECTION)
    #glOrtho(-1.0, 1.0, -1.0, 1.0, -10.0, 10.0)

    # Lighting set up
    glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE)
    glEnable(GL_LIGHTING)
    glEnable(GL_LIGHT0)

    # Set the light position
    LIGHT_POS = 7
    #glLoadIdentity()
    qaLightPosition	= GLfloat_4(-2 * LIGHT_POS , LIGHT_POS , 4 * LIGHT_POS , 1.0)
    glLightfv(GL_LIGHT0, GL_POSITION, qaLightPosition)

    # Set lighting intensity and color
    qaAmbientLight	= GLfloat_4(0.2, 0.2, 0.2, 1.0)
    qaDiffuseLight	= GLfloat_4(0.8, 0.8, 0.8, 1.0)
    qaSpecularLight	= GLfloat_4(1.0, 1.0, 1.0, 1.0)
    glLightfv(GL_LIGHT0, GL_AMBIENT, qaAmbientLight)
    glLightfv(GL_LIGHT0, GL_DIFFUSE, qaDiffuseLight)
    glLightfv(GL_LIGHT0, GL_SPECULAR, qaSpecularLight)

def animate(value):
    global _angle
    _angle += 1.0
    if _angle >= 360:
        _angle -= 360
    glutPostRedisplay()
    glutTimerFunc(25, animate, 0)

def receiveData():
    global _sock
    global _pos

    m = 10 # Scaling factor
    b = 0  # Shift amount

    MSG_FORMAT = "ddd"  # messages contain 3 doubles
    MSG_SIZE = struct.calcsize(MSG_FORMAT)

    try:
        msg = _sock.recv(MSG_SIZE)
    except OSError as e:
        err = e.args[0]
        if err == errno.EAGAIN or err == errno.EWOULDBLOCK:
            #print('No data available')
            return
        else:
            # a "real" error occurred
            print(e)
            sys.exit(1)
    else:
        # got a message, do something :)
        raw = struct.unpack(MSG_FORMAT, msg)
        _pos = [m*x+b for x in raw] # Scale and shift the data

def main():
    # Initialize the glut system
    glutInit(sys.argv)

    # Select the drawing features
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH | GLUT_ALPHA)

    # Open a maximized window
    glutInitWindowSize(glutGet(GLUT_SCREEN_WIDTH), glutGet(GLUT_SCREEN_HEIGHT))
    glutCreateWindow("Haptics Demo")

    # Initialize lighting/shading/blending
    glutInitialize()

    # Set the drawing callback
    glutDisplayFunc(drawScene)

    # Set the keypress callback
    glutKeyboardFunc(keyPressed)

    # Set the window-resize callback
    glutReshapeFunc(handleResize)

    # Add a timer function for animation
    glutTimerFunc(25, animate, 0) # milliseconds, function, passed value

    # Add a callback to receive socket data when OpenGL is idle
    glutIdleFunc(receiveData)

    # Execute main graphics loop
    glutMainLoop()

# Set up the network socket
def setupSocket():
    global _sock
    
    PORT_SRC = 5556
    PORT_DEST = 5557

    _sock = socket(AF_INET, SOCK_DGRAM)
    _sock.bind(('', PORT_SRC))
    _sock.connect((remoteHost,PORT_DEST))
    fcntl.fcntl(_sock, fcntl.F_SETFL, os.O_NONBLOCK)

if __name__ == "__main__":
    if len(sys.argv) == 2:
	    remoteHost = sys.argv[1]
    else:
        print("Usage: %s <WAM IP address>" % sys.argv[0])
        exit()
    
    setupSocket()
    main()
