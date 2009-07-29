/* ======================================================================== *
 *  Module ............. wamdisplay
 *  File ............... graphicThread.cpp
 *  Author ............. Victor J Wang
 *  Creation Date ...... 13 July 2009
 *                                                                          *
 *  **********************************************************************  *
 *                                                                          *
 *  NOTES: originally from wamdisplay.cpp
 *
 *  REVISION HISTORY:
 * 
 *                                                                          *
 * ======================================================================== */
/** Communication thread for WAM GUI. Receives WAM position streams via socket 
 * and updates GUI in real-time. 
 * */


#define GL_GLEXT_PROTOTYPES

extern "C" {
#include <math.h>
#include <stdlib.h>
#include <stdio.h>   
#include <string.h>
#include <GL/glut.h>
#include <GL/gl.h>
#include <unistd.h>
#include <stdarg.h>
#include <pthread.h>
#include "wamComponents.h"
#include "dhTransform.h"
#include "../sockets/sockets.h"
#include "client_handler.h"
#include "../include/definitions.h"
#include "../include/wam_Spec.h"

}



#include <iostream>
#include "wamdisplayGlobals.h"
#include "GraphicThread.h"



/*! define WAM_SLEEP as 1 to reduce CPU Load<BR>
    WARNING: This can make display very slow
*/
#define WAM_SLEEP 1

#define WAM_USLEEPTIME 10000

#define MAIN_WINDOW_WIDTH 800
#define MAIN_WINDOW_HEIGHT 600

#define GAP 15

#define CONTROL_WINDOW_WIDTH 150
#define CONTROL_WINDOW_HEIGHT 150

/*! define number of frames/sec
*/
#define NUM_FPS 30
#define SCALE 0.007


/********************************************************************************************\
|                                                                                            |
|                                    WAM arm Parameters                                      |
|                                                                                            |
\********************************************************************************************/




//WAM BASE DIMENSIONS mm
#define WAM_BASE_WITH 360
#define WAM_BASE_HIGH 160
#define WAM_BASE_DEPTH 280

//WAM 1rst DOF Shoulder dimensions
//Restrictions  (WAM_SHOULDER_DEPTH > WAM_BASE_DEPTH)
#define WAM_SHOULDER_DEPTH 348
#define WAM_SHOULDER_WHIDTH 180
#define WAM_SHOULDER_HEIGHT 186
#define WAM_SHOULDER_CAVITY 119

//WAM 2nd DOF joint
#define WAM_JOINT_DIAM1 130
#define WAM_JOINT_DIAM2 118
#define WAM_JOINT_HEIGHT 110

//WAM link param
#define WAM_LINK_DIAM 89
#define WAM_LINK_ELBOW 45

//WAM 3rd DOF link
#define WAM_3LINK_HEIGHT 550

//WAM 4th DOF link
#define WAM_4LINK_HEIGHT 275

//WAM 5th DOF joint
#define WAM_5JOINT_HEIGHT 25
// IMPORTANT ->>>>WAM_4LINK_HEIGHT+WAM_5JOINT_HEIGHT=300

//WAM 6th DOF joint

#define WAM_6JOINT_HEIGHT 61





static GLint win, world, /*command,*/ text_window;
static GLfloat yellow[3] = {1,1,0};
static GLfloat green[3] = {0,1,0};
GLboolean ShowHelp = GL_TRUE;
GLboolean degRad = GL_TRUE;   //if true degrees else radians
struct camView{
  GLfloat rotx;
  GLfloat roty;
  GLfloat rotz;
  GLfloat xPos;
  GLfloat yPos;
  GLfloat zPos;
};
camView view = {0.0, 0.0, 0.0, 0.0, -3.5, 0.0};

struct button{
  GLfloat x;
  GLfloat y;
  GLfloat width;
  GLfloat height;
  GLfloat *color;
};
button buttons[4]={
  {30.0,75.0,35.0,35.0,yellow},
  {120.0,75.0,35.0,35.0,yellow},
  {75.0,30.0,35.0,35.0,yellow},
  {75.0,120.0,35.0,35.0,yellow},
};

//Lists to Call wamComponets functions
static GLint wamBase, wamShoulder_1DOF, wamJoint_2DOF, wamLink_3DOF, wamLink_4DOF, wamJoint_5DOF, wamJoint_6DOF, wamEndPlate, wamReferencial;


//WAM DH parameter ---- struct defined in "wamComponents.h"
DHparam wam7DOFparam[7]={
  {a1,alpha1,d1,theta1},
  {a2,alpha2,d2,theta2},
  {a3,alpha3,d3,theta3},
  {a4,alpha4,d4,theta4},
  {a5,alpha5,d5,theta5},
  {a6,alpha6,d6,theta6},
  {a7,alpha7,d7,theta7},
};

      /** Functions */
//threads
//void read_shared_memory();

//OPEN GL ROUTINES
void init(int argc, char *argv[]);
void visible(int vis);
void world_reshape(int width, int height);
void special(int k, int x, int y);
void key(unsigned char k, int x, int y);
void idle(void);
void world_draw(void);
void main_reshape(int width,  int height);
void main_display(void);
// void command_reshape(int width, int height);
// void command_display(void);
// void command_motion(int x, int y);
// void command_mouse(int button, int state, int x, int y);
void text_window_reshape(int width, int height);
void text_window_display(void);
void drawstr(GLuint x, GLuint y, GLvoid *font_style, char* format, ...);
void printString(GLvoid *font_style,const char *s);
void helpMenu();
void redisplay_all(void);
void cleanup(void);


GraphicThread * gra_thd;

#if 0
struct arguments {
   int argc0;
   char **argv0;
} argument;
#endif



GraphicThread::GraphicThread(double * shared_angle, 
                     int shared_finish, 
                     pthread_mutex_t * shared_mutex,
                     void * struct_ptr): shared_angle(shared_angle), 
                                    shared_finish(shared_finish),
                                    shared_mutex(shared_mutex)
{
   gra_thd = this;
   argument = (arguments *) struct_ptr;
}

GraphicThread::~GraphicThread()
{
}


      /** MAIN LOOP */
void * GraphicThread::run(void){
  glutInit(&argument->argc0, argument->argv0);
  glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);

  glutInitWindowPosition(0, 0);

  //MAIN WINDOW
  glutInitWindowSize(MAIN_WINDOW_WIDTH, MAIN_WINDOW_HEIGHT);
  //create
  win = glutCreateWindow("WAM Display");
  //main_reshape called each time window is resized
  glutReshapeFunc(main_reshape);
  //main_display called to make the display
  glutDisplayFunc(main_display);
  //visible called while window is visible
  glutVisibilityFunc(visible);
  //key called each time a key is pressed
  glutKeyboardFunc(key);
  //special characters (PGUP, ARROWS,...)
  glutSpecialFunc(special);


  //WINDOW TO DISPLAY WAM
  world = glutCreateSubWindow(win, CONTROL_WINDOW_WIDTH+2*GAP, GAP, MAIN_WINDOW_WIDTH-CONTROL_WINDOW_WIDTH-3*GAP, MAIN_WINDOW_HEIGHT-2*GAP);
  init(argument->argc0, argument->argv0);
  glutDisplayFunc(world_draw);
  glutKeyboardFunc(key);
  glutSpecialFunc(special);
  glutReshapeFunc(world_reshape);


  // WINDOW WHERE BUTTONS ARE DISPLAYED
//   command = glutCreateSubWindow(win, GAP, GAP, CONTROL_WINDOW_WIDTH , CONTROL_WINDOW_HEIGHT);
//   glutReshapeFunc(command_reshape);
//   glutDisplayFunc(command_display);
//   glutPassiveMotionFunc(command_motion);
//   glutKeyboardFunc(key);
//   glutSpecialFunc(special);
//   glutMouseFunc(command_mouse);


  //WINDOW WHERE TEXT IS DISPLAYED

  text_window = glutCreateSubWindow(win, GAP, CONTROL_WINDOW_HEIGHT+ 2*GAP, CONTROL_WINDOW_WIDTH, MAIN_WINDOW_HEIGHT-CONTROL_WINDOW_HEIGHT-3*GAP);
  text_window = glutCreateSubWindow(win, GAP, GAP, CONTROL_WINDOW_WIDTH, MAIN_WINDOW_HEIGHT-2*GAP);
  glutReshapeFunc(text_window_reshape);
  glutKeyboardFunc(key);
  glutSpecialFunc(special);
  glutDisplayFunc(text_window_display);

  //redisplay all windows
  redisplay_all();

  //start display loop
  glutMainLoop();
  return 0;
}




//dipsplay main window
void
main_display(void)
{
    glClearColor(0.8, 0.8, 0.8, 0.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glutSwapBuffers();
}

// display menu
void helpMenu(){
    glColor3f(0.0,0.0,1.0);
  glWindowPos2iARB(10, 500);
  printString(GLUT_BITMAP_HELVETICA_12,"h");
  glWindowPos2iARB(80, 500);
  printString(GLUT_BITMAP_HELVETICA_12,"-  toggle help");
  glWindowPos2iARB(10, 480);
  printString(GLUT_BITMAP_HELVETICA_12,"a,d,w,s");
  glWindowPos2iARB(80, 480);
  printString(GLUT_BITMAP_HELVETICA_12,"-  rotate robot");
  glWindowPos2iARB(10, 460);
  printString(GLUT_BITMAP_HELVETICA_12,"UP KEY");
  glWindowPos2iARB(80, 460);
  printString(GLUT_BITMAP_HELVETICA_12,"-  move robot up");
  glWindowPos2iARB(10, 440);
  printString(GLUT_BITMAP_HELVETICA_12,"DOWN KEY");
  glWindowPos2iARB(80, 440);
  printString(GLUT_BITMAP_HELVETICA_12,"-  move robot down");
  glWindowPos2iARB(10, 420);
  printString(GLUT_BITMAP_HELVETICA_12,"LEFT KEY");
  glWindowPos2iARB(80, 420);
  printString(GLUT_BITMAP_HELVETICA_12,"-  move robot left");
  glWindowPos2iARB(10, 400);
  printString(GLUT_BITMAP_HELVETICA_12,"RIGHT KEY");
  glWindowPos2iARB(80, 400);
  printString(GLUT_BITMAP_HELVETICA_12,"-  move robot right");
  glWindowPos2iARB(10, 380);
  printString(GLUT_BITMAP_HELVETICA_12,"PG UP");
  glWindowPos2iARB(80, 380);
  printString(GLUT_BITMAP_HELVETICA_12,"-  zoom in");
  glWindowPos2iARB(10, 360);
  printString(GLUT_BITMAP_HELVETICA_12,"PG DN");
  glWindowPos2iARB(80, 360);
  printString(GLUT_BITMAP_HELVETICA_12,"-  zoom out");
  glWindowPos2iARB(10, 340);
  printString(GLUT_BITMAP_HELVETICA_12,"g");
  glWindowPos2iARB(80, 340);
  printString(GLUT_BITMAP_HELVETICA_12,"-  degrees/radians");
}




int
ComputeFPS(void)
{
   static double t0 = -1.0;
   static int frames = 0;
   double t = glutGet(GLUT_ELAPSED_TIME) / 1000.0;
   static int fps = 0;

   frames++;

   if (t0 < 0.0) {
      t0 = t;
      fps = 0;
   }
   else if (t - t0 >= 1.0) {
      fps = (int) (frames / (t - t0) + 0.5);
      t0 = t;
      frames = 0;
   }

   return fps;
}

// display WAM
void world_draw(void)
{
  GLint fps;
  char s[50];
  gra_thd->read_shared_memory();
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glClearColor(230.0/256.0, 240.0/256.0, 240.0/256.0, 0.0);

  glPushMatrix();
  glTranslatef(view.xPos, view.yPos, view.zPos);
  glRotatef(-90, 1.0, 0.0, 0.0);
  glRotatef(view.rotx, 1.0, 0.0, 0.0);
  //glRotatef(view.roty, 0.0, 1.0, 0.0);
  glRotatef(view.rotz, 0.0, 0.0, 1.0);



  glScalef(SCALE,SCALE,SCALE);
 // box();

  glCallList(wamBase);
  glPushMatrix();

  //primeira rotação
  glRotatef(RAD_TO_DEG(wam7DOFparam[0].theta), 0.0, 0.0, 1.0);
  glCallList(wamShoulder_1DOF);


  glTranslatef(0.0, 0.0, WAM_SHOULDER_HEIGHT);
  //segunda rotacao
  glRotatef(RAD_TO_DEG(wam7DOFparam[1].theta), 0.0, -1.0, 0.0);
  glCallList(wamJoint_2DOF);


  //terceira rotacao
  glRotatef(RAD_TO_DEG(wam7DOFparam[2].theta), 0.0, 0.0, 1.0);
  glCallList(wamLink_3DOF);


  glTranslatef(-WAM_LINK_ELBOW, 0.0, WAM_3LINK_HEIGHT);
  //quarta rotação
  glRotatef(RAD_TO_DEG(wam7DOFparam[3].theta), 0.0, -1.0, 0.0);
  glCallList(wamLink_4DOF);


  glTranslatef(WAM_LINK_ELBOW, 0.0, WAM_4LINK_HEIGHT);
  //quinta rotação
  glRotatef(RAD_TO_DEG(wam7DOFparam[4].theta)+180, 0.0, 0.0, 1.0);
  glCallList(wamJoint_5DOF);


  glTranslatef(0.0,0.0,WAM_5JOINT_HEIGHT);
  //sexta rotação
  glRotatef(RAD_TO_DEG(wam7DOFparam[5].theta), 0.0, -1.0, 0.0);
  glCallList(wamJoint_6DOF);

  glTranslatef(0.0,0.0,WAM_6JOINT_HEIGHT);
  //setima rotação
  glRotatef(RAD_TO_DEG(wam7DOFparam[6].theta), 0.0, 0.0, 1.0);
  glCallList(wamEndPlate);


  glPushMatrix();
  glTranslatef(0.0,0.0,100.0);
  glRotatef(180, 0.0, 0.0, 1.0);
  glCallList(wamReferencial);
  glPopMatrix();

  glPopMatrix();
  glPopMatrix();

  if(ShowHelp)
    helpMenu();
  fps=ComputeFPS();
  glColor3f(1.0,1.0,1.0);
  glWindowPos2iARB(10, 10);
  sprintf(s, "%d FPS",fps);
  printString(GLUT_BITMAP_HELVETICA_12,s);


  glutSwapBuffers();

}

// display buttons
// void command_display(void)
// {
//     glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//     glClearColor(230.0/256.0, 240.0/256.0, 240.0/256.0, 0.0);
//
//     glColor3fv(buttons[0].color);
//     glBegin(GL_TRIANGLES);
//     glVertex2f(buttons[0].x-buttons[0].width/2, buttons[0].y);
//     glVertex2f(buttons[0].x+buttons[0].width/2, buttons[0].y-buttons[0].height/2);
//     glVertex2f(buttons[0].x+buttons[0].width/2, buttons[0].y+buttons[0].height/2);
//     glEnd();
//
//     glColor3f(0.8,0.8,0.8);
//     drawstr((GLuint)buttons[0].x,(GLuint) buttons[0].y+4,GLUT_BITMAP_HELVETICA_18,"a");
//
//
//
//     glColor3fv(buttons[1].color);
//     glBegin(GL_TRIANGLES);
//     glVertex2f(buttons[1].x-buttons[1].width/2, buttons[1].y-buttons[1].height/2);
//     glVertex2f(buttons[1].x+buttons[1].width/2, buttons[1].y);
//     glVertex2f(buttons[1].x-buttons[1].width/2, buttons[1].y+buttons[1].height/2);
//     glEnd();
//
//     glColor3f(0.8,0.8,0.8);
//     drawstr((GLuint)buttons[1].x-12, (GLuint)buttons[1].y+4,GLUT_BITMAP_HELVETICA_18,"d");
//
//
//
//     glColor3fv(buttons[2].color);
//     glBegin(GL_TRIANGLES);
//     glVertex2f(buttons[2].x-buttons[2].width/2, buttons[2].y+buttons[2].height/2);
//     glVertex2f(buttons[2].x+buttons[2].width/2, buttons[2].y+buttons[2].height/2);
//     glVertex2f(buttons[2].x, buttons[2].y-buttons[2].height/2);
//     glEnd();
//
//     glColor3f(0.8,0.8,0.8);
//     drawstr((GLuint)buttons[2].x-7, (GLuint)buttons[2].y+10,GLUT_BITMAP_HELVETICA_18,"w");
//
//
//
//     glColor3fv(buttons[3].color);
//     glBegin(GL_TRIANGLES);
//     glVertex2f(buttons[3].x-buttons[3].width/2, buttons[3].y-buttons[3].height/2);
//     glVertex2f(buttons[3].x+buttons[3].width/2, buttons[3].y-buttons[3].height/2);
//     glVertex2f(buttons[3].x, buttons[3].y+buttons[3].height/2);
//     glEnd();
//     glColor3f(0.8,0.8,0.8);
//     drawstr((GLuint)buttons[3].x-5, (GLuint)buttons[3].y,GLUT_BITMAP_HELVETICA_18,"s");
//
//
//     glutSwapBuffers();
// }

//display text window
void text_window_display(void)
{
    double x,y,z;
    get_ENDtranslate(7, wam7DOFparam, &x, &y, &z);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(230.0/256.0, 240.0/256.0, 240.0/256.0, 0.0);
    get_ENDtranslate(7, wam7DOFparam, &x, &y, &z);


    //GLUT_BITMAP_TIMES_ROMAN_10
    //GLUT_BITMAP_HELVETICA_12
    //GLUT_BITMAP_TIMES_ROMAN_24

    glColor3f(0,0,0);
    if(degRad)
      drawstr(5, 20,GLUT_BITMAP_HELVETICA_18,"Joints (DEG):");
    else
      drawstr(5, 20,GLUT_BITMAP_HELVETICA_18,"Joints (RAD):");
    drawstr(10, 50,GLUT_BITMAP_HELVETICA_12,"theta1 =");
    glColor3f(0.0,0.0,1.0);
    drawstr(60, 50,GLUT_BITMAP_HELVETICA_12,"%6.3f",(degRad)? RAD_TO_DEG(wam7DOFparam[0].theta) : wam7DOFparam[0].theta);


    glColor3f(0,0,0);
    drawstr(10, 65,GLUT_BITMAP_HELVETICA_12,"theta2 =");
    glColor3f(0.0,0.0,1.0);
    drawstr(60, 65,GLUT_BITMAP_HELVETICA_12,"%6.3f",(degRad)? RAD_TO_DEG(wam7DOFparam[1].theta) : wam7DOFparam[1].theta);


    glColor3f(0,0,0);
    drawstr(10, 80,GLUT_BITMAP_HELVETICA_12,"theta3 =");
    glColor3f(0.0,0.0,1.0);
    drawstr(60, 80,GLUT_BITMAP_HELVETICA_12,"%6.3f",(degRad)? RAD_TO_DEG(wam7DOFparam[2].theta) : wam7DOFparam[2].theta);


    glColor3f(0,0,0);
    drawstr(10, 95,GLUT_BITMAP_HELVETICA_12,"theta4 =");
    glColor3f(0.0,0.0,1.0);
    drawstr(60, 95,GLUT_BITMAP_HELVETICA_12,"%6.3f",(degRad)? RAD_TO_DEG(wam7DOFparam[3].theta) : wam7DOFparam[3].theta);


    glColor3f(0,0,0);
    drawstr(10, 110,GLUT_BITMAP_HELVETICA_12,"theta5 =");
    glColor3f(0.0,0.0,1.0);
    drawstr(60, 110,GLUT_BITMAP_HELVETICA_12,"%6.3f",(degRad)? RAD_TO_DEG(wam7DOFparam[4].theta+M_PI/2) : wam7DOFparam[4].theta+M_PI/2);


    glColor3f(0,0,0);
    drawstr(10, 125,GLUT_BITMAP_HELVETICA_12,"theta6 =");
    glColor3f(0.0,0.0,1.0);
    drawstr(60, 125,GLUT_BITMAP_HELVETICA_12,"%6.3f",(degRad)? RAD_TO_DEG(wam7DOFparam[5].theta) : wam7DOFparam[5].theta);


    glColor3f(0,0,0);
    drawstr(10, 140,GLUT_BITMAP_HELVETICA_12,"theta7 =");
    glColor3f(0.0,0.0,1.0);
    drawstr(60, 140,GLUT_BITMAP_HELVETICA_12,"%6.3f",(degRad)? RAD_TO_DEG(wam7DOFparam[6].theta) : wam7DOFparam[6].theta);


    glColor3f(0,0,0);
    drawstr(5, 200,GLUT_BITMAP_HELVETICA_18,"End Efector");
    drawstr(15, 220,GLUT_BITMAP_HELVETICA_18,"Position:");

    glColor3f(0,0,0);
    drawstr(10, 250,GLUT_BITMAP_HELVETICA_12,"X =");
    glColor3f(0.0,0.0,1.0);
    drawstr(50, 250,GLUT_BITMAP_HELVETICA_12,"%6.3f",(float)x);

    glColor3f(0,0,0);
    drawstr(10, 265,GLUT_BITMAP_HELVETICA_12,"Y =");
    glColor3f(0.0,0.0,1.0);
    drawstr(50, 265,GLUT_BITMAP_HELVETICA_12,"%6.3f",(float)y);

    glColor3f(0,0,0);
    drawstr(10, 280,GLUT_BITMAP_HELVETICA_12,"Z =");
    glColor3f(0.0,0.0,1.0);
    drawstr(50, 280,GLUT_BITMAP_HELVETICA_12,"%6.3f",(float)z);


    glutSwapBuffers();


}


//reshape windows
void main_reshape(int width,  int height)
{
    GLfloat sub_width, sub_height;
    if((width>CONTROL_WINDOW_WIDTH+3*GAP)&&(height>CONTROL_WINDOW_HEIGHT+3*GAP)){
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(0, width, height, 0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    sub_width = width-CONTROL_WINDOW_WIDTH-3*GAP;
    sub_height = height-GAP*2;

    glutSetWindow(world);
    glutPositionWindow(CONTROL_WINDOW_WIDTH+2*GAP, GAP);
    glutReshapeWindow((int)sub_width, (int)sub_height);

//     glutSetWindow(command);
//     glutPositionWindow(GAP, GAP);
//     glutReshapeWindow(CONTROL_WINDOW_WIDTH, CONTROL_WINDOW_HEIGHT);

    glutSetWindow(text_window);
    glutPositionWindow(GAP, CONTROL_WINDOW_HEIGHT+ 2*GAP);
    glutReshapeWindow(CONTROL_WINDOW_WIDTH, height-CONTROL_WINDOW_HEIGHT-3*GAP);

    glutPositionWindow(GAP, GAP);
    glutReshapeWindow(CONTROL_WINDOW_WIDTH, (int)sub_height);
    }
}

void world_reshape(int width, int height)
{

  GLfloat h = (GLfloat) height / (GLfloat) width;


  glViewport(0, 0, (GLint) width, (GLint) height);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glFrustum(-1.0, 1.0, -h, h, 5.0, 200.0);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glTranslatef(0.0, 0.0, -40.0);
}

// void command_reshape(int width, int height)
// {
//     glViewport(0, 0, width, height);
//     glMatrixMode(GL_PROJECTION);
//     glLoadIdentity();
//     gluOrtho2D(0, width, height, 0);
//     glMatrixMode(GL_MODELVIEW);
//     glLoadIdentity();
//     glClearColor(0.0, 0.0, 0.0, 0.0);
// }

void text_window_reshape(int width, int height)
{
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(0, width, height, 0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glClearColor(0.0, 0.0, 0.0, 0.0);
}

void redisplay_all(void)
{
    glutSetWindow(world);
    glutPostRedisplay();
//     glutSetWindow(command);
    glutPostRedisplay();
    glutSetWindow(text_window);
    glutPostRedisplay();
}

//initial deffinitions
void init(int argc, char *argv[])
{


  static GLfloat pos[4] = {5,5, 10.0, 0.0};
  const GLfloat specular[4] = { 0.8, 0.8, 0.8, 1.0 };

  glLightfv(GL_LIGHT0, GL_POSITION, pos);
  glEnable(GL_CULL_FACE);
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
  glEnable(GL_DEPTH_TEST);
  glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 40);
  glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular);

  /*Make the WAM Base*/
  wamBase = glGenLists(1);
  glNewList(wamBase, GL_COMPILE_AND_EXECUTE);
  base(WAM_BASE_WITH, WAM_BASE_HIGH, WAM_BASE_DEPTH);
  glEndList();

  /*Make the WAM 1rst DOF shoulder*/

  wamShoulder_1DOF = glGenLists(1);
  glNewList(wamShoulder_1DOF, GL_COMPILE_AND_EXECUTE);
  shoulder_1DOF(WAM_BASE_DEPTH, WAM_SHOULDER_DEPTH, WAM_SHOULDER_WHIDTH, WAM_SHOULDER_HEIGHT, WAM_SHOULDER_CAVITY);
  glEndList();

   /*Make the WAM 2nd DOF joint*/
  wamJoint_2DOF = glGenLists(1);
  glNewList(wamJoint_2DOF, GL_COMPILE_AND_EXECUTE);
  joint_2DOF(WAM_SHOULDER_CAVITY, WAM_JOINT_DIAM1, WAM_JOINT_DIAM2, WAM_JOINT_HEIGHT);
  glEndList();

  wamLink_3DOF = glGenLists(1);
  glNewList(wamLink_3DOF, GL_COMPILE_AND_EXECUTE);
  link_3DOF(WAM_LINK_DIAM, WAM_3LINK_HEIGHT, WAM_LINK_ELBOW);
  glEndList();

  wamLink_4DOF = glGenLists(1);
  glNewList(wamLink_4DOF, GL_COMPILE_AND_EXECUTE);
  link_4DOF_WRIST(WAM_LINK_DIAM, WAM_4LINK_HEIGHT, WAM_LINK_ELBOW);
  glEndList();

  wamJoint_5DOF = glGenLists(1);
  glNewList(wamJoint_5DOF, GL_COMPILE_AND_EXECUTE);
  joint_5DOF_WRIST(WAM_LINK_DIAM, WAM_5JOINT_HEIGHT);
  glEndList();

  wamJoint_6DOF = glGenLists(1);
  glNewList(wamJoint_6DOF, GL_COMPILE_AND_EXECUTE);
  joint_6DOF_WRIST(WAM_LINK_DIAM, WAM_6JOINT_HEIGHT);
  glEndList();

  wamEndPlate = glGenLists(1);
  glNewList(wamEndPlate, GL_COMPILE_AND_EXECUTE);
  endPlate(WAM_LINK_DIAM);
  glEndList();

  wamReferencial = glGenLists(1);
  glNewList(wamReferencial, GL_COMPILE_AND_EXECUTE);
  referencial();
  glEndList();

  glEnable(GL_NORMALIZE);

}

void visible(int vis)
{
  static int count=0;
  if (vis == GLUT_VISIBLE){
    glutIdleFunc(idle);
  }
  else
    glutIdleFunc(NULL);
  count++;
}


//this funtion is responsable to call the display functions whith the deffined FPS
void idle(void)
{
  static double t0us=-1.0,t1us=-1.0;
  double dt0us, dt1us, tus = glutGet(GLUT_ELAPSED_TIME);
  if(t0us<0.0)
    t0us=tus;
  dt0us=tus-t0us;

  if(t1us<0.0)
    t1us=tus;
  dt1us=tus-t1us;

  if(dt0us>(1000/NUM_FPS)){
    glutSetWindow(world);
    glutPostRedisplay();
    t0us=tus;
  }

#if WAM_SLEEP == 1
  else
      usleep(WAM_USLEEPTIME);
#endif

  if(dt1us>100){
    glutSetWindow(text_window);
    glutPostRedisplay();
    t1us=tus;
  }
}

/* change view angle, exit upon ESC */
/* ARGSUSED1 */
void key(unsigned char k, int x, int y)
{
  switch (k) {
  case 'a':
    view.rotz += 5.0;
    break;
  case 'd':
    view.rotz -= 5.0;
    break;
  case 'w':
    if(view.rotx<=85)
      view.rotx += 5.0;
    break;
  case 's':
    if(view.rotx>=5)
    view.rotx -= 5.0;
    break;
  case 'h':
    ShowHelp= !ShowHelp;
    break;
    case 'g':
      degRad= !degRad;
    break;
  case 27:  /* Escape */
    cleanup();
    gra_thd->shared_finish=0;
    exit(0);
    break;
  default:
    return;

  }
  redisplay_all();
}

void special(int k, int x, int y)
{
  switch (k) {
  case GLUT_KEY_UP:
    if(view.yPos<4)
      view.yPos += 0.5;
    break;

  case GLUT_KEY_DOWN:
    if(view.yPos>-10)
      view.yPos -= 0.5;
    break;
  case GLUT_KEY_LEFT:
    if(view.xPos>-7)
      view.xPos -= 0.5;
    break;
  case GLUT_KEY_RIGHT:
    if(view.xPos<7)
      view.xPos += 0.5;
    break;
  case GLUT_KEY_PAGE_UP:
    if(view.zPos<25)
      view.zPos += 0.5;
    break;
  case GLUT_KEY_PAGE_DOWN:
    if(view.zPos>-20)
      view.zPos -= 0.5;
    break;
  default:
    return;
  }
  redisplay_all();
}



// mouse functions
// void
// command_motion(int x, int y)
// {
//   if((x>buttons[0].x-buttons[0].width/2)&&(x<buttons[0].x+buttons[0].width/2)&&(y>buttons[0].y-buttons[0].height/2)&&(y<buttons[0].y+buttons[0].height/2))
//     buttons[0].color=green;
//   else
//     buttons[0].color=yellow;
//
//   if((x>buttons[1].x-buttons[1].width/2)&&(x<buttons[1].x+buttons[1].width/2)&&(y>buttons[1].y-buttons[1].height/2)&&(y<buttons[1].y+buttons[1].height/2))
//     buttons[1].color=green;
//   else
//     buttons[1].color=yellow;
//
//   if((x>buttons[2].x-buttons[2].width/2)&&(x<buttons[2].x+buttons[2].width/2)&&(y>buttons[2].y-buttons[2].height/2)&&(y<buttons[2].y+buttons[2].height/2))
//     buttons[2].color=green;
//   else
//     buttons[2].color=yellow;
//
//   if((x>buttons[3].x-buttons[3].width/2)&&(x<buttons[3].x+buttons[3].width/2)&&(y>buttons[3].y-buttons[3].height/2)&&(y<buttons[3].y+buttons[3].height/2))
//     buttons[3].color=green;
//   else
//     buttons[3].color=yellow;
//
//     glutSetWindow(command);
//     glutPostRedisplay();
// }

// void
// command_mouse(int button, int state, int x, int y)
// {
//     if (state == GLUT_DOWN) {
//         if((x>buttons[0].x-buttons[0].width/2)&&(x<buttons[0].x+buttons[0].width/2)&&(y>buttons[0].y-buttons[0].height/2)&&(y<buttons[0].y+buttons[0].height/2))
//     view.rotz += 5.0;
//   if((x>buttons[1].x-buttons[1].width/2)&&(x<buttons[1].x+buttons[1].width/2)&&(y>buttons[1].y-buttons[1].height/2)&&(y<buttons[1].y+buttons[1].height/2))
//     view.rotz -= 5.0;
//   if((x>buttons[2].x-buttons[2].width/2)&&(x<buttons[2].x+buttons[2].width/2)&&(y>buttons[2].y-buttons[2].height/2)&&(y<buttons[2].y+buttons[2].height/2))
//     view.rotx += 5.0;
//   if((x>buttons[3].x-buttons[3].width/2)&&(x<buttons[3].x+buttons[3].width/2)&&(y>buttons[3].y-buttons[3].height/2)&&(y<buttons[3].y+buttons[3].height/2))
//     view.rotx -= 5.0;
//
//     }
//
//
//
//     redisplay_all();
// }

void cleanup(void)
{
   glDeleteLists(wamBase, 1);
   glDeleteLists(wamShoulder_1DOF, 1);
   glDeleteLists(wamJoint_2DOF, 1);
   glDeleteLists(wamLink_3DOF, 1);
   glDeleteLists(wamLink_4DOF, 1);
   glDeleteLists(wamJoint_5DOF, 1);
   glDeleteLists(wamJoint_6DOF, 1);
   glDeleteLists(wamEndPlate, 1);
   glDeleteLists(wamReferencial, 1);
   glutDestroyWindow(win);
}

void drawstr(GLuint x, GLuint y, GLvoid *font_style, char* format, ...)
{
    va_list args;
    char buffer[255], *s;

    va_start(args, format);
    vsprintf(buffer, format, args);
    va_end(args);

    glRasterPos2i(x, y);
    for (s = buffer; *s; s++)
        glutBitmapCharacter(font_style, *s);
}

void printString(GLvoid *font_style, const char *s)
{
   while (*s) {
      glutBitmapCharacter(font_style, (int) *s);
      s++;
   }
}



void GraphicThread::read_shared_memory(){
   pthread_mutex_lock( shared_mutex );
   wam7DOFparam[0].theta= shared_angle[0]+theta1;
   wam7DOFparam[1].theta= shared_angle[1]+theta2;
   wam7DOFparam[2].theta= shared_angle[2]+theta3;
   wam7DOFparam[3].theta= shared_angle[3]+theta4;
   wam7DOFparam[4].theta= shared_angle[4]+theta5;
   wam7DOFparam[5].theta= shared_angle[5]+theta6;
   wam7DOFparam[6].theta= shared_angle[6]+theta7;
   pthread_mutex_unlock( shared_mutex );
}
