//
//
//Author: Pedro Nobre <pedrognobre@gmail.com>, (C) 2007
//
//
/***************************************************************************
 *   Copyright (C) 2007 by Pedro Nobre                                     *
 *   pedrognobre@gmail.com                                                 *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <GL/glut.h>

#include "wamComponents.h"






static GLUquadric *Q;

 static GLfloat dirty_white[4] = {  0.78,0.75,0.74};
 static GLfloat light_metalic[4] = {0.59,0.61,0.62};
 static GLfloat dark_metalic[4] = {0.27,0.27,0.27};
 static GLfloat dark_grey[4] = {0.1,0.1,0.1};
 static GLfloat blue[4] = {0.49,0.58,0.67};
 static GLfloat red[4] = {0.99,0.28,0.28};
 static GLfloat green[4] = {0.26,0.99,0.28};
 static GLfloat back_collor[] ={0.2,0.060,0.8};
 static GLfloat elbow_dark_collor[] ={0.35,0.35,0.39};


void base(GLfloat width, GLfloat height, GLfloat depth){
 
  Q=gluNewQuadric();
  glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, dirty_white);
  glShadeModel(GL_FLAT);
  glNormal3f(0.0, 0.0, 1.0);
  
  
  glShadeModel(GL_FLAT);

  glNormal3f(0.0, 0.0, 1.0);
  glBegin(GL_QUADS);
  glVertex3f(0,-depth/2,0);
  glVertex3f(width-depth/2,-depth/2,0);
  glVertex3f(width-depth/2,depth/2,0);
  glVertex3f(0,depth/2,0);  
  glEnd();
  
  glNormal3f(0.0, 0.0, -1.0);
  glBegin(GL_QUADS);
  glVertex3f(0,depth/2,-height);
  glVertex3f(width-depth/2,depth/2,-height);
  glVertex3f(width-depth/2,-depth/2,-height);
  glVertex3f(0,-depth/2,-height);  
  glEnd();
  
  glNormal3f(0.0, -1.0, 0.0);
  glBegin(GL_QUADS);
  glVertex3f(0,-depth/2,-height);
  glVertex3f(width-depth/2,-depth/2,-height);
  glVertex3f(width-depth/2,-depth/2,0);
  glVertex3f(0,-depth/2,0);
  glEnd();
  
  glNormal3f(0.0, 1.0, 0.0);
  glBegin(GL_QUADS);
  glVertex3f(0,depth/2,0);
  glVertex3f(width-depth/2,depth/2,0);
  glVertex3f(width-depth/2,depth/2,-height);
  glVertex3f(0,depth/2,-height);
  glEnd();
  
  glNormal3f(1.0, 0.0, 0.0);
  glBegin(GL_QUADS);
  glVertex3f(width-depth/2,-depth/2,-height);
  glVertex3f(width-depth/2,depth/2,-height);
  glVertex3f(width-depth/2,depth/2,0);
  glVertex3f(width-depth/2,-depth/2,0); 
  glEnd();

  gluQuadricOrientation(Q, GLU_OUTSIDE);
  gluDisk(Q,0,depth/2,50,1);
  glPushMatrix();
  
  glTranslatef(0.0, 0.0, -height);
  gluCylinder(Q,depth/2,depth/2,height,50,4);
  gluQuadricOrientation(Q, GLU_INSIDE);
  gluDisk(Q,0,depth/2,50,1);
  glPopMatrix();
  
}
void drawShoulder(GLfloat depth1,GLfloat depth2,GLfloat depth3, GLfloat whidth, GLfloat height){
  Q=gluNewQuadric();
  glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, dirty_white);
  GLfloat normAng;
  
  //depth1 front, back and side faces
  glNormal3f(0.0, -1.0, 0.0);
  glBegin(GL_QUADS);
  glVertex3f(-whidth/2,0,0);
  glVertex3f(whidth/2,0,0);
  glVertex3f(whidth/2,0,height);
  glVertex3f(-whidth/2,0,height);
  glEnd();
  glNormal3f(0.0, 1.0, 0.0);
  glBegin(GL_QUADS);
  glVertex3f(-whidth/2,depth1,height);
  glVertex3f(whidth/2,depth1,height);
  glVertex3f(whidth/2,depth1,0);
  glVertex3f(-whidth/2,depth1,0);
  glEnd();
  glNormal3f(-1.0, 0.0, 0.0);
  glBegin(GL_QUADS);
  glVertex3f(-whidth/2,0,0);
  glVertex3f(-whidth/2,0,height);
  glVertex3f(-whidth/2,depth1,height);
  glVertex3f(-whidth/2,depth1,0);
  glEnd();
  glNormal3f(1.0, 0.0, 0.0);
  glBegin(GL_QUADS);
  glVertex3f(whidth/2,depth1,0);
  glVertex3f(whidth/2,depth1,height);
  glVertex3f(whidth/2,0,height);
  glVertex3f(whidth/2,0,0);
  glEnd();
    
  
  //depth2 side faces
  glNormal3f(-1.0, 0.0, 0.0);
  glBegin(GL_QUADS);
  glVertex3f(-whidth/2+5,depth1,0);
  glVertex3f(-whidth/2+5,depth1,height);
  glVertex3f(-whidth/2+5,depth2+depth1,height);
  glVertex3f(-whidth/2+5,depth2+depth1,0);
  glEnd();
  glNormal3f(1.0, 0.0, 0.0);
  glBegin(GL_QUADS);
  glVertex3f(whidth/2-5,depth2+depth1,0);
  glVertex3f(whidth/2-5,depth2+depth1,height);
  glVertex3f(whidth/2-5,depth1,height);
  glVertex3f(whidth/2-5,depth1,0);
  glEnd();
  
  normAng=atan(depth3/10);
  //depth3 obliquous side faces
  glNormal3f(-1.0*sin(normAng), 1*cos(normAng), 0.0);
  glBegin(GL_QUADS);
  glVertex3f(-whidth/2+5,depth2+depth1,0);
  glVertex3f(-whidth/2+5,depth2+depth1,height);
  glVertex3f(-70,depth3+depth2+depth1,height);
  glVertex3f(-70,depth3+depth2+depth1,0);
  glEnd();
  glNormal3f(1.0*sin(normAng), 1*cos(normAng), 0.0);
  glBegin(GL_QUADS);
  glVertex3f(70,depth3+depth2+depth1,0);
  glVertex3f(70,depth3+depth2+depth1,height);
  glVertex3f(whidth/2-5,depth2+depth1,height);
  glVertex3f(whidth/2-5,depth2+depth1,0);
  glEnd();
  
  //depth3 front faces
  glNormal3f(0.0, 1.0, 0.0);
  glBegin(GL_QUADS);
  glVertex3f(-70,depth3+depth2+depth1,height);
  glVertex3f(70,depth3+depth2+depth1,height);
  glVertex3f(70,depth3+depth2+depth1,0);
  glVertex3f(-70,depth3+depth2+depth1,0);
  glEnd();
  
  //bottom faces
  
  glNormal3f(0.0, 0.0,-1.0);
  glBegin(GL_QUADS);
  glVertex3f(-whidth/2+5,depth1,0);
  glVertex3f(-whidth/2+5,depth2+depth1,0);
  glVertex3f(whidth/2-5,depth2+depth1,0);
  glVertex3f(whidth/2-5,depth1,0);
  glEnd();
  
  glNormal3f(0.0, 0.0,-1.0);
  glBegin(GL_QUADS);
  glVertex3f(-whidth/2+5,depth2+depth1,0);
  glVertex3f(-70,depth3+depth2+depth1,0);
  glVertex3f(70,depth3+depth2+depth1,0);
  glVertex3f(whidth/2-5,depth2+depth1,0);
  glEnd();
  
  
  
  glNormal3f(1.0, 0.0, 0.0);
  glBegin(GL_QUADS);
  glVertex3f(whidth/2-5,depth2+depth1,0);
  glVertex3f(whidth/2-5,depth2+depth1,height);
  glVertex3f(whidth/2-5,depth1,height);
  glVertex3f(whidth/2-5,depth1,0);
  glEnd();
  
  
  glPushMatrix();
  glTranslatef(0.0,0.0,height);
  glRotatef(-90.0, 1.0, 0.0, 0.0);
  gluQuadricOrientation(Q, GLU_INSIDE);
  gluDisk(Q,0,whidth/2,50,1);
  gluQuadricOrientation(Q, GLU_OUTSIDE);
  gluCylinder(Q,whidth/2,whidth/2,depth1,50,4);
  glTranslatef(0.0,0.0,depth1);
  gluDisk(Q,0,whidth/2,50,1);
  gluCylinder(Q,whidth/2-5,whidth/2-5,depth2,50,4);
  glTranslatef(0.0,0.0,depth2);
  gluCylinder(Q,whidth/2-5,70,depth3,50,4);
  glTranslatef(0.0,0.0,depth3);
  gluDisk(Q,0,70,50,1);
  glPopMatrix();
}

void shoulder_1DOF(GLfloat baseDepth, GLfloat shoulderDepth, GLfloat shoulderWhidth, GLfloat shoulderHeight, GLfloat cavityDepth)
{
  float angle;
  GLfloat depth1, depth2, depth3;
  glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, dirty_white);
  /* to compute depth1 I assumed that this segment starts at cavityDepth/2 and end when intersect the base cylinder.*/
  angle=asin((shoulderWhidth/2)/(baseDepth/2));
  depth1=(baseDepth/2)*cos(angle)-cavityDepth/2;
  depth2=((shoulderDepth-cavityDepth)/2-depth1)*0.4;
  depth3=((shoulderDepth-cavityDepth)/2-depth1)*0.6;
  glPushMatrix();  
  glTranslatef(0.0, 0.0, 5);
  glTranslatef(0.0, cavityDepth/2, 0.0);
  drawShoulder(depth1,depth2,depth3, shoulderWhidth, shoulderHeight-5);
  glTranslatef(0.0, -cavityDepth, 0.0);
  glRotatef(180.0, 0.0, 0.0, 1.0);
  drawShoulder(depth1,depth2,depth3, shoulderWhidth, shoulderHeight-5);
  glPopMatrix();
  
  Q=gluNewQuadric();
  gluQuadricOrientation(Q, GLU_OUTSIDE);
  /// \todo REview this values;
  
  glPushMatrix();
  glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, light_metalic);
  gluCylinder(Q,baseDepth/2,baseDepth/2,5,50,4);
  glTranslatef(0.0, 0.0, 5);
  glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, dirty_white);
  gluCylinder(Q,baseDepth/2,baseDepth/2,50,50,4);
  glTranslatef(0.0, 0.0, 50);
  gluCylinder(Q,baseDepth/2,shoulderWhidth/2,30,50,4);
  glTranslatef(0.0, 0.0, 30);
  gluDisk(Q,0,shoulderWhidth/2,50,1);
  glPopMatrix();
}

void joint_2DOF(GLfloat cavityDepth, GLfloat diam1, GLfloat diam2, GLfloat height){
  Q=gluNewQuadric();
  glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, light_metalic);
  
  gluQuadricOrientation(Q, GLU_OUTSIDE);
  glPushMatrix();
  glTranslatef(0.0, -cavityDepth/2, 0.0);
  glRotatef(-90, 1.0, 0.0, 0.0);
  gluCylinder(Q,diam1/2,diam1/2,cavityDepth,50,4);  
  glPopMatrix();
  glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, dirty_white);
  gluCylinder(Q,diam2/2,diam2/2,height,50,4);
  glPushMatrix();
  glTranslatef(0.0,0.0, height);
  gluDisk(Q,0,diam2/2,50,1);
  glPopMatrix(); 
}

void elbow_link_conection(GLfloat diam,GLfloat square, GLfloat distance){
  int i;
  //z normals has only 4 ellements becuse his values reapeats 4 times in the complete 360º
  GLfloat points[64][2], normals[64][2],upPoints[64][2], zNormals[16];
  
  for (i = 0; i < 64; i++) {
      const float angle = i * 5.625;
      float angleZ;
      float x = cos(DEG_TO_RAD(angle));
      float y = sin(DEG_TO_RAD(angle));
      normals[i][0] = x;
      normals[i][1] = y;
      x *= diam/2;
      y *= diam/2;
      points[i][0] = x;
      points[i][1] = y;
      
     //compute top face vertexes
     if(angle>=45&&angle<=135){
       upPoints[i][0]=-tan(DEG_TO_RAD(angle-90))*square/2;
       upPoints[i][1]=square/2;      
     }
     else if(angle>=135&&angle<=225){
       upPoints[i][0]=-square/2;
       upPoints[i][1]=-tan(DEG_TO_RAD(angle-180))*square/2;
     }
     else if(angle>225&&angle<315){
       upPoints[i][0]=tan(DEG_TO_RAD(angle-270))*square/2;
       upPoints[i][1]=-square/2;
     }
     else{
       upPoints[i][0]=square/2;
       upPoints[i][1]=tan(DEG_TO_RAD(angle))*square/2;
     }
     
     if(i<8){
         angleZ=atan(distance/(upPoints[i][0]-points[i][0]));
         zNormals[i]=-cos(angleZ);
     }
     else if(i<16){
       angleZ=atan(distance/(upPoints[i][1]-points[i][1]));
       zNormals[i]=-cos(angleZ);
     }
  
   }

   /* front face */
   glNormal3f(0, 0, 1);
   glBegin(GL_QUADS);
   glVertex3f(square/2, -square/2, distance);
   glVertex3f(square/2, square/2, distance);
   glVertex3f(-square/2, square/2, distance);
   glVertex3f(-square/2, -square/2, distance);
   glEnd();

   /* edge */
   glBegin(GL_QUAD_STRIP);
   for (i = 0; i <= 64; i++) {
      const int j = i % 64;
      glNormal3f(normals[j][0], normals[j][1], zNormals[(i%16)]);
      glVertex3f(upPoints[j][0], upPoints[j][1], distance);
      glVertex3f(points[j][0], points[j][1], 0);
   }
   glEnd();
}

//draw the intern suport of the elbow which is connected to the 3DOF link
void inside_suport(GLfloat diam, GLfloat height, GLfloat elbow, GLfloat suportWidth, GLfloat ajust_distance){
  float angle, x, z;
  
  //round area 
  glPushMatrix();
  glTranslatef(-elbow, 0, height);
  glRotatef(-90, 1.0, 0.0, 0.0);  
  gluQuadricOrientation(Q, GLU_INSIDE);
  gluDisk(Q,0,37,50,1);
  gluQuadricOrientation(Q, GLU_OUTSIDE);
  gluCylinder(Q,37,37,suportWidth,50,4);
  glTranslatef(0.0, 0.0, suportWidth); 
  gluDisk(Q,0,37,50,1);
  glPopMatrix();
  
  //front face
  glNormal3f(0, -1, 0);
  glBegin(GL_POLYGON);
  glVertex3f(diam/2, 0, ajust_distance);
  glVertex3f(diam/2, 0, height);
  glVertex3f(-elbow+15, 0, height+33);
  glVertex3f(-elbow-15, 0, height-33);
  glVertex3f(-diam/2, 0, 41);
  glVertex3f(-diam/2, 0, ajust_distance);
  glEnd();
  
  //back face
  glNormal3f(0, 1, 0);
  glBegin(GL_POLYGON);
  glVertex3f(-elbow+15, 0+suportWidth, height+33);
  glVertex3f(diam/2, 0+suportWidth, height);
  glVertex3f(diam/2, 0+suportWidth, ajust_distance);
  glVertex3f(-diam/2, 0+suportWidth, ajust_distance);
  glVertex3f(-diam/2, 0+suportWidth, 41);
  glVertex3f(-elbow-15, 0+suportWidth, height-33);
  glEnd();
  
  angle=atan((diam/2-(-elbow+15))/33);
  x=sin(angle);
  z=cos(angle);
  
  glNormal3f(x, 0, z);
  glBegin(GL_QUADS);
  glVertex3f(diam/2, 0, height);
  glVertex3f(diam/2, suportWidth, height);
  glVertex3f(-elbow+15, suportWidth, height+33);
  glVertex3f(-elbow+15, 0, height+33);
  glEnd();
  
  glNormal3f(1, 0, 0);
  glBegin(GL_QUADS);
  glVertex3f(diam/2, suportWidth, ajust_distance);
  glVertex3f(diam/2, suportWidth, height);
  glVertex3f(diam/2, 0, height);
  glVertex3f(diam/2, 0, ajust_distance);
  glEnd();
  
  glNormal3f(-x, 0, -z);
  glBegin(GL_QUADS);
  glVertex3f(-diam/2, 0, 41);
  glVertex3f(-elbow-15, 0, height-33);
  glVertex3f(-elbow-15, 0+suportWidth, height-33);
  glVertex3f(-diam/2, 0+suportWidth, 41);
  glEnd();
  
  glNormal3f(-1, 0, 0);
  glBegin(GL_QUADS);
  glVertex3f(-diam/2, 0, ajust_distance);
  glVertex3f(-diam/2, 0, 41);
  glVertex3f(-diam/2, 0+suportWidth, 41);
  glVertex3f(-diam/2, 0+suportWidth, ajust_distance);
  
  glEnd();
  
}

// draw the extern suport of elbow which is conected to the 3DOF link
void outside_suport(GLfloat diam, GLfloat height, GLfloat elbow, GLfloat externSuportWidth, GLfloat ajust_distance){
  float angle, x, z;
  //round area 
  glPushMatrix();
  glTranslatef(-elbow, 0, height);
  glRotatef(-90, 1.0, 0.0, 0.0);
  gluQuadricOrientation(Q, GLU_INSIDE);
  gluDisk(Q,0,32,50,1);
  gluQuadricOrientation(Q, GLU_OUTSIDE);
  gluCylinder(Q,32,32,externSuportWidth,50,4);
  glTranslatef(0, 0, externSuportWidth);
  gluDisk(Q,0,32,50,1);
  glPopMatrix();
  
  //front face
  glNormal3f(0, -1, 0);
  glBegin(GL_POLYGON);
  glVertex3f(diam/2-15, 0, ajust_distance);
  glVertex3f(diam/2-15, 0, height+1);
  glVertex3f(-elbow+15, 0, height+28);
  glVertex3f(-elbow-15, 0, height-28);
  glVertex3f(-diam/2+5, 0, 44);
  glVertex3f(-diam/2+5, 0, ajust_distance);
  glEnd();
  
  //back face
  glNormal3f(0, 1, 0);
  glBegin(GL_POLYGON);
  glVertex3f(-elbow+15,externSuportWidth, height+28);
  glVertex3f(diam/2-15, externSuportWidth, height+1);
  glVertex3f(diam/2-15, externSuportWidth, ajust_distance);
  glVertex3f(-diam/2+5, externSuportWidth, ajust_distance);
  glVertex3f(-diam/2+5, externSuportWidth, 44);
  glVertex3f(-elbow-15, externSuportWidth, height-28);
  glEnd();
  
  angle=atan(((diam/2-5)-(-elbow+15))/33);
  x=sin(angle);
  z=cos(angle);
  
  glNormal3f(x, 0, z);
  glBegin(GL_QUADS);
  glVertex3f(diam/2-15, 0, height+1);
  glVertex3f(diam/2-15, externSuportWidth, height+1);
  glVertex3f(-elbow+15, externSuportWidth, height+28);
  glVertex3f(-elbow+15, 0, height+28);
  glEnd();
  
  glNormal3f(1, 0, 0);
  glBegin(GL_QUADS);
  glVertex3f(diam/2-15, externSuportWidth, ajust_distance);
  glVertex3f(diam/2-15, externSuportWidth, height+1);
  glVertex3f(diam/2-15, 0, height+1);
  glVertex3f(diam/2-15, 0, ajust_distance);
  glEnd();
  
  glNormal3f(-x, 0, -z);
  glBegin(GL_QUADS);
  glVertex3f(-diam/2+5, 0, 44);
  glVertex3f(-elbow-15, 0, height-28);
  glVertex3f(-elbow-15, externSuportWidth, height-28);
  glVertex3f(-diam/2+5, externSuportWidth, 44);
  glEnd();
  
  glNormal3f(-1, 0, 0);
  glBegin(GL_QUADS);
  glVertex3f(-diam/2+5, 0, ajust_distance);
  glVertex3f(-diam/2+5, 0, 44);
  glVertex3f(-diam/2+5, externSuportWidth, 44);
  glVertex3f(-diam/2+5, externSuportWidth, ajust_distance);
  glEnd();
  
  glNormal3f(0, 0, -1);
  glBegin(GL_QUADS);
  glVertex3f(-diam/2+5, 0, ajust_distance);
  glVertex3f(-diam/2+5, externSuportWidth, ajust_distance);
  glVertex3f(diam/2-15, externSuportWidth, ajust_distance);
  glVertex3f(diam/2-15, 0, ajust_distance);
  glEnd();
}

//draw the part of the elbow connected to the 3DOF LINK
void elbow1(GLfloat diam, GLfloat height, GLfloat elbow){
  const float ajust_distance=19;
  float suportWidth=7, externSuportWidth=7;
  
  elbow_link_conection(diam, diam, ajust_distance);
  glPushMatrix();
  glTranslatef(-elbow, -diam/2, height);
  glRotatef(-90, 1.0, 0.0, 0.0);
   
  gluQuadricOrientation(Q, GLU_OUTSIDE);
  gluCylinder(Q,20,20,diam,50,4);
  glPopMatrix();
  
  
  glPushMatrix();
  glTranslatef(0.0, -diam/2, 0);
  inside_suport(diam, height, elbow, suportWidth, ajust_distance);
  glPopMatrix();
  
  glPushMatrix();
  glTranslatef(0.0, diam/2-suportWidth, 0);
  inside_suport(diam, height, elbow, suportWidth, ajust_distance);
  glPopMatrix();
  
  glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, elbow_dark_collor);
  
  glPushMatrix();
  glTranslatef(0.0, -diam/2-externSuportWidth, 0);
  outside_suport(diam, height, elbow,externSuportWidth, ajust_distance);
  glPopMatrix();
  
  glPushMatrix();
  glTranslatef(0.0, diam/2, 0);
  outside_suport(diam, height, elbow,externSuportWidth, ajust_distance);
  glPopMatrix();
}



//Draw the inner suport of the elboy which is connecte to the 4DOF link
void elbow_support2(GLfloat diam, GLfloat height, GLfloat elbow,GLfloat suportWidth,GLfloat internSuportWidth){
  Q=gluNewQuadric();
  float angle, x,z;
  glPushMatrix();
  glRotatef(-90,1.0,0.0,0.0);
  gluQuadricOrientation(Q, GLU_INSIDE);
  gluDisk(Q,0,50,50,1);
  gluQuadricOrientation(Q, GLU_OUTSIDE);
  gluCylinder(Q,50,50,internSuportWidth ,50,4);
  glTranslatef(0.0,0.0,internSuportWidth);
  gluDisk(Q,0,50,50,1);
  glPopMatrix();
  
  glNormal3f(0, -1, 0);
  glBegin(GL_POLYGON);
  glVertex3f(48.0,0.0,10.0);
  glVertex3f(elbow+diam/2-suportWidth,0.0,50.0);
  glVertex3f(elbow-diam/2+suportWidth,0.0,50.0);
  glVertex3f(0.0,0.0,10.0);
  glEnd();
  
  glNormal3f(0, 1, 0);
  glBegin(GL_POLYGON);
  
  glVertex3f(0.0,internSuportWidth,10.0);
  glVertex3f(elbow-diam/2+suportWidth,internSuportWidth,50.0);
  glVertex3f(elbow+diam/2-suportWidth,internSuportWidth,50.0);
  glVertex3f(48.0,internSuportWidth,10.0);
  glEnd();
  
  angle=atan(40/(elbow+diam/2-suportWidth));
  x=sin(angle);
  z=-cos(angle);
  glNormal3f(x, 0.0, z);
  glBegin(GL_QUADS);
  glVertex3f(48.0,0,10.0);
  glVertex3f(48.0,internSuportWidth,10.0);
  glVertex3f(elbow+diam/2-suportWidth,internSuportWidth,50.0);
  glVertex3f(elbow+diam/2-suportWidth,0,50.0);
  glEnd();
  
}

//draw the part of the elbow connected to the 4DOF LINK
void elbow2(GLfloat diam, GLfloat height, GLfloat elbow){
  //this values represents the elbow suport cilinders thickness
  //suportWidth mustt be equal to the one difined earlier 
  float suportWidth=7, internSuportWidth=22;
  Q=gluNewQuadric();
  glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, dark_metalic);
  glPushMatrix();
  glTranslatef(elbow,0,70);
  glRotatef(180,1.0,0.0,0.0);
  elbow_link_conection(diam, diam-suportWidth*2, 20);
  glPopMatrix();
  
  
  glPushMatrix();  
  glTranslatef(0.0,-diam/2+suportWidth,0.0);
  elbow_support2(diam,height,elbow,suportWidth,internSuportWidth);
  glPopMatrix();
  
  glPushMatrix();  
  glTranslatef(0.0,diam/2-suportWidth-internSuportWidth,0.0);
  elbow_support2(diam,height,elbow,suportWidth,internSuportWidth);
  glPopMatrix();
  

  
}

void link_3DOF(GLfloat diam, GLfloat height, GLfloat elbow){
  
  Q=gluNewQuadric();
  glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, light_metalic);
  gluQuadricOrientation(Q, GLU_OUTSIDE);
  gluCylinder(Q,diam/2,diam/2, height-81,64,4);

  glPushMatrix();
  glTranslatef(-0, 0, height-81);
  elbow1(diam, 81,elbow);
  glPopMatrix();
}

void link_4DOF_WRIST(GLfloat diam, GLfloat height, GLfloat elbow){
  Q=gluNewQuadric();
  gluQuadricOrientation(Q, GLU_OUTSIDE);
  glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, light_metalic);
  elbow2(diam, 70,elbow);
  
  glPushMatrix();
  glTranslatef(elbow,0,70);
  glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, light_metalic);
  gluCylinder(Q,diam/2,diam/2,height-elbow-20,50,4);
  glTranslatef(0.0,0.0,height-elbow-20);
  gluDisk(Q,0,diam/2,50,1);
  glPopMatrix();
}

void joint_5DOF_WRIST(GLfloat diam, GLfloat height){
  Q=gluNewQuadric();
  glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, blue);
  
  glPushMatrix();
  glTranslatef(0.0,0.0,height/2+7.5);
  glutSolidCube(height+20);
  glPopMatrix();

  glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, light_metalic);
  
  glPushMatrix();
  glTranslatef(0.0,0.0,height);
  gluQuadricOrientation(Q, GLU_OUTSIDE);
  glRotatef(90,1.0,0.0,0.0);
  glTranslatef(0.0,0.0,-diam/2+5);
  gluCylinder(Q,15,15,diam-10,50,4);
  glPopMatrix();
}

void side_6DOF_suport(GLfloat diam){
 GLfloat xyOutPoints[35][2], xyInPoints[35][2], zPoints[35],normals[35][2];
 const float width=20;
 float l1;
 int i;
  
  for (i = -17; i <= 17; i++) {
      float angle, angle2,x,y;
      float angleZ;
      
      
      if(i<=-5)
        angle=(i+1)*5.625;
      else if(i<5){
        angle=i*5.625;
        angle2=i*22.5;
      }
      else
        angle=(i-1)*5.625;
 
      x = cos(DEG_TO_RAD(angle));
      y = sin(DEG_TO_RAD(angle));

      normals[17+i][0] = x;

      normals[17+i][1] = y;

      
      xyOutPoints[17+i][0] = x*diam/2;
      xyOutPoints[17+i][1] = y*diam/2;
      
      xyInPoints[17+i][0]= x*(diam/2-width);
      xyInPoints[17+i][1]= y*(diam/2-width);
      
      if(i>=-5&&i<=5)
        xyInPoints[17+i][1]= y*diam/2;

      if(i<=-5)
        zPoints[17+i]=diam/2*(-sin(DEG_TO_RAD(angle)));
      else if(i<5){
        l1=sin(DEG_TO_RAD(22.5))*sqrt(diam/2*diam/2+(diam/2*(1-cos(angle))*diam/2*(1-cos(angle))));
        zPoints[17+i]=-cos(DEG_TO_RAD(angle2))*l1;
      }
      else
        zPoints[17+i]=diam/2*sin(DEG_TO_RAD(angle));
   }
   


   /* edge */
   glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, dark_metalic);
   glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, light_metalic);
   
   
   // to plot of the face must be divided into 3 stages to alow different collors
   glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, dark_metalic);
   glBegin(GL_QUAD_STRIP);
   for (i = 0; i <=12; i++) {
      glNormal3f(normals[i][0], normals[i][1], 0);
      glVertex3f(xyOutPoints[i][0], xyOutPoints[i][1], diam/2);
      glVertex3f(xyOutPoints[i][0], xyOutPoints[i][1], zPoints[i]);
   }
   glEnd();
   glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, light_metalic);
   glBegin(GL_QUAD_STRIP);
   for (i = 12; i <=21; i++) {
      glNormal3f(normals[i][0], normals[i][1], 0);
      glVertex3f(xyOutPoints[i][0], xyOutPoints[i][1], diam/2);
      glVertex3f(xyOutPoints[i][0], xyOutPoints[i][1], zPoints[i]);
   }
   glEnd();
   glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, dark_metalic);
   glBegin(GL_QUAD_STRIP);
   for (i = 21; i <=34; i++) {
      glNormal3f(normals[i][0], normals[i][1], 0);
      glVertex3f(xyOutPoints[i][0], xyOutPoints[i][1], diam/2);
      glVertex3f(xyOutPoints[i][0], xyOutPoints[i][1], zPoints[i]);
   }
   glEnd();
   
   
   
   glBegin(GL_QUAD_STRIP);
   for (i = 0; i <=12; i++) {
      glNormal3f(-normals[i][0], -normals[i][1], 0);
      glVertex3f(xyInPoints[i][0], xyInPoints[i][1], zPoints[i]);
      glVertex3f(xyInPoints[i][0], xyInPoints[i][1], diam/2);
   }
   glEnd();
   glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, light_metalic);
   glBegin(GL_QUAD_STRIP);
   for (i = 12; i <=21; i++) {
      glNormal3f(-normals[i][0], -normals[i][1], 0);
      glVertex3f(xyInPoints[i][0], xyInPoints[i][1], zPoints[i]);
      glVertex3f(xyInPoints[i][0], xyInPoints[i][1], diam/2);
   }
   glEnd();
   glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, dark_metalic);
   glBegin(GL_QUAD_STRIP);
   for (i = 21; i <=34; i++) {
      glNormal3f(-normals[i][0], -normals[i][1], 0);
      glVertex3f(xyInPoints[i][0], xyInPoints[i][1], zPoints[i]);
      glVertex3f(xyInPoints[i][0], xyInPoints[i][1], diam/2);
   }
   glEnd();

   //ax+by+cz=d -> (a,b,c) defines a normal vector to the plan
   for(i=0;i<34;i++){
     float u[3], v[3];
     float bottomNormal[3];
     
     u[0]=xyInPoints[i+1][0]-xyOutPoints[i][0];
     u[1]=xyInPoints[i+1][1]-xyOutPoints[i][1];
     u[2]=zPoints[i+1]-zPoints[i];
     
     v[0]=xyOutPoints[i+1][0]-xyInPoints[i][0];
     v[1]=xyOutPoints[i+1][1]-xyInPoints[i][1];
     v[2]=zPoints[i+1]-zPoints[i];
     
     
     //normal=u*v
     bottomNormal[0]=(u[1]*v[2]-v[1]*u[2]);
     bottomNormal[1]=(u[2]*v[0]-v[2]*u[0]);
     bottomNormal[2]=(u[0]*v[1]-v[0]*u[1]);
     
     if(i<12||i>21)
       glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, dark_metalic);
     else
       glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, light_metalic);

     glBegin(GL_QUADS);
     glNormal3f(bottomNormal[0], bottomNormal[0], bottomNormal[0]);
     glVertex3f(xyOutPoints[i][0], xyOutPoints[i][1], zPoints[i]);
     glVertex3f(xyInPoints[i][0], xyInPoints[i][1], zPoints[i]);
     glVertex3f(xyInPoints[i+1][0], xyInPoints[i+1][1], zPoints[i+1]);
     glVertex3f(xyOutPoints[i+1][0], xyOutPoints[i+1][1], zPoints[i+1]);
     
     glEnd();
   }

}

void joint_6DOF_WRIST(GLfloat diam, GLfloat height){
  Q=gluNewQuadric();
  glPushMatrix();
  glRotatef(90,0.0,0.0,1.0);
  side_6DOF_suport(diam);
  glRotatef(180,0.0,0.0,1.0);
  side_6DOF_suport(diam);
  glPopMatrix();
  glPushMatrix();
  glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, light_metalic);
  
  glPushMatrix();
  glTranslatef(0,diam/2,0);
  gluQuadricOrientation(Q, GLU_INSIDE);
  gluDisk(Q,0,15,40,1);
  gluQuadricOrientation(Q, GLU_OUTSIDE);
  gluCylinder(Q,15,15,diam/2,40,4);
  glTranslatef(0,0,diam/2);
  gluDisk(Q,0,15,40,1);
  glPopMatrix();
  
  glTranslatef(0.0,0.0,diam/2);
  gluQuadricOrientation(Q, GLU_INSIDE);
  gluDisk(Q,0,diam/2,64,1);
  gluQuadricOrientation(Q, GLU_OUTSIDE);
  gluCylinder(Q,diam/2,diam/2,height-diam/2,64,4);
  glPopMatrix();
}

void endPlate(GLfloat diam){
  Q=gluNewQuadric();
  gluQuadricOrientation(Q, GLU_OUTSIDE);
  glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, light_metalic);
  gluDisk(Q,15,diam/2,64,1);
  gluCylinder(Q,diam/2,diam/2,4,64,4);
  gluQuadricOrientation(Q, GLU_INSIDE);
  gluCylinder(Q,diam/2-1,diam/2-1,4,64,4);
  
  glPushMatrix();
  glTranslatef(0.0,0.0,4);
  gluQuadricOrientation(Q, GLU_OUTSIDE);
  gluDisk(Q,diam/2-1,diam/2,64,1);
  glPopMatrix();
  
  glPushMatrix();
  glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, dark_metalic);
  gluQuadricOrientation(Q, GLU_OUTSIDE);
  glTranslatef(0.0,0.0,-5);
  gluDisk(Q,0,15,64,1);
  gluQuadricOrientation(Q, GLU_INSIDE);
  gluCylinder(Q,15,15,5,64,4);
  glPopMatrix();
  
  glPushMatrix();
  gluQuadricOrientation(Q, GLU_OUTSIDE);
  glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, dark_grey);
  glTranslatef(20.0,20.0,0.1);
  gluDisk(Q,0,2.5,64,1);
  glTranslatef(-40.0,0.0,0.0);
  gluDisk(Q,0,2.5,64,1);
  glTranslatef(0.0,-40.0,0.1);
  gluDisk(Q,0,2.5,64,1);
  glTranslatef(40.0,0.0,0.1);
  gluDisk(Q,0,2.5,64,1);
  glPopMatrix();
  
}

void referencial(){
  Q=gluNewQuadric();
  gluQuadricOrientation(Q, GLU_OUTSIDE);
  //z axis
  glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, blue);
  glPushMatrix();
  gluCylinder(Q,5,5,150,50,4);
  glTranslatef(0.0,0.0,150);
  gluCylinder(Q,15,0,50,50,4);
  glPopMatrix();
  
  //x axis
  glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, red);
  glPushMatrix();
  glRotatef(90,0,1,0);
  gluCylinder(Q,5,5,100,50,4);
  glTranslatef(0.0,0.0,100);
  gluCylinder(Q,15,0,50,50,4);
  glPopMatrix();
  
  glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, green);
  glPushMatrix();
  glRotatef(90,-1,0,0);
  gluCylinder(Q,5,5,100,50,4);
  glTranslatef(0.0,0.0,100);
  gluCylinder(Q,15,0,50,50,4);
  glPopMatrix();
  
}

void box(){
  const GLfloat x=1300,y=1300,z=1500, z_bottom=250;
  glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, back_collor);
  glBegin(GL_QUADS);
     glNormal3f(0.0, 0.0, 1.0);
          
     glVertex3f(-x, -y, -z_bottom);
     glVertex3f(x, -y, -z_bottom);
     glVertex3f(x, y, -z_bottom);
     glVertex3f(-x, y, -z_bottom);

     glEnd();
     

     
     glBegin(GL_QUADS);
     glNormal3f(0.0, -1.0, 0.0);
     glVertex3f(-x, y, -z_bottom);
     glVertex3f(x, y, -z_bottom);
     glVertex3f(x, y, z);
     glVertex3f(-x, y, z);
     glEnd();
     
     glBegin(GL_QUADS);
     glNormal3f(-1.0, 0.0, 0.0);
     glVertex3f(x, y, -z_bottom);
     glVertex3f(x, -y, -z_bottom);
     glVertex3f(x, -y, z);
     glVertex3f(x, y, z);
     glEnd();
  
  

}
