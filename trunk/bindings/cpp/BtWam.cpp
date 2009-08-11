/* ======================================================================== *
 *  Module ............. WAM-ROS
 *  File ............... BtWam.cpp
 *  Author ............. vw
 *  Creation Date ...... 30 July 2009
 *                                                                          *
 *  **********************************************************************  *
 *                                                                          *
 *  C++ wrapper for libbarrett. Allows for creation of a WAM object. 
 *
 * ======================================================================== */


#include <iostream>
#include <cassert>

/* System Libraries */
#include <signal.h>
#include <sys/mman.h>
#include <stdio.h>
#include <stdlib.h>`
#include <math.h>

/* Package Dependencies */
#include <syslog.h>

/* libbt dependencies */
extern "C" {
#include <libbarrett/wam.h>
//#include <libbarrett/wam_custom.h>
//#include <libbarrett/wam_local.h>
}

#include "BtWam.hpp"

/** Parses string from WAM function. Helper function.
 * \param jstr Character pointer of streamed string
 * \param dof DOF of wam, does not have to be 7 but is max value
 * \return array of doubles */
double * strToDbl(char * jstr, int dof)
{
    char store[20] = "";
    int current = 0;        //index pointer to string
    double * d_array;

    /* to keep track of which angle */
    int counter = 0;

    /*check if starting char is correct */
    if (*jstr != '<')
        return false;

    jstr += 2;

    /*begin parsing  */
    while(*jstr != '>')
    {
        //if character ascii value is betw. period and 9
        if ( ( (*jstr >= 46) & (*jstr <= 57) ) | (*jstr == '-') ) 
        {
            store[current++] = *jstr;
            jstr++;
        }
        //if comma, reset
        else if (*jstr == ',')   
        {
            jstr++;
            store[current] = 0;
            current = 0;
            d_array[counter++] = atof(store);         //stores values 
        }
        
        //skip if empty space
        else if (*jstr == ' ')
              jstr++;
        else
            return false;

    }
    d_array[counter++] = atof(store);   
    assert(counter == dof);
    return d_array;                   //checks if WAM DOF matches number of received values
}

/** ======================================================================== */

/** WAM class functions */
Wam::Wam(char * name):name(name)
{
}

Wam::~Wam()
{   
   //TODO: is bt_wam_local ever freed??
   
   /* Close the WAM */ 
   bt_wam_destroy(wam);

   /* Close syslog */
   closelog();
}
   
void Wam::init() throw (WamException)
{
   /* Initialize syslog */
   openlog("WAM", LOG_CONS | LOG_NDELAY, LOG_USER);
   

   /* Lock memory */
   mlockall(MCL_CURRENT | MCL_FUTURE);
   
   /* Open the WAM (or WAMs!) */
   bt_wam_create(&wam, name);
   if (!wam)
      throw WamException(CANT_OPEN_WAM, "locked wam or invalid config file");
      
   wam_local = bt_wam_get_local(wam);
   
   return;
}

void Wam::setGravityCompensation(int onoff)
{
   bt_wam_setgcomp(wam, onoff);
   return;
}

int Wam::isGravityCompensation()
{
   return bt_wam_isgcomp(wam);
}

int Wam::dof()
{
   return bt_wam_dof(wam);
}

void Wam::idle()
{
   bt_wam_idle(wam);   
   return;
}

void Wam::hold() 
{
   if (this->isHolding())
      this->idle();
   else
      bt_wam_hold(wam);
   return;
}

bool Wam::isHolding()
{
   return bt_wam_is_holding(wam);
}

char * Wam::conPosition()
{
   return bt_wam_str_con_position(wam, this->buf);
}

char * Wam::getCurrentControllerName()
{
   return bt_wam_get_current_controller_name(wam, this->buf);
}
char * Wam::getCurrentControllerSpace()
{
   return bt_wam_get_current_controller_space(wam, this->buf);
}
void Wam::controllerToggle()
{
   bt_wam_controller_toggle(wam);
   return;
}
void Wam::controlUseName(char * name)
{
   bt_wam_control_use_name(wam, name);
   return;
}
void Wam::controlUseSpace(char * space)
{
   bt_wam_control_use_space(wam, space);
   return;
}

void Wam::refgenClear()
{
   bt_wam_refgen_clear(wam);
   return;
}
char * Wam::refgenActiveName()
{
   return bt_wam_refgen_active_name(wam, this->buf);
}
char * Wam::refgenLoadedName()
{
   return bt_wam_refgen_loaded_name(wam, this->buf);
}
void Wam::refgenSave(char * filename)
{
   bt_wam_refgen_save(wam, filename);
}
void Wam::refgenLoad(char * filename)
{
   bt_wam_refgen_load(wam, filename);
}

bool Wam::isTeaching()
{
   return bt_wam_is_teaching(wam);
}

void Wam::teachStart() throw (WamException)
{
   int start = bt_wam_teach_start(wam);
   if (start == -1)
      throw WamException(TEACH_ERROR, "failed to start teach");
   return;
}

void Wam::teachEnd()
{
   bt_wam_teach_end(wam);
   return;
}

void Wam::playback()
{
   bt_wam_run(wam);
   return;
}

void Wam::setVelocity(double vel)
{
   bt_wam_set_velocity(wam, vel);
   return;
}

void Wam::setAcceleration(double acc)
{
   bt_wam_set_acceleration(wam, acc);
   return;
}

/*
void Wam::moveTo(int n, double * dest)
{
   bt_wam_moveto(wam, n, dest);
}
*/

void Wam::moveHome()
{
   bt_wam_movehome(wam);
   return;
}

bool Wam::moveIsDone()
{
   return bt_wam_moveisdone(wam);
}

char * Wam::getName()
{
   return name;
}

char * Wam::getJointPosition()
{
   return bt_wam_str_jposition(wam, this->buf);
}

char * Wam::getJointVelocity()
{
   return bt_wam_str_jvelocity(wam, this->buf);
}

char * Wam::getJointTorque()
{
   return bt_wam_str_jtorque(wam, this->buf);
}

char * Wam::getCartesianPosition() //??
{
   return bt_wam_str_cposition(wam, this->buf);
}

char * Wam::getCartesianRotationRow1() //??
{
   return bt_wam_str_crotation_r1(wam, this->buf);
}

char * Wam::getCartesianRotationRow2() //??
{
   return bt_wam_str_crotation_r2(wam, this->buf);
}

char * Wam::getCartesianRotationRow3() //??
{
   return bt_wam_str_crotation_r3(wam, this->buf);
}

double * Wam::getJointPositionDbl()
{
   return strToDbl(bt_wam_str_jposition(wam, this->buf), bt_wam_dof(wam) );
}

double * Wam::getJointVelocityDbl()
{
   return strToDbl(bt_wam_str_jvelocity(wam, this->buf), bt_wam_dof(wam) );
}

double * Wam::getJointTorqueDbl()
{
   return strToDbl(bt_wam_str_jtorque(wam, this->buf), bt_wam_dof(wam) );
}





