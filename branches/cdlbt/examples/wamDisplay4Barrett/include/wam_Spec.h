#ifndef WAM_SPEC_H
#define WAM_SPEC_H

// Definitions: ///////

// Choose tool:
// #define USE_DISK_TOOL
#define USE_NULL1_TOOL

// Define if you don't want use tool lenght in kinematics:
// #define USE_NO_KIN_TOOL_HEIGHT

///////////////////////

#ifndef USE_DISK_TOOL
#ifndef USE_NULL1_TOOL
#error NO TOOL DEFINED
#endif
#endif

//this scale factor is used only in the 5 and 6 joints in the Iyy inertia tensor value
#define INERTIA_SCALE_FACTOR 1 //for default inertia parameters
// #define INERTIA_SCALE_FACTOR 4 //correct the standard inertia parameters




#define PI   3.14159265358979323846264338327950288419716939937510582097494




// End Effector to Force Sensor Adapter
#define end2fMass          36e-3
#define end2fsHeight       5.5e-3
// #define end2fComX          0.0
// #define end2fComY          0.0
#define end2fComZ          (5.5e-3/2)

// JR3 Force Sensor Parameters
#define forceSensorMass    (179e-3+25e-3)
#define forceSensorHeight  (25e-3+5e-3)
// #define forceSensorComX    0.0
// #define forceSensorComY    0.0
#define forceSensorComZ    ((179e-3*(12.5e-3+5e-3)+25e-3*5e-3/2)/forceSensorMass)





/*------------------------------------------------------------------------*/
/* Tool's                                                                   */
/*------------------------------------------------------------------------*/


// small tool
#define smallToolMass      164e-3
#define smallToolHeight    102e-3
// #define smallToolComX      0.0
// #define smallToolComY      0.0
#define smallToolComZ      29.69e-3

// null space tool (revolution)
#define null1ToolMass      74e-3
#define null1ToolHeight    282e-3
// #define null1ToolComX      0.0
// #define null1ToolComY      0.0
#define null1ToolComZ      61e-3

// Tool in use
#ifdef USE_DISK_TOOL
	#define toolMass     end2fMass
	#define toolHeight    end2fsHeight
	#define toolComX      0.0
	#define toolComY      0.0
	#define toolComZ      end2fComZ
#else
#ifdef USE_NULL1_TOOL
	#define toolMass      null1ToolMass
	#define toolHeight    null1ToolHeight
	#define toolComX      0.0
	#define toolComY      0.0
	#define toolComZ      null1ToolComZ
#endif
#endif












//Total end effector parameters (end2fs+orceSensor+smallTool/bigTool)
#define total_endMass      (end2fMass+forceSensorMass+toolMass)
#define total_endHeight    (end2fsHeight+forceSensorHeight+toolHeight)
// #define total_endComX      0.0
// #define total_endComY      0.0
#define total_endComZ      ((end2fMass*end2fComZ+forceSensorMass*(forceSensorComZ+end2fsHeight) +  toolMass*(toolComZ+end2fsHeight+forceSensorHeight))/total_endMass)












// WAM Arm transmission Ratios
// (new values from WAM_MassParams_AA-00.pdf , old values commented)
#define WAMTR_N1 42.0013    // 42.0
#define WAMTR_N2 28.2510    // 28.25
#define WAMTR_N3 28.2510    // 28.25
#define WAMTR_n3 1.6800     // 1.68
#define WAMTR_N4 18.0016    // 18.0
#define WAMTR_N5 9.6973     // 10.27
#define WAMTR_N6 9.6973     // 10.27
#define WAMTR_N7 14.9333    // 14.93
#define WAMTR_n6 1.0000     // 1.0



/*------------------------------------------------------------------------*/
/* RNE: boundary conditions at the base link: acceleration.               */
/* Gravity vector characterization (module and unit vector).              */
/*              vd0   - gconst * [ gunitvx gunitvy gunitvz }#'            */
/* Gravity characterization also used in the Lagrangian formulation.      */
/*------------------------------------------------------------------------*/
#define gconst   9.80665
#define gunitvx   0.0
#define gunitvy   0.0
#define gunitvz   (-1.0)
#define GVECX   (gconst*gunitvx)
#define GVECY   (gconst*gunitvy)
#define GVECZ   (gconst*gunitvz)
#define gvecx   gvec[0] //GVECX
#define gvecy   gvec[1] //GVECY
#define gvecz   gvec[2] //GVECZ
#define vd0   gconst
#define vd0x   (-gvecx)
#define vd0y   (-gvecy)
#define vd0z   (-gvecz)
#define v0x   0.0
#define v0y   0.0
#define v0z   0.0

/*------------------------------------------------------------------------*/
/* RNE: boundary conditions at the base link.                             */
/*      Angular velocity and acceleration: w0, wd0.                       */
/*------------------------------------------------------------------------*/
#define w0x    0.0
#define w0y    0.0
#define w0z    0.0
#define wd0x   0.0
#define wd0y   0.0
#define wd0z   0.0

/*------------------------------------------------------------------------*/
/* RNE: boundary conditions at the tip. Ftip (ftip,ntip) denote the       */
/*      external end-of-arm (tip) force and moment vector(s) acting on    */
/*      the tool tip due to a load or contact with the environment.       */
/*      As in: M(q)*ddq+C(q,dq)*dq+h(q) tau*J'(q)*Ftip                    */
/*      RNE backward rec. initialization: f^{n+1} -ftip, n^{n+1} -ntip.   */
/*------------------------------------------------------------------------*/
#define ftipx   0.0
#define ftipy   0.0
#define ftipz   0.0
#define ntipx   0.0
#define ntipy   0.0
#define ntipz   0.0



/*------------------------------------------------------------------------*/
/*                                                                        */
/* Frictions specification                                                */
/*                                                                        */
/*------------------------------------------------------------------------*/
/*  FricEpsilon is a friction model parameter (have no phisics meaning)   */
/*------------------------------------------------------------------------*/
// #define FricviscousExample 0.4
// #define FricdynamicExample 1.0
// #define FricstaticExample 1.5
// #define  FricEpsilonExampleA 0.1
// #define  FricEpsilonExampleB 0.4

// #define Fricviscous4DOF 0.4
// #define Fricdynamic4DOF 1.0
// #define Fricstatic4DOF 1.5
// #define  FricEpsilon4DOF 0.1
// // 
// #define FricviscousWrist 0.4
// #define FricdynamicWrist 1.0
// #define FricstaticWrist 1.5
// #define FricEpsilonWrist 0.4


#define Fricviscous4DOF 0.4/4.0
#define Fricdynamic4DOF 1.0/4.0
#define Fricstatic4DOF 1.5/4.0
#define  FricEpsilon4DOF 0.1/4.0

#define FricviscousWrist 0.4/8.0
#define FricdynamicWrist 1.0/8.0
#define FricstaticWrist 1.5/8.0
#define FricEpsilonWrist 0.4/8.0

// #define Fricviscous4DOF 0.001
// #define Fricdynamic4DOF 0.001
// #define Fricstatic4DOF 0.001
// #define  FricEpsilon4DOF 0.001

// #define FricviscousWrist 0.001
// #define FricdynamicWrist 0.001
// #define FricstaticWrist 0.001
// #define FricEpsilonWrist 0.001



// Joint 1
#define Fricviscous1 Fricviscous4DOF
#define Fricdynamic1 Fricdynamic4DOF
#define Fricstatic1 Fricstatic4DOF
#define FricEpsilon1 FricEpsilon4DOF
// Joint 2
#define Fricviscous2 Fricviscous4DOF
#define Fricdynamic2 Fricdynamic4DOF
#define Fricstatic2 Fricstatic4DOF
#define FricEpsilon2 FricEpsilon4DOF
// Joint 3
#define Fricviscous3 Fricviscous4DOF
#define Fricdynamic3 Fricdynamic4DOF
#define Fricstatic3 Fricstatic4DOF
#define FricEpsilon3 FricEpsilon4DOF
// Joint 4
#define Fricviscous4 Fricviscous4DOF
#define Fricdynamic4 Fricdynamic4DOF
#define Fricstatic4 Fricstatic4DOF
#define FricEpsilon4 FricEpsilon4DOF
// Joint 5
#define Fricviscous5 FricviscousWrist
#define Fricdynamic5 FricdynamicWrist
#define Fricstatic5 FricstaticWrist
#define FricEpsilon5 FricEpsilonWrist
// Joint 6
#define Fricviscous6 FricviscousWrist
#define Fricdynamic6 FricdynamicWrist
#define Fricstatic6 FricstaticWrist
#define FricEpsilon6 FricEpsilonWrist
// Joint 7
#define Fricviscous7 FricviscousWrist
#define Fricdynamic7 FricdynamicWrist
#define Fricstatic7 FricstaticWrist
#define FricEpsilon7 FricEpsilonWrist



// Joints restrictions parameters for simulation (fictional parameters)
#define KQ 10000 // spring constant
#define KQDA 10  // dump constant (bigger)
#define KQDB 1   // dump constant (smaller)







/*------------------------------------------------------------------------*/
/*                                                                        */
/*                       Kinematics specification                         */
/*                                                                        */
/*------------------------------------------------------------------------*/

// Defines the first four joint  Kinematic parameters

/*------------------------------------------------------------------------*/
/* Frame 1                                                                */
/*------------------------------------------------------------------------*/
#define sigma1 0
#define zeta1 (1-sigma1)
#define a1 0.0
#define alpha1 -PI/2.0
#define d1 0.0
#define theta1 0.0
/* theta1  */
/*---------------------------------------*/
#define theta1maxrad 2.6
#define theta1maxdeg 150.0
#define theta1minrad -2.6
#define theta1mindeg -150.0



/*------------------------------------------------------------------------*/
/* Frame 2                                                                */
/*------------------------------------------------------------------------*/
#define sigma2 0
#define zeta2 (1-sigma2)
#define a2 0.0
#define alpha2 PI/2.0
#define d2 0.0
#define theta2 0.0
/* theta2  */
/*---------------------------------------*/
#define theta2maxrad 2.0
#define theta2maxdeg 113.0
#define theta2minrad -2.0
#define theta2mindeg -113.0



/*------------------------------------------------------------------------*/
/* Frame 3                                                                */
/*------------------------------------------------------------------------*/
#define sigma3 0
#define zeta3 (1-sigma3)
#define a3 0.045
#define alpha3 -PI/2.0
#define d3 0.55
#define theta3 0.0
/* theta3  */
/*---------------------------------------*/
#define theta3maxrad 2.8
#define theta3maxdeg 157.0
#define theta3minrad -2.8
#define theta3mindeg -157.0



/*------------------------------------------------------------------------*/
/* Frame 4                                                                */
/*------------------------------------------------------------------------*/
#define sigma4 0
#define zeta4 (1-sigma4)
#define a4 -0.045
#define alpha4 PI/2.0
#define d4 0.0
#define theta4 0.0
/* theta4  */
/*---------------------------------------*/
#define theta4maxrad 3.14
#define theta4maxdeg 180.0
#define theta4minrad -0.9
#define theta4mindeg -50.0


// Defines the last three joint  Kinematic parameters for 7DOF
#if DOF == 7
/*------------------------------------------------------------------------*/
/* Frame 5                                                                */
/*------------------------------------------------------------------------*/
#define sigma5 0
#define zeta5 (1-sigma5)
#define a5 0.0
#define alpha5 -PI/2.0
#define d5 0.3
#define theta5 -PI/2.0
/* theta5  */
/*---------------------------------------*/
#define theta5maxrad 3.0
#define theta5maxdeg 175.0
#define theta5minrad -3.0
#define theta5mindeg -175.0
/*------------------------------------------------------------------------*/
/* Frame 6                                                                */
/*------------------------------------------------------------------------*/
#define sigma6 0
#define zeta6 (1-sigma6)
#define a6 0.0
#define alpha6 PI/2.0
#define d6 0.0
#define theta6 0.0
/* theta6  */
/*---------------------------------------*/
#define theta6maxrad 1.6
#define theta6maxdeg 90.0
#define theta6minrad -1.6
#define theta6mindeg -90.0
/*------------------------------------------------------------------------*/
/* Frame 7                                                                */
/*------------------------------------------------------------------------*/
#define sigma7 0
#define zeta7 (1-sigma7)
#define a7 0.0
#define alpha7 0.0
#define d7 (0.06)          //+total_endHeight) //ATENTION!!!  //OK
#define theta7 0.0
/* theta7  */
/*---------------------------------------*/
#define theta7maxrad 2.2
#define theta7maxdeg 128.0
#define theta7minrad -2.2
#define theta7mindeg -128.0
#endif
/*------------------------------------------------------------------------*/
/* Tool                                                                */
/*------------------------------------------------------------------------*/
#define sigmaTool 0
#define zetaTool (1-sigmaTool)
#define aTool 0.0          //18.8e-2            //ATENTION //OK
#define alphaTool 0.0
#ifndef USE_NO_KIN_TOOL_HEIGHT
	#define dTool (total_endHeight) // +18e-2) //ATENTION      //OK
#else
	#define dTool (end2fsHeight) // +18e-2) //ATENTION      //OK
#endif
#define thetaTool 0.0










/*------------------------------------------------------------------------*/
/*                                                                        */
/*                       Dynamics specification                           */
/*                     (WAM_MassParams_AA-00.pdf)                         */
/*------------------------------------------------------------------------*/


/*------------------------------------------------------------------------*/
/* Link 1 Relative to Frame 1 ("Base")                                    */
/*------------------------------------------------------------------------*/
/* Center of Mass (relative to frame 1) */
#define mass1 10.76768767
#define comx1 -0.00443422
#define comy1 0.12189039
#define comz1 -0.00066489
/* Moments of Inertia (at center of mass, aligned with frame 1) */
/* Notation: here Ixx (etc) are exchanged with Lxx (etc) with respect */
/*       to the [WAM Arm Inertial Specifications manual. */
#define Ixx1  0.13488033
#define Ixy1  -0.00213041
#define Ixz1  -0.00012485
#define Iyx1  -0.00213041
#ifdef NOTUSE_MOTOR_INERTIAS
#define Iyy1 0.11328369
#endif
#ifndef NOTUSE_MOTOR_INERTIAS
#define Iyy1 (0.11328369+0.20518962)
#endif
#define Iyz1  0.00068555
#define Izx1  -0.00012485
#define Izy1  0.00068555
#define Izz1  0.09046330



/*------------------------------------------------------------------------*/
/* Link 2 Relative to Frame 2 ("Pitch")                                   */
/*------------------------------------------------------------------------*/
/* Center of Mass (relative to frame 2) */
#define mass2 3.87493756
#define comx2 -0.00236983
#define comy2 0.03105614
#define comz2 0.01542114
/* Moments of Inertia (at center of mass, aligned with frame 2) */
/* Notation: here Ixx (etc) are exchanged with Lxx (etc) with respect */
/*       to the [WAM Arm Inertial Specifications manual. */
#define Ixx2  0.02140958
#define Ixy2  0.00027172
#define Ixz2  0.00002461
#define Iyx2  0.00027172
#ifdef NOTUSE_MOTOR_INERTIAS
#define Iyy2 0.01377875
#endif
#ifndef NOTUSE_MOTOR_INERTIAS
#define Iyy2 (0.01377875+0.18885672)
#endif
#define Iyz2  -0.00181920
#define Izx2  0.00002461
#define Izy2  -0.00181920
#define Izz2  0.01558906


/*------------------------------------------------------------------------*/
/* Link 3 Relative to Frame 3  ("Twist")                                  */
/*------------------------------------------------------------------------*/
/* Center of Mass (relative to frame 3) */
#define mass3 1.80228141
#define comx3 -0.03825858
#define comy3 0.20750770
#define comz3 0.00003309
/* Moments of Inertia (at center of mass, aligned with frame 3) */
/* Notation: here Ixx (etc) are exchanged with Lxx (etc) with respect */
#define Ixx3  0.05911077
#define Ixy3  -0.00249612
#define Ixz3  0.00000738
#define Iyx3  -0.00249612
#ifdef NOTUSE_MOTOR_INERTIAS
#define Iyy3 0.00324550
#endif
#ifndef NOTUSE_MOTOR_INERTIAS
#define Iyy3 (0.00324550+0.18885672)
#endif
#define Iyz3  -0.00001767
#define Izx3  0.00000738
#define Izy3  -0.00001767
#define Izz3  0.05927043



/*________________________________________________________________________ */
/*|                                FOR 4 DOF                              |*/
/*|_______________________________________________________________________|*/
#if DOF == 4
/*------------------------------------------------------------------------*/
/* Link 4 and Outer Link Relative to Frame 4 ("Elbow")                    */
/*------------------------------------------------------------------------*/
/* Center of Mass (relative to frame 4) */
#define mass4 1.06513649
#define comx4 0.01095471
#define comy4 -0.00002567
#define comz4 0.14053900
/* Moments of Inertia (at center of mass, aligned with frame 4) */
/* Notation: here Ixx (etc) are exchanged with Lxx (etc) with respect */
/*       to the [WAM Arm Inertial Specifications manual. */
#define Ixx4  0.01848577
#define Ixy4  0.00000219
#define Ixz4  -0.00160868
#define Iyx4  0.00000219
#ifdef NOTUSE_MOTOR_INERTIAS
#define Iyy4 0.01891658
#endif
#ifndef NOTUSE_MOTOR_INERTIAS
#define Iyy4 (0.01891658+0.03462804)
#endif
#define Iyz4  0.00000515
#define Izx4  -0.00160868
#define Izy4  0.00000515
#define Izz4  0.00197517
#endif


/*________________________________________________________________________ */
/*|                                FOR 7 DOF                              |*/
/*|_______________________________________________________________________|*/
#if DOF == 7
/*---------------------------------------------------------------------------------*/
/* Link 4 and Fixed Wrist Component Relative to Frame 4   ("Elbow + Wrist body")   */
/*---------------------------------------------------------------------------------*/
#define mass4 (2.40016804)
#define comx4 0.00498512
#define comy4 -0.00022942
#define comz4 (0.13271662+0.02)
/* Moments of Inertia (at center of mass, aligned with frame 4) */
/* Notation: here Ixx (etc) are exchanged with Lxx (etc) with respect */
/*       to the [WAM Arm Inertial Specifications manual. */
#define Ixx4  0.01491672
#define Ixy4  0.00001741
#define Ixz4  -0.00150604
#define Iyx4  0.00001741
#ifdef NOTUSE_MOTOR_INERTIAS
#define Iyy4 0.01482922
#endif
#ifndef NOTUSE_MOTOR_INERTIAS
#define Iyy4 (0.01482922+0.03462804)
#endif
#define Iyz4  -0.00002109
#define Izx4  -0.00150604
#define Izy4  -0.00002109
#define Izz4  0.00294463


/*------------------------------------------------------------------------*/
/* Link 5 Relative to Frame 5 ("Wrist Yaw")                               */
/*------------------------------------------------------------------------*/
// correction factor introduced to increase innertia from the engine
// this will compensate the frictions forces that can't be modeled in the engine
#define INERTIA_FACTOR5 INERTIA_SCALE_FACTOR

/* Center of Mass (relative to frame 5) */
#define mass5 0.12376019
#define comx5 0.00008921
#define comy5 0.00511217
#define comz5 0.00435824
/* Moments of Inertia (at center of mass, aligned with frame 5) */
/* Notation: here Ixx (etc) are exchanged with Lxx (etc) with respect */
/*       to the [WAM Arm Inertial Specifications manual. */
#define Ixx5  0.00005029
#define Ixy5  0.00000020
#define Ixz5  -0.00000005
#define Iyx5  0.00000020
#ifdef NOTUSE_MOTOR_INERTIAS
#define Iyy5 0.00007582
#endif
#ifndef NOTUSE_MOTOR_INERTIAS
#define Iyy5 (0.00007582+0.00322537)*INERTIA_FACTOR5
#endif
#define Iyz5  -0.00000359
#define Izx5  -0.00000005
#define Izy5  -0.00000359
#define Izz5  0.00006270

/*------------------------------------------------------------------------*/
/* Link 6 Relative to Frame 6 ("Wrist Pitch")                             */
/*------------------------------------------------------------------------*/
#define INERTIA_FACTOR6 INERTIA_SCALE_FACTOR

/* Center of Mass (relative to frame 6) */
#define mass6 0.41797364
#define comx6 -0.00012262
#define comy6 -0.01703194
#define comz6 0.02468336
/* Moments of Inertia (at center of mass, aligned with frame 6) */
/* Notation: here Ixx (etc) are exchanged with Lxx (etc) with respect */
/*       to the [WAM Arm Inertial Specifications manual. */
#define Ixx6  0.00055516
#define Ixy6  0.00000061
#define Ixz6  -0.00000074
#define Iyx6  0.00000061
#ifdef NOTUSE_MOTOR_INERTIAS
#define Iyy6 0.00024367
#endif
#ifndef NOTUSE_MOTOR_INERTIAS
#define Iyy6 (0.00024367+0.00322537)*INERTIA_FACTOR6
#endif
#define Iyz6  -0.00004590
#define Izx6  -0.00000074
#define Izy6  -0.00004590
#define Izz6  0.00045358



/*------------------------------------------------------------------------*/
/* Link 7 Relative to Frame 7 ("Wrist Roll")                              */
/*------------------------------------------------------------------------*/
/* Center of Mass (relative to frame 7) */



#define mass7 (0.06864753+total_endMass)
#define comx7 (-0.00007974)
#define comy7 (0.00016313)
#define comz7 (   (-0.00323552*0.06864753+total_endMass*total_endComZ)/mass7)    // -   total_endHeight) //ATENTION!!! //OK
/* Moments of Inertia (at center of mass, aligned with frame 7) */
/* Notation: here Ixx (etc) are exchanged with Lxx (etc) with respect */
/*       to the [WAM Arm Inertial Specifications manual. */
#define Ixx7  0.00003773
#define Ixy7  -0.00000019
#define Ixz7  0.00000000
#define Iyx7  -0.00000019
#define Iyy7  0.00003806
#define Iyz7  0.00000000
#define Izx7  0.00000000
#define Izy7  0.00000000
#ifdef NOTUSE_MOTOR_INERTIAS
#define Izz7 0.00007408
#endif
#ifndef NOTUSE_MOTOR_INERTIAS
#define Izz7 (0.00007408+0.00031753)
#endif



#endif //END of #if DOF == 7







#endif
