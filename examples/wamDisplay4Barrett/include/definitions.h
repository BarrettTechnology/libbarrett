#ifndef MR_DEFINITIONS_H
#define MR_DEFINITIONS_H

//----my current definitions----//
#define DOF 7
#define USING_DOUBLE
// #define NOTUSE_MOTOR_INERTIAS
// #define USE_ELD_MASSMATRIX_COMPUTATION
// #define USE_SYS_GET_TIME
#define USE_RT_GET_TIME
//------------------------------//

// default DOF set to 7
#ifndef DOF
#warning using DOF = 7 by default
#define DOF 7
#endif


// default precision type set to double
#ifndef USING_FLOAT
	#ifndef USING_DOUBLE
		#define USING_DOUBLE
		#warning using double by default
	#endif
#endif

#ifdef USING_FLOAT                      // precision type to float
typedef float Real;
#endif

#ifdef USING_DOUBLE                     // precision type to double
typedef double Real;
#endif


#endif
