/*  File    : MCM_Rev_Park.c
 *  Abstract:
 *
 *      FOC code Reverse Park transform  
 *
 *      Calculate Valpha & Vbeta at a sample rate Ts
 *      Valpha =  Vqs*cos(theta) + Vds*sin(theta)
 *      Vbeta  = -Vqs*sin(theta) + Vds*cos(theta)   
 *   
 *      ***Need to add circle limit *** 11/24/22
 *
 *      Discrete time, no states, direct feedthrough 
 *
 *   Brian Tremaine Nov 16, 2020
 */

#define S_FUNCTION_NAME MCM_Rev_Park
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include <math.h>
#include "circle_limitation.h"

#define U(element) (*uPtrs[element])  /* Pointer to Input Port0 */
#define TRUE 1
 
/*====================*
 * S-function methods *
 *====================*/

/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *    The sizes information is used by Simulink to determine the S-function
 *    block's characteristics (number of inputs, outputs, states, etc.).
 */
static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 0);  /* Number of expected parameters = 0 */
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return; /* Parameter mismatch will be reported by Simulink */
    }

    ssSetNumContStates(S, 0);  // no states
    ssSetNumDiscStates(S, 0);  // no states

    if (!ssSetNumInputPorts(S, 1)) return;
    ssSetInputPortWidth(S, 0, 3);
    ssSetInputPortDirectFeedThrough(S, 0, TRUE); 

    if (!ssSetNumOutputPorts(S, 1)) return;
    ssSetOutputPortWidth(S, 0, 2);

    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);
    ssSetOperatingPointCompliance(S, USE_DEFAULT_OPERATING_POINT);

    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);
}

/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    Specify the sample time as Ts (inherited)
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
    ssSetModelReferenceSampleTimeDefaultInheritance(S);            
}

// #define MDL_INITIALIZE_CONDITIONS
/* Function: mdlInitializeConditions ========================================
 * Abstract:
 *    not required
 */
// static void mdlInitializeConditions(SimStruct *S)\
//{
//}


/* Function: mdlOutputs =======================================================
 * Abstract:
 *      y = f(u) 
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    real_T            *y    = ssGetOutputPortRealSignal(S,0);
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0);
 
    UNUSED_ARG(tid); /* not used in single tasking mode */
    
    real_T      Vqs =  U(0);
    real_T      Vds =  U(1);
    real_T      theta= U(2); 
    real_T      Vq;
    real_T      Vd;
    real_T      Vmag_sqr;
    real_T      Vmag;
    
    #define MMI 32000
    #define S16_MAX 32767
    
    qd_t Vqd;

    Vqd.q= Vqs;
    Vqd.d= Vds;

    /* apply Circle_Limitation on Vqs,Vds
       includes modulation index
    */
    Vqd = Circle_Limitation( &CircleLimitationM1, Vqd );
    
    /* Reverse Park transform */
    y[0]=  Vqd.q*cos(theta) + Vqd.d*sin(theta); /* Valpha */ 
    y[1]= -Vqd.q*sin(theta) + Vqd.d*cos(theta); /* Vbeta */
}


//#define MDL_UPDATE
/* Function: mdlUpdate ======================================================
 * Abstract:
 *      not required, no states
 *      xdot = Ax + Bu
 */
//static void mdlUpdate(SimStruct *S, int_T tid)
//{
//}


/* Function: mdlTerminate =====================================================
 * Abstract:
 *    No termination needed, but we are required to have this routine.
 */
static void mdlTerminate(SimStruct *S)
{
    UNUSED_ARG(S); /* unused input argument */
}

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
