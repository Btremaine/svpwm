/*  File    : svpwm.c
 *  Abstract:
 *
 *  Simulate SVMPWM for 3-phase PMSM motor for FOC
 *
 *  parameters: Vbus, Ts (2 parameters set before run-time)
 *  inputs:     Valpha, Vbeta SCALED to signed 14-bit
 *              discrete time @ PWM rate pulse train
 *  Outputs:    U, V, W and angle ramp and sector.
 *              (U,V & W) are voltage levels of Vbus or 0 (gnd)
 *  states: 1, continuous.
 *  Operate at 'fast' sample rate Tfast (pwm rate) + continuous
 *  direct feed-through
 *
 *  Given input samples (Valpha, Vbeta), compute angle and decompose
 *  these into three components, U, V and W
 *  Generate a continuous ramp between max and min with equal
 *  magnitude (+/-) slopes and period Tfast.
 *  Use the ramp as a comparator reference to convert the decomposed
 *  components into short bursts on U, V and W outputs.
 *  Uses center-aligned pwm.
 *
 *  ref:
 *    https://www.switchcraft.org/learning/2017/3/15/space-vector-pwm-intro
 *    Part 1: "https://www.youtube.com/watch?v=vJuaTbwjfMo&t=0s"
 *    Part 2: "https://www.youtube.com/watch?v=oq868piQ9Q4"
 *
 *   Brian Tremaine Nov 26, 2022
 */

/* specify S-function name consistent with block name */
#define S_FUNCTION_NAME  svpwm
#define S_FUNCTION_LEVEL 2

#include <math.h>
#include <string.h>
#include <stdio.h>#define Ui0(element) (*uPtrs0[element])      /* Pointer to Input Port0 */
#include <stdint.h>
#include "simstruc.h"
#include "matrix.h"

#define Ui0(element) (*uPtrs0[element])    /* Pointer to Input Port0 */
#define Ui1(element) (*uPtrs1[element])    /* Pointer to Input Port1 */
#define Vbus_PARAM(S) ssGetSFcnParam(S,0)  /* define Vbus */
#define Ts_PARAM(S) ssGetSFcnParam(S,1)    /* define Ts   */

#define NUM_CSTATES 1  // continuous states
#define NUM_DSTATES 0  // discrete states
#define NPARAMS 3      // input parameters
#define TRUE 1
#define PI M_PI

/* Error handling
 * --------------
 *
 * You should use the following technique to report errors encountered within
 * an S-function:
 *
 *       ssSetErrorStatus(S,"Error encountered due to ...");
 *       return;
 *
 * Note that the 2nd argument to ssSetErrorStatus must be persistent memory.
 * It cannot be a local variable. For example the following will cause
 * unpredictable errors:
 *
 *      mdlOutputs()
 *      {
 *         char msg[256];         {ILLEGAL: to fix use "static char msg[256];"}
 *         sprintf(msg,"Error due to %s", string);
 *         ssSetErrorStatus(S,msg);
 *         return;
 *      }
 */

#define IS_PARAM_DOUBLE(pVal) (mxIsNumeric(pVal) && !mxIsLogical(pVal) &&\
!mxIsEmpty(pVal) && !mxIsSparse(pVal) && !mxIsComplex(pVal) && mxIsDouble(pVal))

#define OK_EMPTY_DOUBLE_PARAM(pVal) (mxIsNumeric(pVal) && !mxIsLogical(pVal) &&\
!mxIsSparse(pVal) && !mxIsComplex(pVal) && mxIsDouble(pVal))

/*====================*
 * S-function methods *
 *====================*/

#define MDL_CHECK_PARAMETERS
#if defined(MDL_CHECK_PARAMETERS) && defined(MATLAB_MEX_FILE)

  /* Function: mdlCheckParameters =============================================
   * Abstract:
   *    Validate our parameters to verify they are okay.
   */
  static void mdlCheckParameters(SimStruct *S)
  {
      /* Check 1st parameter: Vbus */
      {
          if ( (mxGetN(Vbus_PARAM(S)) != 1) || !IS_PARAM_DOUBLE(Vbus_PARAM(S)) ) {
              ssSetErrorStatus(S,"1st parameter to S-function, Vbus, is in error ");
              return;
          }
      }
      /* Check 2nd parameter: Ts */
      {
          if ( (mxGetN(Ts_PARAM(S)) != 1) || !IS_PARAM_DOUBLE(Ts_PARAM(S)) ) {
              ssSetErrorStatus(S,"2nd parameter to S-function, Ts, is in error ");
              return;
          }
      }
  }
#endif /* MDL_CHECK_PARAMETERS */

/* Function: mdlInitializeSizes ===========================================
 * Abstract:  REQUIRED
 *    The sizes information is used by Simulink to determine the S-function
 *    block's characteristics (number of inputs, outputs, states, etc.).
 */
static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, NPARAMS);  /* Number of expected parameters */
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        /* Return if number of expected != number of actual parameters */
        return;
    }

    ssSetNumContStates(S, NUM_CSTATES); // ramp
    ssSetNumDiscStates(S, NUM_DSTATES); // none

    if (!ssSetNumInputPorts(S, 2)) return;
    ssSetInputPortWidth(S, 0, 2);
    ssSetInputPortWidth(S, 1, 1);
    ssSetInputPortDirectFeedThrough(S, 0, TRUE);
    ssSetInputPortDirectFeedThrough(S, 1, TRUE);

    ssSetInputPortRequiredContiguous(S, 0, 0); // not required

    if (!ssSetNumOutputPorts(S, 1)) return;
    ssSetOutputPortWidth(S, 0, 9);

    // ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE); // why/how is this used

    ssSetNumSampleTimes(S, 2);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);

    /* Specify the operating point save/restore compliance to be same as a
     * built-in block */
    ssSetOperatingPointCompliance(S, USE_DEFAULT_OPERATING_POINT);
    // ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);  // redundant
}

/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:  REQUIRED
 *    This function is used to specify the sample time(s) for your
 *    S-function. You must register the same number of sample times as
 *    specified in ssSetNumSampleTimes.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, 50E-6);
    ssSetSampleTime(S, 1, CONTINUOUS_SAMPLE_TIME);

    ssSetOffsetTime(S, 0, 0.0);
    ssSetOffsetTime(S, 1, 0.0);
    //ssSetModelReferenceSampleTimeDefaultInheritance(S); ** not needed ??
}

#define MDL_INITIALIZE_CONDITIONS   /* Change to #undef to remove function */
#if defined(MDL_INITIALIZE_CONDITIONS)
  /* Function: mdlInitializeConditions ========================================
   * Abstract:
   *    In this function, you should initialize the continuous and discrete
   *    states for your S-function block.  The initial states are placed
   *    in the state vector, ssGetContStates(S) or ssGetRealDiscStates(S).
   *    You can also perform any other initialization activities that your
   *    S-function may require. Note, this routine will be called at the
   *    start of simulation and if it is present in an enabled subsystem
   *    configured to reset states, it will be call when the enabled subsystem
   *    restarts execution to reset the states.
   */
  static void mdlInitializeConditions(SimStruct *S)
  {
     real_T *x0 = ssGetContStates(S);
     int16_t i;
     for (i=0; i< NUM_CSTATES; i++)
     {
        *x0++=0.0;   // initialize continuous-time ramp state
     }
  }

#endif /* MDL_INITIALIZE_CONDITIONS */

/* Function: mdlOutputs =======================================================
 * Abstract:  REQUIRED
 *    In this function, you compute the outputs of your S-function
 *    block.
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    real_T *x    = ssGetContStates(S);
    real_T *y    = ssGetOutputPortRealSignal(S,0);
    InputRealPtrsType uPtrs0 = ssGetInputPortRealSignalPtrs(S,0);
    InputRealPtrsType uPtrs1 = ssGetInputPortRealSignalPtrs(S,1);

    UNUSED_ARG(tid); /* not used in single tasking mode */

    real_T ramp = 4.0*x[0];             // scaled ramp
    real_T Va = Ui0(0) / (pow(2.0,14)); // Valpha
    real_T Vb = Ui0(1) / (pow(2.0,14)); // Vbeta
    real_T U;
    real_T V;
    real_T W;
    real_T angle;    // radians
    real_T deg;      // degrees
    int16_t sector;
    real_T ta;       // computed switch time T1 + T2 + T0/2
    real_T tb;       // computed switch time T1 + T0/2
    real_T tc;       // computed switch time T2 + T0/2
    real_T td;       // computed switch time T0/2
    real_T sine1;    // input to inverter half-bridge 1
    real_T sine2;    // input to inverter half-bridge 2
    real_T sine3;    // input to inverter half-bridge 3

    real_T T1;
    real_T T2;
    real_T Tz;
    real_T del1;
    real_T del2;
    real_T del3;
    real_T Mi;

    const real_T      *Vbus = mxGetPr(Vbus_PARAM(S)); // line voltage
    const real_T      *Ts   = mxGetPr(Ts_PARAM(S));   // pwm period

    // ref: Part 1: "https://www.youtube.com/watch?v=vJuaTbwjfMo&t=0s"
    //      Part 2: "https://www.youtube.com/watch?v=oq868piQ9Q4"

    // compute angle and modulation index
    angle = atan2(Vb, Va);    // radians  ? change to (A,B) ?
    deg = angle * 180.0/PI;   // degrees
    Mi = sqrt(Vb*Vb + Va*Va); //
    

    // compute sector number [1..6]
    if (deg>= 0 && deg <= 60) {
       sector = 1; }
    else if (deg > 60 && deg <= 120) {
       sector = 2; }
    else if (deg > 120 && deg <= 180) {
       sector = 3; }
    else if (deg < -120 && deg > -180) {
       sector = 4; }
    else if (deg < -60 && deg >= -120) {
       sector = 5; }
    else if (deg < 0  && deg >= -60) {
       sector = 6; }
    else {
        sector = 1;
    }

    real_T n = sector;

    // compute switching times here
    //
    del1 = (2.0/sqrt(3))*(Mi)*(cos(angle)*sin(n*PI/3.0) - sin(angle)*cos(n*PI/3.0));
    del2 = (2.0/sqrt(3))*(Mi)*(sin(angle)*cos((n-1.0)*PI/3.0) - cos(angle)*sin((n-1.0)*PI/3.0));
    del3 = 1.0 - fabs(del1)- fabs(del2);

    T1 = del1*(*Ts);
    T2 = del2*(*Ts);
    Tz = del3*(*Ts);

    //  !!! below needs derivation & checking
    //  U,V,W not switching high/low but instead ramping
    //  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    td = (Tz)/2.0;
    ta = T1 + T2 + td;
    tb = T1 + td;
    tc = T2 + td;

    // gate switch times to appropriate half-bridge:
    switch(sector) {
    case 1  :
      sine1 = ta;   // sequence U
      sine2 = tc;   //          V
      sine3 = td;   //          W
      break;
    case 2  :
      sine1 = tb;
      sine2 = ta;
      sine3 = td;
      break;
    case 3  :
      sine1 = td;
      sine2 = ta;
      sine3 = tc;
      break;
    case 4  :
      sine1 = td;
      sine2 = tb;
      sine3 = ta;
      break;
    case 5  :
      sine1 = tc;
      sine2 = td;
      sine3 = ta;
      break;
    case 6  :
      sine1 = ta;
      sine2 = td;
      sine3 = tb;
      break;
    /* catch errors here --- verify what to use */
    default :
      sine1 = ta;
      sine2 = tc;
      sine3 = td;
    }

    // operate the inverter ramp in code following,
    // and set output half bridges U, V and W
    if (sine1 > ramp){
        U = *Vbus ;
    } 
    else U = 0.0;
    if (sine2 > ramp){
        V = *Vbus ;
    }
    else V = 0.0;
    if (sine3 > ramp){
        W = *Vbus ;
    }
    else W = 0.0;

    // outputs here
    /* ============================================================== */
    y[0] = U;
    y[1] = V;
    y[2] = W;
    // debug variables:
    y[3] = angle;  // radians
    y[4] = sector; // (1:6)
    y[5] = ramp;
    y[6] = T1;
    y[7] = T2;
    y[8] = Tz;

}

#undef MDL_UPDATE  /* Change to #undef to remove function */
#if defined(MDL_UPDATE)
  /* Function: mdlUpdate ==================================================
   * Abstract:
   *    This function is called once for every major integration time step.
   *    Discrete states are typically updated here, but this function is
   *    useful for performing any tasks that should only take place once
   *    per integration step.
   */
  static void mdlUpdate(SimStruct *S, int_T tid)
  {
    real_T      *dx   = ssGetdX(S);
    real_T      *xC   = ssGetContStates(S);
    real_T      *xD   = ssGetDiscStates(S);

    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0);
  }
#endif /* MDL_UPDATE */

#define MDL_DERIVATIVES  /* Change to #undef to remove function */
#if defined(MDL_DERIVATIVES)
  /* Function: mdlDerivatives =============================================
   * Abstract:
   *    In this function, you compute the S-function block's derivatives.
   *    The derivatives are placed in the derivative vector, ssGetdX(S).
   */
  static void mdlDerivatives(SimStruct *S)
  {
    real_T            *dx    = ssGetdX(S);
    real_T            *x     = ssGetContStates(S);
    InputRealPtrsType uPtrs1 = ssGetInputPortRealSignalPtrs(S,1);
    // real_T            ramp   = x[0];   // doesn't do anything
    dx[0]= Ui1(0) - 0.500;    // mean is zero
  }
#endif /* MDL_DERIVATIVES */

/* Function: mdlTerminate =================================================
 * Abstract:  REQUIRED
 *    In this function, you should perform any actions that are necessary
 *    at the termination of a simulation.  For example, if memory was
 *    allocated in mdlStart, this is the place to free it.
 */
static void mdlTerminate(SimStruct *S)
{
}

/*=============================*
 * Required S-function trailer *
 *=============================*/

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
