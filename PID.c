#define S_FUNCTION_LEVEL 2 
#define S_FUNCTION_NAME PID
#include "simstruc.h" 
#include <math.h> 

#define U(element) (*uPtrs[element]) /*Pointer to Input Port0*/ 

static void mdlInitializeSizes(SimStruct *S){ 
    ssSetNumDiscStates(S, 3); 
    if (!ssSetNumInputPorts(S, 1)) return; 
    ssSetInputPortWidth(S, 0, 5); 
    ssSetInputPortDirectFeedThrough(S, 0, 1); 
    ssSetInputPortOverWritable(S, 0, 1); 
    if (!ssSetNumOutputPorts(S, 1)) return; 
    ssSetOutputPortWidth(S, 0, 1); 
    ssSetNumSampleTimes(S, 1); 

    ssSetOptions(S, (SS_OPTION_EXCEPTION_FREE_CODE 
    | SS_OPTION_DISCRETE_VALUED_OUTPUT));} 

static void mdlInitializeSampleTimes(SimStruct *S){ 
    ssSetSampleTime(S, 0, 1e-4); 
    ssSetOffsetTime(S, 0, 0.0);} 

#define MDL_INITIALIZE_CONDITIONS 
static void mdlInitializeConditions(SimStruct *S){ 
    real_T *X0 = ssGetRealDiscStates(S); 
    int_T nXStates = ssGetNumDiscStates(S); 
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0); 
    int_T i; 

/* initialize the states to 0.0 */ 
    for (i=0; i < nXStates; i++) { 
    X0[i] = 0.0; } } 

static void mdlOutputs(SimStruct *S, int_T tid){ 
    real_T *Y = ssGetOutputPortRealSignal(S,0); 
    real_T *X = ssGetRealDiscStates(S); 

    Y[0] = X[2];

} 

#define MDL_UPDATE 
static void mdlUpdate(SimStruct *S, int_T tid) { 
    real_T *X = ssGetRealDiscStates(S); 
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0); 

    real_T dt = 1e-5;
    real_T Kp, Ki, Kd, V_ref, V_actual, control;
    real_T integral, integral_old;
    real_T error, error_old;
    real_T derivative;

    Kp = U(0);
    Ki = U(1);
    Kd = U(2);
    V_ref = U(3);
    V_actual = U(4);

    error_old = X[0];
    integral_old = X[1];
    error = V_ref - V_actual;

    integral = integral_old + error *dt;
    derivative = (error - error_old) / dt;
    control = Kp * error + Ki * integral + Kd *derivative;

    X[0] = error;
    X[1] = integral;
    X[2] = control;

}

static void mdlTerminate(SimStruct *S) 
{ } /*Keep this function empty since no memory is allocated*/ 

#ifdef MATLAB_MEX_FILE 
/* Is this file being compiled as a MEX-file? */ 
#include "simulink.c" /*MEX-file interface mechanism*/ 
#else 
#include "cg_sfun.h" /*Code generation registration function*/ 
#endif 

