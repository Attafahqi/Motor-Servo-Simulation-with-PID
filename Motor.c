#define S_FUNCTION_LEVEL 2 
#define S_FUNCTION_NAME Motor
#include "simstruc.h" 
#include <math.h> 

#define U(element) (*uPtrs[element]) /*Pointer to Input Port0*/ 

static void mdlInitializeSizes(SimStruct *S){ 
    ssSetNumContStates(S, 3); 
    if (!ssSetNumInputPorts(S, 1)) return; 
    ssSetInputPortWidth(S, 0, 2); 
    ssSetInputPortDirectFeedThrough(S, 0, 1); 
    ssSetInputPortOverWritable(S, 0, 1); 
    if (!ssSetNumOutputPorts(S, 1)) return; 
    ssSetOutputPortWidth(S, 0, 3); 
    ssSetNumSampleTimes(S, 1); 

    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE); } 

static void mdlInitializeSampleTimes(SimStruct *S) { 
    ssSetSampleTime(S, 0, CONTINUOUS_SAMPLE_TIME); 
    ssSetOffsetTime(S, 0, 0.0); } 

#define MDL_INITIALIZE_CONDITIONS 
static void mdlInitializeConditions(SimStruct *S) { 

    real_T *X0 = ssGetContStates(S); 
    int_T nStates = ssGetNumContStates(S); 
    int_T i; 

    /* initialize the states to 0.0 */ 
    for (i=0; i < nStates; i++) {X0[i] = 0.0;} } 

static void mdlOutputs(SimStruct *S, int_T tid) { 
    real_T *Y = ssGetOutputPortRealSignal(S,0); 
    real_T *X = ssGetContStates(S); 
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0); 

    Y[0] = X[2];
    Y[1] = X[1];
    Y[2] = X[0];

} 

#define MDL_DERIVATIVES 
static void mdlDerivatives(SimStruct *S) { 
    real_T *dX = ssGetdX(S); 
    real_T *X = ssGetContStates(S); 
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0); 

    real_T V_in, T_load, I, W, Theta, I_dot, W_dot, Theta_dot;
    real_T R, L, J, Kt, Ke, b;

    R = 0.11;
    L = 0.2e-3;
    J = 1.2e-5;
    b = 0.0001 ;
    Kt = 0.97;
    Ke = 62.6;

    T_load = U(0);
    V_in = U(1);
    I = X[0];
    W = X[1];
    Theta = X[2];


    I_dot = -(R / L) * I - (Ke / L) * W + (1 / L) * V_in;
    W_dot = (Kt / J) * I - (b / J) * W - (1 / J) * T_load;
    Theta_dot = W;

    dX[0] = I_dot;
    dX[1] = W_dot;
    dX[2] = Theta_dot;
 
} 

static void mdlTerminate(SimStruct *S) 
{} /*Keep this function empty since no memory is allocated*/ 

#ifdef MATLAB_MEX_FILE 
/* Is this file being compiled as a MEX-file? */ 
#include "simulink.c" /* MEX-file interface mechanism */ 
#else 
#include "cg_sfun.h" /*Code generation registration function*/ 
#endif 