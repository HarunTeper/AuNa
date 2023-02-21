//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: CACC_Center.h
//
// Code generated for Simulink model 'CACC_Center'.
//
// Model version                  : 4.6
// Simulink Coder version         : 9.8 (R2022b) 13-May-2022
// C/C++ source code generated on : Tue Feb 21 12:18:59 2023
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Linux 64)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_CACC_Center_h_
#define RTW_HEADER_CACC_Center_h_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "slros2_initialize.h"
#include "CACC_Center_types.h"

extern "C"
{

#include "rt_nonfinite.h"

}

#include <string.h>
#include <stddef.h>

// Macros for accessing real-time model data structure
#ifndef rtmGetContStateDisabled
#define rtmGetContStateDisabled(rtm)   ((rtm)->contStateDisabled)
#endif

#ifndef rtmSetContStateDisabled
#define rtmSetContStateDisabled(rtm, val) ((rtm)->contStateDisabled = (val))
#endif

#ifndef rtmGetContStates
#define rtmGetContStates(rtm)          ((rtm)->contStates)
#endif

#ifndef rtmSetContStates
#define rtmSetContStates(rtm, val)     ((rtm)->contStates = (val))
#endif

#ifndef rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag
#define rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm) ((rtm)->CTOutputIncnstWithState)
#endif

#ifndef rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag
#define rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm, val) ((rtm)->CTOutputIncnstWithState = (val))
#endif

#ifndef rtmGetDerivCacheNeedsReset
#define rtmGetDerivCacheNeedsReset(rtm) ((rtm)->derivCacheNeedsReset)
#endif

#ifndef rtmSetDerivCacheNeedsReset
#define rtmSetDerivCacheNeedsReset(rtm, val) ((rtm)->derivCacheNeedsReset = (val))
#endif

#ifndef rtmGetIntgData
#define rtmGetIntgData(rtm)            ((rtm)->intgData)
#endif

#ifndef rtmSetIntgData
#define rtmSetIntgData(rtm, val)       ((rtm)->intgData = (val))
#endif

#ifndef rtmGetOdeF
#define rtmGetOdeF(rtm)                ((rtm)->odeF)
#endif

#ifndef rtmSetOdeF
#define rtmSetOdeF(rtm, val)           ((rtm)->odeF = (val))
#endif

#ifndef rtmGetOdeY
#define rtmGetOdeY(rtm)                ((rtm)->odeY)
#endif

#ifndef rtmSetOdeY
#define rtmSetOdeY(rtm, val)           ((rtm)->odeY = (val))
#endif

#ifndef rtmGetPeriodicContStateIndices
#define rtmGetPeriodicContStateIndices(rtm) ((rtm)->periodicContStateIndices)
#endif

#ifndef rtmSetPeriodicContStateIndices
#define rtmSetPeriodicContStateIndices(rtm, val) ((rtm)->periodicContStateIndices = (val))
#endif

#ifndef rtmGetPeriodicContStateRanges
#define rtmGetPeriodicContStateRanges(rtm) ((rtm)->periodicContStateRanges)
#endif

#ifndef rtmSetPeriodicContStateRanges
#define rtmSetPeriodicContStateRanges(rtm, val) ((rtm)->periodicContStateRanges = (val))
#endif

#ifndef rtmGetZCCacheNeedsReset
#define rtmGetZCCacheNeedsReset(rtm)   ((rtm)->zCCacheNeedsReset)
#endif

#ifndef rtmSetZCCacheNeedsReset
#define rtmSetZCCacheNeedsReset(rtm, val) ((rtm)->zCCacheNeedsReset = (val))
#endif

#ifndef rtmGetdX
#define rtmGetdX(rtm)                  ((rtm)->derivs)
#endif

#ifndef rtmSetdX
#define rtmSetdX(rtm, val)             ((rtm)->derivs = (val))
#endif

#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
#define rtmGetStopRequested(rtm)       ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
#define rtmSetStopRequested(rtm, val)  ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
#define rtmGetStopRequestedPtr(rtm)    (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
#define rtmGetT(rtm)                   (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTPtr
#define rtmGetTPtr(rtm)                ((rtm)->Timing.t)
#endif

// Block signals (default storage)
struct B_CACC_Center_T {
  SL_Bus_nav_msgs_Odometry In1;        // '<S30>/In1'
  SL_Bus_nav_msgs_Odometry b_varargout_2;
  SL_Bus_auna_its_msgs_CAM In1_a;      // '<S32>/In1'
  SL_Bus_auna_its_msgs_CAM b_varargout_2_m;
  SL_Bus_geometry_msgs_PoseStamped In1_g;// '<S29>/In1'
  SL_Bus_geometry_msgs_PoseStamped b_varargout_2_c;
  SL_Bus_geometry_msgs_Twist BusAssignment;// '<S2>/Bus Assignment'
  real_T curv;                         // '<S3>/Divide'
  real_T v;                            // '<S2>/Product'
  real_T Memory;                       // '<S2>/Memory'
  real_T Switch2;                      // '<S2>/Switch2'
  real_T Gain2;                        // '<S2>/Gain2'
  real_T Switch1;                      // '<S2>/Switch1'
  real_T Switch4;                      // '<S2>/Switch4'
  real_T CoordinateTransformationConvers[3];
                                 // '<S2>/Coordinate Transformation Conversion'
  real_T h;                            // '<S4>/Switch1'
  real_T r;                            // '<S4>/Switch'
  real_T Merge;                        // '<S11>/Merge'
  real_T Fcn;                          // '<S7>/Fcn'
  real_T z1;                           // '<S4>/z1'
  real_T z2;                           // '<S4>/z2'
  real_T z3;                           // '<S4>/z3'
  real_T z4;                           // '<S4>/z4'
  real_T Fcn4_o;                       // '<S9>/Fcn4'
  real_T Fcn5_e;                       // '<S9>/Fcn5'
  real_T Add_a[2];                     // '<S9>/Add'
  real_T aSinInput;
  real_T e_data;
  real_T b_x_data;
  real_T b;
  real_T Gain;                         // '<S9>/Gain'
  real_T Fcn_k;                        // '<S9>/Fcn'
  real_T Fcn1;                         // '<S9>/Fcn1'
  real_T Fcn2;                         // '<S9>/Fcn2'
  real_T Fcn3;                         // '<S9>/Fcn3'
  real_T y_idx_0;
  real_T y_idx_1;
  real_T y_idx_2;
  real_T CoordinateTransformationConve_c;
  real_T CoordinateTransformationConve_b;
  real_T CoordinateTransformationConve_p;
  SL_Bus_example_interfaces_Float64 In1_m;// '<S13>/In1'
  SL_Bus_example_interfaces_Float64 In1_h;// '<S12>/In1'
  SL_Bus_example_interfaces_Float64 b_varargout_2_cv;
};

// Block states (default storage) for system '<Root>'
struct DW_CACC_Center_T {
  ros_slros2_internal_block_Sub_T obj; // '<S31>/SourceBlock'
  ros_slros2_internal_block_Sub_T obj_k;// '<S28>/SourceBlock'
  ros_slros2_internal_block_Sub_T obj_f;// '<S27>/SourceBlock'
  ros_slros2_internal_block_Sub_T obj_n;// '<S6>/SourceBlock'
  ros_slros2_internal_block_Sub_T obj_p;// '<S5>/SourceBlock'
  ros_slros2_internal_block_Pub_T obj_c;// '<S26>/SinkBlock'
  real_T UnitDelay1_DSTATE;            // '<S4>/Unit Delay1'
  real_T UnitDelay_DSTATE;             // '<S4>/Unit Delay'
  real_T TimeStampA;                   // '<S3>/Derivative2'
  real_T LastUAtTimeA;                 // '<S3>/Derivative2'
  real_T TimeStampB;                   // '<S3>/Derivative2'
  real_T LastUAtTimeB;                 // '<S3>/Derivative2'
  real_T Memory_PreviousInput;         // '<S2>/Memory'
  real_T Memory1_PreviousInput;        // '<S2>/Memory1'
  real_T TimeStampA_k;                 // '<S4>/Derivative'
  real_T LastUAtTimeA_h;               // '<S4>/Derivative'
  real_T TimeStampB_c;                 // '<S4>/Derivative'
  real_T LastUAtTimeB_m;               // '<S4>/Derivative'
  boolean_T IC_FirstOutputTime;        // '<S4>/IC'
  boolean_T IC1_FirstOutputTime;       // '<S4>/IC1'
};

// Continuous states (default storage)
struct X_CACC_Center_T {
  real_T Integrator_CSTATE;            // '<S2>/Integrator'
};

// State derivatives (default storage)
struct XDot_CACC_Center_T {
  real_T Integrator_CSTATE;            // '<S2>/Integrator'
};

// State disabled
struct XDis_CACC_Center_T {
  boolean_T Integrator_CSTATE;         // '<S2>/Integrator'
};

#ifndef ODE3_INTG
#define ODE3_INTG

// ODE3 Integration Data
struct ODE3_IntgData {
  real_T *y;                           // output
  real_T *f[3];                        // derivatives
};

#endif

// Parameters (default storage)
struct P_CACC_Center_T_ {
  real_T L;                            // Variable: L
                                          //  Referenced by: '<S2>/Multiply'

  real_T h;                            // Variable: h
                                          //  Referenced by: '<S4>/IC1'

  real_T k1;                           // Variable: k1
                                          //  Referenced by:
                                          //    '<S8>/Gain'
                                          //    '<S9>/Gain'

  real_T k2;                           // Variable: k2
                                          //  Referenced by:
                                          //    '<S8>/Gain1'
                                          //    '<S9>/Gain1'

  real_T r;                            // Variable: r
                                          //  Referenced by: '<S4>/IC'

  real_T v_threshold;                  // Variable: v_threshold
                                          //  Referenced by:
                                          //    '<S2>/Switch1'
                                          //    '<S2>/Switch2'
                                          //    '<S2>/Switch3'

  SL_Bus_nav_msgs_Odometry Out1_Y0;    // Computed Parameter: Out1_Y0
                                          //  Referenced by: '<S30>/Out1'

  SL_Bus_nav_msgs_Odometry Constant_Value;// Computed Parameter: Constant_Value
                                             //  Referenced by: '<S28>/Constant'

  SL_Bus_auna_its_msgs_CAM Out1_Y0_k;  // Computed Parameter: Out1_Y0_k
                                          //  Referenced by: '<S32>/Out1'

  SL_Bus_auna_its_msgs_CAM Constant_Value_e;// Computed Parameter: Constant_Value_e
                                               //  Referenced by: '<S31>/Constant'

  SL_Bus_geometry_msgs_PoseStamped Out1_Y0_o;// Computed Parameter: Out1_Y0_o
                                                //  Referenced by: '<S29>/Out1'

  SL_Bus_geometry_msgs_PoseStamped Constant_Value_b;// Computed Parameter: Constant_Value_b
                                                       //  Referenced by: '<S27>/Constant'

  SL_Bus_geometry_msgs_Twist Constant_Value_i;// Computed Parameter: Constant_Value_i
                                                 //  Referenced by: '<S25>/Constant'

  SL_Bus_example_interfaces_Float64 Out1_Y0_b;// Computed Parameter: Out1_Y0_b
                                                 //  Referenced by: '<S12>/Out1'

  SL_Bus_example_interfaces_Float64 Out1_Y0_a;// Computed Parameter: Out1_Y0_a
                                                 //  Referenced by: '<S13>/Out1'

  SL_Bus_example_interfaces_Float64 Constant_Value_m;// Computed Parameter: Constant_Value_m
                                                        //  Referenced by: '<S6>/Constant'

  SL_Bus_example_interfaces_Float64 Constant_Value_h;// Computed Parameter: Constant_Value_h
                                                        //  Referenced by: '<S5>/Constant'

  real_T UnitDelay1_InitialCondition;  // Expression: 0
                                          //  Referenced by: '<S4>/Unit Delay1'

  real_T UnitDelay_InitialCondition;   // Expression: 0
                                          //  Referenced by: '<S4>/Unit Delay'

  real_T One_Value;                    // Expression: 1
                                          //  Referenced by: '<S3>/One'

  real_T Memory_InitialCondition;      // Expression: 0
                                          //  Referenced by: '<S2>/Memory'

  real_T Integrator_IC;                // Expression: 0.0
                                          //  Referenced by: '<S2>/Integrator'

  real_T Memory1_InitialCondition;     // Expression: 0
                                          //  Referenced by: '<S2>/Memory1'

  real_T Gain2_Gain;                   // Expression: 0
                                          //  Referenced by: '<S2>/Gain2'

  real_T Constant1_Value;              // Expression: 1
                                          //  Referenced by: '<S2>/Constant1'

  real_T Constant_Value_f;             // Expression: 0
                                          //  Referenced by: '<S2>/Constant'

  real_T Zero_Value;                   // Expression: 0
                                          //  Referenced by: '<S2>/Zero'

  real_T Switch4_Threshold;            // Expression: 0
                                          //  Referenced by: '<S2>/Switch4'

};

// Real-time Model Data Structure
struct tag_RTM_CACC_Center_T {
  const char_T *errorStatus;
  RTWSolverInfo solverInfo;
  X_CACC_Center_T *contStates;
  int_T *periodicContStateIndices;
  real_T *periodicContStateRanges;
  real_T *derivs;
  XDis_CACC_Center_T *contStateDisabled;
  boolean_T zCCacheNeedsReset;
  boolean_T derivCacheNeedsReset;
  boolean_T CTOutputIncnstWithState;
  real_T odeY[1];
  real_T odeF[3][1];
  ODE3_IntgData intgData;

  //
  //  Sizes:
  //  The following substructure contains sizes information
  //  for many of the model attributes such as inputs, outputs,
  //  dwork, sample times, etc.

  struct {
    int_T numContStates;
    int_T numPeriodicContStates;
    int_T numSampTimes;
  } Sizes;

  //
  //  Timing:
  //  The following substructure contains information regarding
  //  the timing information for the model.

  struct {
    uint32_T clockTick0;
    time_T stepSize0;
    uint32_T clockTick1;
    struct {
      uint8_T TID[3];
    } TaskCounters;

    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *t;
    time_T tArray[3];
  } Timing;
};

// Class declaration for model CACC_Center
class CACC_Center
{
  // public data and function members
 public:
  // Real-Time Model get method
  RT_MODEL_CACC_Center_T * getRTM();

  // model initialize function
  void initialize();

  // model step function
  void step();

  // model terminate function
  void terminate();

  // Constructor
  CACC_Center();

  // Destructor
  ~CACC_Center();

  // private data and function members
 private:
  // Block signals
  B_CACC_Center_T CACC_Center_B;

  // Block states
  DW_CACC_Center_T CACC_Center_DW;

  // Tunable parameters
  static P_CACC_Center_T CACC_Center_P;

  // Block continuous states
  X_CACC_Center_T CACC_Center_X;

  // private member function(s) for subsystem '<Root>'
  void CACC_Center_binary_expand_op(real_T in1[3], const real_T in3_data[],
    const int32_T *in3_size, const real_T in4_data[], const int32_T *in4_size);
  void CACC_Center_SystemCore_setup_d(ros_slros2_internal_block_Sub_T *obj);
  void CACC_Center_SystemCore_setup(ros_slros2_internal_block_Sub_T *obj);
  void CACC_Cente_SystemCore_setup_dfd(ros_slros2_internal_block_Sub_T *obj);
  void CACC_Cen_SystemCore_setup_dfdzt(ros_slros2_internal_block_Sub_T *obj);
  void CACC_Cent_SystemCore_setup_dfdz(ros_slros2_internal_block_Sub_T *obj);
  void CACC_Center_SystemCore_setup_df(ros_slros2_internal_block_Pub_T *obj);

  // Continuous states update member function
  void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si );

  // Derivatives member function
  void CACC_Center_derivatives();

  // Real-Time Model
  RT_MODEL_CACC_Center_T CACC_Center_M;
};

extern volatile boolean_T stopRequested;
extern volatile boolean_T runModel;

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<S8>/Add3' : Unused code path elimination
//  Block '<S8>/Product8' : Unused code path elimination
//  Block '<S8>/Product9' : Unused code path elimination
//  Block '<S8>/Scope' : Unused code path elimination
//  Block '<S8>/Scope1' : Unused code path elimination
//  Block '<S8>/Scope2' : Unused code path elimination
//  Block '<S8>/Scope3' : Unused code path elimination
//  Block '<S8>/Scope4' : Unused code path elimination
//  Block '<S2>/Derivative' : Unused code path elimination
//  Block '<S2>/Divide' : Unused code path elimination
//  Block '<S2>/One' : Unused code path elimination
//  Block '<S2>/Switch' : Unused code path elimination


//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'CACC_Center'
//  '<S1>'   : 'CACC_Center/controller'
//  '<S2>'   : 'CACC_Center/publisher'
//  '<S3>'   : 'CACC_Center/subscriber'
//  '<S4>'   : 'CACC_Center/controller/LA'
//  '<S5>'   : 'CACC_Center/controller/LA/Subscribe'
//  '<S6>'   : 'CACC_Center/controller/LA/Subscribe1'
//  '<S7>'   : 'CACC_Center/controller/LA/alpha'
//  '<S8>'   : 'CACC_Center/controller/LA/dz3,dz4 calculation'
//  '<S9>'   : 'CACC_Center/controller/LA/in parentheses'
//  '<S10>'  : 'CACC_Center/controller/LA/invGam'
//  '<S11>'  : 'CACC_Center/controller/LA/s'
//  '<S12>'  : 'CACC_Center/controller/LA/Subscribe/Enabled Subsystem'
//  '<S13>'  : 'CACC_Center/controller/LA/Subscribe1/Enabled Subsystem'
//  '<S14>'  : 'CACC_Center/controller/LA/dz3,dz4 calculation/GamInv '
//  '<S15>'  : 'CACC_Center/controller/LA/dz3,dz4 calculation/H_i'
//  '<S16>'  : 'CACC_Center/controller/LA/dz3,dz4 calculation/H_i-1'
//  '<S17>'  : 'CACC_Center/controller/LA/dz3,dz4 calculation/R(alph)'
//  '<S18>'  : 'CACC_Center/controller/LA/dz3,dz4 calculation/Rinv(alph)'
//  '<S19>'  : 'CACC_Center/controller/LA/dz3,dz4 calculation/beta1'
//  '<S20>'  : 'CACC_Center/controller/LA/dz3,dz4 calculation/beta2'
//  '<S21>'  : 'CACC_Center/controller/LA/dz3,dz4 calculation/eq Sig3 line 3'
//  '<S22>'  : 'CACC_Center/controller/LA/dz3,dz4 calculation/eq Sig3 line 3/mat'
//  '<S23>'  : 'CACC_Center/controller/LA/s/Approximation of s around zero'
//  '<S24>'  : 'CACC_Center/controller/LA/s/function s'
//  '<S25>'  : 'CACC_Center/publisher/Blank Message'
//  '<S26>'  : 'CACC_Center/publisher/Publish'
//  '<S27>'  : 'CACC_Center/publisher/Subscribe'
//  '<S28>'  : 'CACC_Center/publisher/Subscribe1'
//  '<S29>'  : 'CACC_Center/publisher/Subscribe/Enabled Subsystem'
//  '<S30>'  : 'CACC_Center/publisher/Subscribe1/Enabled Subsystem'
//  '<S31>'  : 'CACC_Center/subscriber/Subscribe'
//  '<S32>'  : 'CACC_Center/subscriber/Subscribe/Enabled Subsystem'

#endif                                 // RTW_HEADER_CACC_Center_h_

//
// File trailer for generated code.
//
// [EOF]
//
