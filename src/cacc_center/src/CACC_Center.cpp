//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: CACC_Center.cpp
//
// Code generated for Simulink model 'CACC_Center'.
//
// Model version                  : 4.5
// Simulink Coder version         : 9.8 (R2022b) 13-May-2022
// C/C++ source code generated on : Mon Jan 30 12:36:54 2023
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Linux 64)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "CACC_Center.h"
#include "CACC_Center_types.h"
#include "rtwtypes.h"
#include <math.h>
#include "CACC_Center_private.h"

extern "C"
{

#include "rt_nonfinite.h"

}

#include "rmw/qos_profiles.h"
#include "rmw/types.h"
#include <stddef.h>
#include "rt_defines.h"

static void rate_scheduler(RT_MODEL_CACC_Center_T *const CACC_Center_M);

//
//         This function updates active task flag for each subrate.
//         The function is called at model base rate, hence the
//         generated code self-manages all its subrates.
//
static void rate_scheduler(RT_MODEL_CACC_Center_T *const CACC_Center_M)
{
  // Compute which subrates run during the next base time step.  Subrates
  //  are an integer multiple of the base rate counter.  Therefore, the subtask
  //  counter is reset when it reaches its limit (zero means run).

  (CACC_Center_M->Timing.TaskCounters.TID[2])++;
  if ((CACC_Center_M->Timing.TaskCounters.TID[2]) > 1) {// Sample time: [0.1s, 0.0s] 
    CACC_Center_M->Timing.TaskCounters.TID[2] = 0;
  }
}

//
// This function updates continuous states using the ODE3 fixed-step
// solver algorithm
//
void CACC_Center::rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  // Solver Matrices
  static const real_T rt_ODE3_A[3] = {
    1.0/2.0, 3.0/4.0, 1.0
  };

  static const real_T rt_ODE3_B[3][3] = {
    { 1.0/2.0, 0.0, 0.0 },

    { 0.0, 3.0/4.0, 0.0 },

    { 2.0/9.0, 1.0/3.0, 4.0/9.0 }
  };

  time_T t = rtsiGetT(si);
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE3_IntgData *id = static_cast<ODE3_IntgData *>(rtsiGetSolverData(si));
  real_T *y = id->y;
  real_T *f0 = id->f[0];
  real_T *f1 = id->f[1];
  real_T *f2 = id->f[2];
  real_T hB[3];
  int_T i;
  int_T nXc = 1;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  // Save the state values at time t in y, we'll use x as ynew.
  (void) memcpy(y, x,
                static_cast<uint_T>(nXc)*sizeof(real_T));

  // Assumes that rtsiSetT and ModelOutputs are up-to-date
  // f0 = f(t,y)
  rtsiSetdX(si, f0);
  CACC_Center_derivatives();

  // f(:,2) = feval(odefile, t + hA(1), y + f*hB(:,1), args(:)(*));
  hB[0] = h * rt_ODE3_B[0][0];
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[0]);
  rtsiSetdX(si, f1);
  this->step();
  CACC_Center_derivatives();

  // f(:,3) = feval(odefile, t + hA(2), y + f*hB(:,2), args(:)(*));
  for (i = 0; i <= 1; i++) {
    hB[i] = h * rt_ODE3_B[1][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[1]);
  rtsiSetdX(si, f2);
  this->step();
  CACC_Center_derivatives();

  // tnew = t + hA(3);
  // ynew = y + f*hB(:,3);
  for (i = 0; i <= 2; i++) {
    hB[i] = h * rt_ODE3_B[2][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1] + f2[i]*hB[2]);
  }

  rtsiSetT(si, tnew);
  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    int32_T tmp;
    int32_T tmp_0;
    if (u0 > 0.0) {
      tmp = 1;
    } else {
      tmp = -1;
    }

    if (u1 > 0.0) {
      tmp_0 = 1;
    } else {
      tmp_0 = -1;
    }

    y = atan2(static_cast<real_T>(tmp), static_cast<real_T>(tmp_0));
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

void CACC_Center::CACC_Center_binary_expand_op(real_T in1[3], const real_T
  in3_data[], const int32_T *in3_size, const real_T in4_data[], const int32_T
  *in4_size)
{
  int32_T loop_ub;

  // MATLABSystem: '<S2>/Coordinate Transformation Conversion'
  loop_ub = *in4_size == 1 ? *in3_size : *in4_size;
  for (int32_T i = 0; i < loop_ub; i++) {
    in1[0] = -in3_data[0] * 2.0 * in4_data[0];
  }

  // End of MATLABSystem: '<S2>/Coordinate Transformation Conversion'
}

real_T rt_powd_snf(real_T u0, real_T u1)
{
  real_T y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else {
    real_T tmp;
    real_T tmp_0;
    tmp = fabs(u0);
    tmp_0 = fabs(u1);
    if (rtIsInf(u1)) {
      if (tmp == 1.0) {
        y = 1.0;
      } else if (tmp > 1.0) {
        if (u1 > 0.0) {
          y = (rtInf);
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = (rtInf);
      }
    } else if (tmp_0 == 0.0) {
      y = 1.0;
    } else if (tmp_0 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > floor(u1))) {
      y = (rtNaN);
    } else {
      y = pow(u0, u1);
    }
  }

  return y;
}

void CACC_Center::CACC_Center_SystemCore_setup_d(ros_slros2_internal_block_Sub_T
  *obj)
{
  rmw_qos_durability_policy_t durability;
  rmw_qos_history_policy_t history;
  rmw_qos_profile_t qos_profile;
  rmw_qos_reliability_policy_t reliability;
  char_T b_zeroDelimTopic[14];
  static const char_T tmp[13] = { 'c', 'a', 'c', 'c', '/', 't', 'i', 'm', 'e',
    '_', 'g', 'a', 'p' };

  obj->isInitialized = 1;
  qos_profile = rmw_qos_profile_default;
  history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  SET_QOS_VALUES(qos_profile, history, (size_t)1.0, durability, reliability);
  for (int32_T i = 0; i < 13; i++) {
    b_zeroDelimTopic[i] = tmp[i];
  }

  b_zeroDelimTopic[13] = '\x00';
  Sub_CACC_Center_2273.createSubscriber(&b_zeroDelimTopic[0], qos_profile);
  obj->isSetupComplete = true;
}

void CACC_Center::CACC_Center_SystemCore_setup(ros_slros2_internal_block_Sub_T
  *obj)
{
  rmw_qos_durability_policy_t durability;
  rmw_qos_history_policy_t history;
  rmw_qos_profile_t qos_profile;
  rmw_qos_reliability_policy_t reliability;
  char_T b_zeroDelimTopic[25];
  static const char_T tmp[24] = { 'c', 'a', 'c', 'c', '/', 's', 't', 'a', 'n',
    'd', 's', 't', 'i', 'l', 'l', '_', 'd', 'i', 's', 't', 'a', 'n', 'c', 'e' };

  obj->isInitialized = 1;
  qos_profile = rmw_qos_profile_default;
  history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  SET_QOS_VALUES(qos_profile, history, (size_t)1.0, durability, reliability);
  for (int32_T i = 0; i < 24; i++) {
    b_zeroDelimTopic[i] = tmp[i];
  }

  b_zeroDelimTopic[24] = '\x00';
  Sub_CACC_Center_2234.createSubscriber(&b_zeroDelimTopic[0], qos_profile);
  obj->isSetupComplete = true;
}

void CACC_Center::CACC_Cente_SystemCore_setup_dfd
  (ros_slros2_internal_block_Sub_T *obj)
{
  rmw_qos_durability_policy_t durability;
  rmw_qos_history_policy_t history;
  rmw_qos_profile_t qos_profile;
  rmw_qos_reliability_policy_t reliability;
  char_T b_zeroDelimTopic[18];
  static const char_T tmp[17] = { 'l', 'o', 'c', 'a', 'l', 'i', 'z', 'a', 't',
    'i', 'o', 'n', '_', 'p', 'o', 's', 'e' };

  obj->isInitialized = 1;
  qos_profile = rmw_qos_profile_default;
  history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  SET_QOS_VALUES(qos_profile, history, (size_t)1.0, durability, reliability);
  for (int32_T i = 0; i < 17; i++) {
    b_zeroDelimTopic[i] = tmp[i];
  }

  b_zeroDelimTopic[17] = '\x00';
  Sub_CACC_Center_2221.createSubscriber(&b_zeroDelimTopic[0], qos_profile);
  obj->isSetupComplete = true;
}

void CACC_Center::CACC_Cen_SystemCore_setup_dfdzt
  (ros_slros2_internal_block_Sub_T *obj)
{
  rmw_qos_durability_policy_t durability;
  rmw_qos_history_policy_t history;
  rmw_qos_profile_t qos_profile;
  rmw_qos_reliability_policy_t reliability;
  char_T b_zeroDelimTopic[13];
  static const char_T tmp[12] = { 'c', 'a', 'm', '_', 'f', 'i', 'l', 't', 'e',
    'r', 'e', 'd' };

  obj->isInitialized = 1;
  qos_profile = rmw_qos_profile_default;
  history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  SET_QOS_VALUES(qos_profile, history, (size_t)1.0, durability, reliability);
  for (int32_T i = 0; i < 12; i++) {
    b_zeroDelimTopic[i] = tmp[i];
  }

  b_zeroDelimTopic[12] = '\x00';
  Sub_CACC_Center_2128.createSubscriber(&b_zeroDelimTopic[0], qos_profile);
  obj->isSetupComplete = true;
}

void CACC_Center::CACC_Cent_SystemCore_setup_dfdz
  (ros_slros2_internal_block_Sub_T *obj)
{
  rmw_qos_durability_policy_t durability;
  rmw_qos_history_policy_t history;
  rmw_qos_profile_t qos_profile;
  rmw_qos_reliability_policy_t reliability;
  char_T b_zeroDelimTopic[5];
  obj->isInitialized = 1;
  qos_profile = rmw_qos_profile_default;
  history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  SET_QOS_VALUES(qos_profile, history, (size_t)1.0, durability, reliability);
  b_zeroDelimTopic[0] = 'o';
  b_zeroDelimTopic[1] = 'd';
  b_zeroDelimTopic[2] = 'o';
  b_zeroDelimTopic[3] = 'm';
  b_zeroDelimTopic[4] = '\x00';
  Sub_CACC_Center_2222.createSubscriber(&b_zeroDelimTopic[0], qos_profile);
  obj->isSetupComplete = true;
}

void CACC_Center::CACC_Center_SystemCore_setup_df
  (ros_slros2_internal_block_Pub_T *obj)
{
  rmw_qos_durability_policy_t durability;
  rmw_qos_history_policy_t history;
  rmw_qos_profile_t qos_profile;
  rmw_qos_reliability_policy_t reliability;
  char_T b_zeroDelimTopic[8];
  static const char_T tmp[7] = { 'c', 'm', 'd', '_', 'v', 'e', 'l' };

  obj->isInitialized = 1;
  qos_profile = rmw_qos_profile_default;
  history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  SET_QOS_VALUES(qos_profile, history, (size_t)1.0, durability, reliability);
  for (int32_T i = 0; i < 7; i++) {
    b_zeroDelimTopic[i] = tmp[i];
  }

  b_zeroDelimTopic[7] = '\x00';
  Pub_CACC_Center_2215.createPublisher(&b_zeroDelimTopic[0], qos_profile);
  obj->isSetupComplete = true;
}

// Model step function
void CACC_Center::step()
{
  // local block i/o variables
  real_T rtb_IC;
  real_T rtb_IC1;
  if (rtmIsMajorTimeStep((&CACC_Center_M))) {
    // set solver stop time
    rtsiSetSolverStopTime(&(&CACC_Center_M)->solverInfo,(((&CACC_Center_M)
      ->Timing.clockTick0+1)*(&CACC_Center_M)->Timing.stepSize0));
  }                                    // end MajorTimeStep

  // Update absolute time of base rate at minor time step
  if (rtmIsMinorTimeStep((&CACC_Center_M))) {
    (&CACC_Center_M)->Timing.t[0] = rtsiGetT(&(&CACC_Center_M)->solverInfo);
  }

  {
    int32_T n_size_idx_1;
    int32_T trueCount;
    boolean_T mask1;
    if (rtmIsMajorTimeStep((&CACC_Center_M)) &&
        (&CACC_Center_M)->Timing.TaskCounters.TID[2] == 0) {
      real_T c_idx_0;
      boolean_T mask2;

      // MATLABSystem: '<S27>/SourceBlock'
      mask1 = Sub_CACC_Center_2221.getLatestMessage
        (&CACC_Center_B.b_varargout_2_c);

      // Outputs for Enabled SubSystem: '<S27>/Enabled Subsystem' incorporates:
      //   EnablePort: '<S29>/Enable'

      if (mask1) {
        // SignalConversion generated from: '<S29>/In1'
        CACC_Center_B.In1_g = CACC_Center_B.b_varargout_2_c;
      }

      // End of MATLABSystem: '<S27>/SourceBlock'
      // End of Outputs for SubSystem: '<S27>/Enabled Subsystem'

      // MATLABSystem: '<S2>/Coordinate Transformation Conversion' incorporates:
      //   SignalConversion generated from: '<S2>/Vector Concatenate'
      //
      CACC_Center_B.b = 1.0 / sqrt(((CACC_Center_B.In1_g.pose.orientation.w *
        CACC_Center_B.In1_g.pose.orientation.w +
        CACC_Center_B.In1_g.pose.orientation.x *
        CACC_Center_B.In1_g.pose.orientation.x) +
        CACC_Center_B.In1_g.pose.orientation.y *
        CACC_Center_B.In1_g.pose.orientation.y) +
        CACC_Center_B.In1_g.pose.orientation.z *
        CACC_Center_B.In1_g.pose.orientation.z);
      CACC_Center_B.y_idx_0 = CACC_Center_B.In1_g.pose.orientation.w *
        CACC_Center_B.b;
      CACC_Center_B.y_idx_1 = CACC_Center_B.In1_g.pose.orientation.x *
        CACC_Center_B.b;
      CACC_Center_B.y_idx_2 = CACC_Center_B.In1_g.pose.orientation.y *
        CACC_Center_B.b;
      CACC_Center_B.b *= CACC_Center_B.In1_g.pose.orientation.z;
      CACC_Center_B.aSinInput = (CACC_Center_B.y_idx_1 * CACC_Center_B.b -
        CACC_Center_B.y_idx_0 * CACC_Center_B.y_idx_2) * -2.0;
      mask1 = (CACC_Center_B.aSinInput >= 0.99999999999999778);
      mask2 = (CACC_Center_B.aSinInput <= -0.99999999999999778);
      if (mask1) {
        CACC_Center_B.aSinInput = 1.0;
      }

      c_idx_0 = CACC_Center_B.aSinInput;
      if (mask2) {
        c_idx_0 = -1.0;
      }

      mask1 = (mask1 || mask2);
      CACC_Center_B.aSinInput = CACC_Center_B.y_idx_0 * CACC_Center_B.y_idx_0;
      CACC_Center_B.CoordinateTransformationConve_c = CACC_Center_B.y_idx_1 *
        CACC_Center_B.y_idx_1;
      CACC_Center_B.CoordinateTransformationConve_b = CACC_Center_B.y_idx_2 *
        CACC_Center_B.y_idx_2;
      CACC_Center_B.CoordinateTransformationConve_p = CACC_Center_B.b *
        CACC_Center_B.b;
      CACC_Center_B.CoordinateTransformationConvers[0] = rt_atan2d_snf
        ((CACC_Center_B.y_idx_1 * CACC_Center_B.y_idx_2 + CACC_Center_B.y_idx_0 *
          CACC_Center_B.b) * 2.0, ((CACC_Center_B.aSinInput +
           CACC_Center_B.CoordinateTransformationConve_c) -
          CACC_Center_B.CoordinateTransformationConve_b) -
         CACC_Center_B.CoordinateTransformationConve_p);
      CACC_Center_B.CoordinateTransformationConvers[1] = asin(c_idx_0);
      CACC_Center_B.CoordinateTransformationConvers[2] = rt_atan2d_snf
        ((CACC_Center_B.y_idx_2 * CACC_Center_B.b + CACC_Center_B.y_idx_0 *
          CACC_Center_B.y_idx_1) * 2.0, ((CACC_Center_B.aSinInput -
           CACC_Center_B.CoordinateTransformationConve_c) -
          CACC_Center_B.CoordinateTransformationConve_b) +
         CACC_Center_B.CoordinateTransformationConve_p);
      trueCount = 0;
      if (mask1) {
        for (int32_T d_i = 0; d_i < 1; d_i++) {
          trueCount++;
        }
      }

      n_size_idx_1 = trueCount;
      if (trueCount - 1 >= 0) {
        CACC_Center_B.b_x_data = c_idx_0;
      }

      trueCount--;
      for (int32_T d_i = 0; d_i <= trueCount; d_i++) {
        if (rtIsNaN(CACC_Center_B.b_x_data)) {
          CACC_Center_B.y_idx_2 = (rtNaN);
        } else if (CACC_Center_B.b_x_data < 0.0) {
          CACC_Center_B.y_idx_2 = -1.0;
        } else {
          CACC_Center_B.y_idx_2 = (CACC_Center_B.b_x_data > 0.0);
        }

        CACC_Center_B.b_x_data = CACC_Center_B.y_idx_2;
      }

      trueCount = 0;
      if (mask1) {
        for (int32_T d_i = 0; d_i < 1; d_i++) {
          trueCount++;
        }
      }

      if (trueCount - 1 >= 0) {
        CACC_Center_B.e_data = rt_atan2d_snf(CACC_Center_B.y_idx_1,
          CACC_Center_B.y_idx_0);
      }

      if (n_size_idx_1 == trueCount) {
        if (n_size_idx_1 - 1 >= 0) {
          CACC_Center_B.CoordinateTransformationConvers[0] =
            -CACC_Center_B.b_x_data * 2.0 * CACC_Center_B.e_data;
        }
      } else {
        CACC_Center_binary_expand_op
          (CACC_Center_B.CoordinateTransformationConvers,
           &CACC_Center_B.b_x_data, &n_size_idx_1, &CACC_Center_B.e_data,
           &trueCount);
      }

      trueCount = 0;
      if (mask1) {
        for (int32_T d_i = 0; d_i < 1; d_i++) {
          trueCount++;
        }
      }

      if (trueCount - 1 >= 0) {
        CACC_Center_B.CoordinateTransformationConvers[2] = 0.0;
      }

      // End of MATLABSystem: '<S2>/Coordinate Transformation Conversion'

      // MATLABSystem: '<S31>/SourceBlock'
      mask1 = Sub_CACC_Center_2128.getLatestMessage
        (&CACC_Center_B.b_varargout_2_m);

      // Outputs for Enabled SubSystem: '<S31>/Enabled Subsystem' incorporates:
      //   EnablePort: '<S32>/Enable'

      if (mask1) {
        // SignalConversion generated from: '<S32>/In1'
        CACC_Center_B.In1_a = CACC_Center_B.b_varargout_2_m;
      }

      // End of MATLABSystem: '<S31>/SourceBlock'
      // End of Outputs for SubSystem: '<S31>/Enabled Subsystem'

      // Switch: '<S3>/Switch' incorporates:
      //   Constant: '<S3>/One'

      if (CACC_Center_B.In1_a.v != 0.0) {
        CACC_Center_B.y_idx_0 = CACC_Center_B.In1_a.v;
      } else {
        CACC_Center_B.y_idx_0 = CACC_Center_P.One_Value;
      }

      // Product: '<S3>/Divide' incorporates:
      //   Switch: '<S3>/Switch'

      CACC_Center_B.curv = CACC_Center_B.In1_a.thetadot / CACC_Center_B.y_idx_0;

      // MATLABSystem: '<S28>/SourceBlock'
      mask1 = Sub_CACC_Center_2222.getLatestMessage(&CACC_Center_B.b_varargout_2);

      // Outputs for Enabled SubSystem: '<S28>/Enabled Subsystem' incorporates:
      //   EnablePort: '<S30>/Enable'

      if (mask1) {
        // SignalConversion generated from: '<S30>/In1'
        CACC_Center_B.In1 = CACC_Center_B.b_varargout_2;
      }

      // End of MATLABSystem: '<S28>/SourceBlock'
      // End of Outputs for SubSystem: '<S28>/Enabled Subsystem'

      // Signum: '<S2>/Sign'
      if (rtIsNaN(CACC_Center_B.In1.twist.twist.linear.x)) {
        CACC_Center_B.y_idx_0 = (rtNaN);
      } else if (CACC_Center_B.In1.twist.twist.linear.x < 0.0) {
        CACC_Center_B.y_idx_0 = -1.0;
      } else {
        CACC_Center_B.y_idx_0 = (CACC_Center_B.In1.twist.twist.linear.x > 0.0);
      }

      // Product: '<S2>/Product' incorporates:
      //   Math: '<S2>/Square'
      //   Math: '<S2>/Square1'
      //   Signum: '<S2>/Sign'
      //   Sqrt: '<S2>/Square Root'
      //   Sum: '<S2>/Add'

      CACC_Center_B.v = sqrt(CACC_Center_B.In1.twist.twist.linear.x *
        CACC_Center_B.In1.twist.twist.linear.x +
        CACC_Center_B.In1.twist.twist.linear.y *
        CACC_Center_B.In1.twist.twist.linear.y) * CACC_Center_B.y_idx_0;
    }

    // Outputs for Atomic SubSystem: '<S1>/LA'
    if (rtmIsMajorTimeStep((&CACC_Center_M)) &&
        (&CACC_Center_M)->Timing.TaskCounters.TID[1] == 0) {
      // MATLABSystem: '<S6>/SourceBlock'
      mask1 = Sub_CACC_Center_2273.getLatestMessage
        (&CACC_Center_B.b_varargout_2_cv);

      // Outputs for Enabled SubSystem: '<S6>/Enabled Subsystem' incorporates:
      //   EnablePort: '<S13>/Enable'

      // Switch: '<S4>/Switch1' incorporates:
      //   MATLABSystem: '<S6>/SourceBlock'

      if (mask1) {
        // SignalConversion generated from: '<S13>/In1'
        CACC_Center_B.In1_m = CACC_Center_B.b_varargout_2_cv;

        // Switch: '<S4>/Switch1'
        CACC_Center_B.h = CACC_Center_B.In1_m.data;
      } else {
        // Switch: '<S4>/Switch1' incorporates:
        //   UnitDelay: '<S4>/Unit Delay1'

        CACC_Center_B.h = CACC_Center_DW.UnitDelay1_DSTATE;
      }

      // End of Switch: '<S4>/Switch1'
      // End of Outputs for SubSystem: '<S6>/Enabled Subsystem'

      // MATLABSystem: '<S5>/SourceBlock'
      mask1 = Sub_CACC_Center_2234.getLatestMessage
        (&CACC_Center_B.b_varargout_2_cv);

      // UnitDelay: '<S4>/Unit Delay' incorporates:
      //   UnitDelay: '<S4>/Unit Delay1'

      CACC_Center_DW.UnitDelay1_DSTATE = CACC_Center_DW.UnitDelay_DSTATE;

      // Outputs for Enabled SubSystem: '<S5>/Enabled Subsystem' incorporates:
      //   EnablePort: '<S12>/Enable'

      // Switch: '<S4>/Switch' incorporates:
      //   MATLABSystem: '<S5>/SourceBlock'

      if (mask1) {
        // SignalConversion generated from: '<S12>/In1'
        CACC_Center_B.In1_h = CACC_Center_B.b_varargout_2_cv;

        // Switch: '<S4>/Switch'
        CACC_Center_B.r = CACC_Center_B.In1_h.data;
      } else {
        // Switch: '<S4>/Switch' incorporates:
        //   UnitDelay: '<S4>/Unit Delay1'

        CACC_Center_B.r = CACC_Center_DW.UnitDelay1_DSTATE;
      }

      // End of Switch: '<S4>/Switch'
      // End of Outputs for SubSystem: '<S5>/Enabled Subsystem'
    }

    if (rtmIsMajorTimeStep((&CACC_Center_M)) &&
        (&CACC_Center_M)->Timing.TaskCounters.TID[2] == 0) {
      // If: '<S11>/If'
      if (rtsiIsModeUpdateTimeStep(&(&CACC_Center_M)->solverInfo)) {
        if ((CACC_Center_B.curv < 0.01) && (CACC_Center_B.curv > -0.01)) {
          // Outputs for IfAction SubSystem: '<S11>/Approximation of s around zero' incorporates:
          //   ActionPort: '<S23>/Action Port'

          // Merge: '<S11>/Merge' incorporates:
          //   Fcn: '<S23>/Taylor approximation 3rd order'

          CACC_Center_B.Merge = rt_powd_snf(CACC_Center_B.h * CACC_Center_B.v +
            CACC_Center_B.r, 2.0) * 0.5 * CACC_Center_B.curv - rt_powd_snf
            (CACC_Center_B.h * CACC_Center_B.v + CACC_Center_B.r, 4.0) * 0.125 *
            rt_powd_snf(CACC_Center_B.curv, 3.0);

          // End of Outputs for SubSystem: '<S11>/Approximation of s around zero' 
        } else {
          // Outputs for IfAction SubSystem: '<S11>/function s' incorporates:
          //   ActionPort: '<S24>/Action Port'

          // Merge: '<S11>/Merge' incorporates:
          //   Fcn: '<S24>/function s'

          CACC_Center_B.Merge = (sqrt(rt_powd_snf(CACC_Center_B.h *
            CACC_Center_B.v + CACC_Center_B.r, 2.0) * rt_powd_snf
            (CACC_Center_B.curv, 2.0) + 1.0) - 1.0) / CACC_Center_B.curv;

          // End of Outputs for SubSystem: '<S11>/function s'
        }
      }

      // End of If: '<S11>/If'
    }

    if (rtmIsMajorTimeStep((&CACC_Center_M)) &&
        (&CACC_Center_M)->Timing.TaskCounters.TID[1] == 0) {
      // Fcn: '<S7>/Fcn'
      CACC_Center_B.Fcn = atan((CACC_Center_B.h * CACC_Center_B.v +
        CACC_Center_B.r) * CACC_Center_B.curv);

      // Fcn: '<S4>/z1' incorporates:
      //   Fcn: '<S4>/z2'
      //   Fcn: '<S9>/Fcn2'
      //   Fcn: '<S9>/Fcn3'

      CACC_Center_B.Fcn3 = sin(CACC_Center_B.In1_a.theta);
      CACC_Center_B.Gain = CACC_Center_B.h * CACC_Center_B.v + CACC_Center_B.r;

      // Fcn: '<S4>/z1'
      CACC_Center_B.z1 = ((CACC_Center_B.In1_a.x -
                           CACC_Center_B.In1_g.pose.position.x) +
                          CACC_Center_B.Merge * CACC_Center_B.Fcn3) -
        CACC_Center_B.Gain * cos(CACC_Center_B.CoordinateTransformationConvers[0]);

      // Fcn: '<S4>/z2'
      CACC_Center_B.z2 = ((CACC_Center_B.In1_a.y -
                           CACC_Center_B.In1_g.pose.position.y) -
                          CACC_Center_B.Merge * cos(CACC_Center_B.In1_a.theta))
        - CACC_Center_B.Gain * sin
        (CACC_Center_B.CoordinateTransformationConvers[0]);

      // Fcn: '<S4>/z3' incorporates:
      //   Fcn: '<S4>/z4'
      //   Fcn: '<S9>/Fcn2'
      //   Fcn: '<S9>/Fcn3'

      CACC_Center_B.Gain = CACC_Center_B.CoordinateTransformationConvers[0] +
        CACC_Center_B.Fcn;
      CACC_Center_B.y_idx_0 = cos(CACC_Center_B.In1_a.theta);

      // Fcn: '<S4>/z3'
      CACC_Center_B.z3 = CACC_Center_B.In1_a.v * CACC_Center_B.y_idx_0 - cos
        (CACC_Center_B.Gain) * CACC_Center_B.v;

      // Fcn: '<S4>/z4' incorporates:
      //   Fcn: '<S4>/z1'

      CACC_Center_B.z4 = CACC_Center_B.In1_a.v * CACC_Center_B.Fcn3 - sin
        (CACC_Center_B.Gain) * CACC_Center_B.v;

      // Gain: '<S9>/Gain'
      CACC_Center_B.Gain = CACC_Center_P.k1 * CACC_Center_B.z1;

      // Gain: '<S9>/Gain1' incorporates:
      //   UnitDelay: '<S4>/Unit Delay1'

      CACC_Center_DW.UnitDelay1_DSTATE = CACC_Center_P.k2 * CACC_Center_B.z2;

      // Fcn: '<S9>/Fcn' incorporates:
      //   Fcn: '<S9>/Fcn2'

      CACC_Center_B.Fcn2 = sin(CACC_Center_B.Fcn);
      CACC_Center_B.y_idx_1 = cos(CACC_Center_B.Fcn);
      CACC_Center_B.Fcn_k = CACC_Center_B.y_idx_1 * CACC_Center_B.z3 +
        CACC_Center_B.Fcn2 * CACC_Center_B.z4;

      // Fcn: '<S9>/Fcn1' incorporates:
      //   Fcn: '<S9>/Fcn'

      CACC_Center_B.Fcn1 = CACC_Center_B.Fcn2 * CACC_Center_B.z3 +
        CACC_Center_B.y_idx_1 * CACC_Center_B.z4;

      // Fcn: '<S9>/Fcn2' incorporates:
      //   Fcn: '<S9>/Fcn3'

      CACC_Center_B.y_idx_1 = (1.0 - CACC_Center_B.y_idx_1) *
        CACC_Center_B.In1_a.v;
      CACC_Center_B.y_idx_2 = CACC_Center_B.Fcn2 * CACC_Center_B.In1_a.v;
      CACC_Center_B.Fcn2 = CACC_Center_B.y_idx_1 * CACC_Center_B.y_idx_0 -
        CACC_Center_B.y_idx_2 * CACC_Center_B.Fcn3;

      // Fcn: '<S9>/Fcn3'
      CACC_Center_B.Fcn3 = CACC_Center_B.y_idx_1 * CACC_Center_B.Fcn3 +
        CACC_Center_B.y_idx_2 * CACC_Center_B.y_idx_0;
    }

    if (rtmIsMajorTimeStep((&CACC_Center_M)) &&
        (&CACC_Center_M)->Timing.TaskCounters.TID[2] == 0) {
      // Fcn: '<S9>/Fcn4'
      CACC_Center_B.Fcn4_o = cos(CACC_Center_B.In1_a.theta) *
        CACC_Center_B.Merge * CACC_Center_B.In1_a.thetadot;

      // Fcn: '<S9>/Fcn5'
      CACC_Center_B.Fcn5_e = sin(CACC_Center_B.In1_a.theta) *
        CACC_Center_B.Merge * CACC_Center_B.In1_a.thetadot;
    }

    if (rtmIsMajorTimeStep((&CACC_Center_M)) &&
        (&CACC_Center_M)->Timing.TaskCounters.TID[1] == 0) {
      // Sum: '<S9>/Add' incorporates:
      //   UnitDelay: '<S4>/Unit Delay1'

      CACC_Center_B.Add_a[0] = ((CACC_Center_B.Gain + CACC_Center_B.Fcn_k) +
        CACC_Center_B.Fcn2) + CACC_Center_B.Fcn4_o;
      CACC_Center_B.Add_a[1] = ((CACC_Center_DW.UnitDelay1_DSTATE +
        CACC_Center_B.Fcn1) + CACC_Center_B.Fcn3) + CACC_Center_B.Fcn5_e;
    }

    // Fcn: '<S10>/det(Gam)' incorporates:
    //   Fcn: '<S10>/11'
    //   Fcn: '<S10>/21'
    //   Fcn: '<S10>/22'

    CACC_Center_B.Fcn_k = CACC_Center_B.h * sin(CACC_Center_B.Merge);
    CACC_Center_B.y_idx_1 = CACC_Center_B.h * CACC_Center_B.v;
    CACC_Center_B.Gain = (CACC_Center_B.h - sin(CACC_Center_B.In1_a.theta -
      CACC_Center_B.CoordinateTransformationConvers[0]) * CACC_Center_B.Fcn_k) *
      (CACC_Center_B.y_idx_1 + CACC_Center_B.r);
    if (rtmIsMajorTimeStep((&CACC_Center_M)) &&
        (&CACC_Center_M)->Timing.TaskCounters.TID[1] == 0) {
      // InitialCondition: '<S4>/IC'
      if (CACC_Center_DW.IC_FirstOutputTime) {
        CACC_Center_DW.IC_FirstOutputTime = false;

        // InitialCondition: '<S4>/IC'
        rtb_IC = CACC_Center_P.r;
      } else {
        // InitialCondition: '<S4>/IC'
        rtb_IC = CACC_Center_B.r;
      }

      // End of InitialCondition: '<S4>/IC'

      // InitialCondition: '<S4>/IC1'
      if (CACC_Center_DW.IC1_FirstOutputTime) {
        CACC_Center_DW.IC1_FirstOutputTime = false;

        // InitialCondition: '<S4>/IC1'
        rtb_IC1 = CACC_Center_P.h;
      } else {
        // InitialCondition: '<S4>/IC1'
        rtb_IC1 = CACC_Center_B.h;
      }

      // End of InitialCondition: '<S4>/IC1'
    }

    // End of Outputs for SubSystem: '<S1>/LA'
    if (rtmIsMajorTimeStep((&CACC_Center_M)) &&
        (&CACC_Center_M)->Timing.TaskCounters.TID[1] == 0) {
      // Memory: '<S2>/Memory'
      CACC_Center_B.Memory = CACC_Center_DW.Memory_PreviousInput;

      // Memory: '<S2>/Memory1' incorporates:
      //   UnitDelay: '<S4>/Unit Delay1'

      CACC_Center_DW.UnitDelay1_DSTATE = CACC_Center_DW.Memory1_PreviousInput;

      // Gain: '<S2>/Gain2' incorporates:
      //   UnitDelay: '<S4>/Unit Delay1'

      CACC_Center_B.Gain2 = CACC_Center_P.Gain2_Gain *
        CACC_Center_DW.UnitDelay1_DSTATE;
    }

    // Switch: '<S2>/Switch2'
    if (CACC_Center_B.In1_a.v > CACC_Center_P.v_threshold) {
      // Outputs for Atomic SubSystem: '<S1>/LA'
      // Switch: '<S2>/Switch2' incorporates:
      //   Fcn: '<S10>/21'
      //   Fcn: '<S10>/22'
      //   Product: '<S10>/Divide2'
      //   Product: '<S10>/Divide3'
      //   Product: '<S4>/Product2'
      //   Product: '<S4>/Product3'
      //   Sum: '<S4>/Add1'

      CACC_Center_B.Switch2 = (-CACC_Center_B.h * sin
        (CACC_Center_B.CoordinateTransformationConvers[0]) - CACC_Center_B.Fcn_k
        * cos(CACC_Center_B.In1_a.theta)) / CACC_Center_B.Gain *
        CACC_Center_B.Add_a[0] + (CACC_Center_B.h * cos
        (CACC_Center_B.CoordinateTransformationConvers[0]) - CACC_Center_B.Fcn_k
        * sin(CACC_Center_B.In1_a.theta)) / CACC_Center_B.Gain *
        CACC_Center_B.Add_a[1];

      // End of Outputs for SubSystem: '<S1>/LA'
    } else {
      // Switch: '<S2>/Switch2'
      CACC_Center_B.Switch2 = CACC_Center_B.Memory;
    }

    // End of Switch: '<S2>/Switch2'

    // Switch: '<S2>/Switch1'
    if (CACC_Center_B.In1_a.v > CACC_Center_P.v_threshold) {
      // Switch: '<S2>/Switch1' incorporates:
      //   Integrator: '<S2>/Integrator'

      CACC_Center_B.Switch1 = CACC_Center_X.Integrator_CSTATE;
    } else {
      // Switch: '<S2>/Switch1'
      CACC_Center_B.Switch1 = CACC_Center_B.Gain2;
    }

    // End of Switch: '<S2>/Switch1'

    // BusAssignment: '<S2>/Bus Assignment' incorporates:
    //   Constant: '<S25>/Constant'

    CACC_Center_B.BusAssignment = CACC_Center_P.Constant_Value_i;

    // MinMax: '<S2>/Max' incorporates:
    //   Constant: '<S2>/Constant'

    if ((CACC_Center_P.Constant_Value_f >= CACC_Center_B.Switch1) || rtIsNaN
        (CACC_Center_B.Switch1)) {
      // BusAssignment: '<S2>/Bus Assignment'
      CACC_Center_B.BusAssignment.linear.x = CACC_Center_P.Constant_Value_f;
    } else {
      // BusAssignment: '<S2>/Bus Assignment'
      CACC_Center_B.BusAssignment.linear.x = CACC_Center_B.Switch1;
    }

    // End of MinMax: '<S2>/Max'

    // Switch: '<S2>/Switch3' incorporates:
    //   Constant: '<S2>/Constant1'

    if (CACC_Center_B.Switch1 > CACC_Center_P.v_threshold) {
      CACC_Center_B.y_idx_0 = CACC_Center_B.Switch1;
    } else {
      CACC_Center_B.y_idx_0 = CACC_Center_P.Constant1_Value;
    }

    // BusAssignment: '<S2>/Bus Assignment' incorporates:
    //   Gain: '<S2>/Multiply'
    //   Product: '<S2>/Divide1'
    //   Switch: '<S2>/Switch3'
    //   Trigonometry: '<S2>/Tanh'

    CACC_Center_B.BusAssignment.angular.z = atan(CACC_Center_P.L *
      CACC_Center_B.Switch2 / CACC_Center_B.y_idx_0);

    // MATLABSystem: '<S26>/SinkBlock'
    Pub_CACC_Center_2215.publish(&CACC_Center_B.BusAssignment);

    // Switch: '<S2>/Switch4' incorporates:
    //   Integrator: '<S2>/Integrator'

    if (CACC_Center_X.Integrator_CSTATE > CACC_Center_P.Switch4_Threshold) {
      // Outputs for Atomic SubSystem: '<S1>/LA'
      // Fcn: '<S10>/11' incorporates:
      //   Fcn: '<S10>/12'

      CACC_Center_B.y_idx_0 = CACC_Center_B.h * CACC_Center_B.v +
        CACC_Center_B.r;

      // Sum: '<S4>/Add' incorporates:
      //   Fcn: '<S10>/11'
      //   Fcn: '<S10>/12'
      //   Product: '<S10>/Divide'
      //   Product: '<S10>/Divide1'
      //   Product: '<S4>/Product'
      //   Product: '<S4>/Product1'
      //   Switch: '<S2>/Switch4'

      CACC_Center_B.Switch4 = CACC_Center_B.y_idx_0 * cos
        (CACC_Center_B.CoordinateTransformationConvers[0]) / CACC_Center_B.Gain *
        CACC_Center_B.Add_a[0] + CACC_Center_B.y_idx_0 * sin
        (CACC_Center_B.CoordinateTransformationConvers[0]) / CACC_Center_B.Gain *
        CACC_Center_B.Add_a[1];

      // End of Outputs for SubSystem: '<S1>/LA'
    } else {
      // Outputs for Atomic SubSystem: '<S1>/LA'
      // Fcn: '<S10>/11' incorporates:
      //   Fcn: '<S10>/12'

      CACC_Center_B.Fcn_k = CACC_Center_B.y_idx_1 + CACC_Center_B.r;

      // Sum: '<S4>/Add' incorporates:
      //   Fcn: '<S10>/11'
      //   Fcn: '<S10>/12'
      //   Product: '<S10>/Divide'
      //   Product: '<S10>/Divide1'
      //   Product: '<S4>/Product'
      //   Product: '<S4>/Product1'

      CACC_Center_B.Switch4 = CACC_Center_B.Fcn_k * cos
        (CACC_Center_B.CoordinateTransformationConvers[0]) / CACC_Center_B.Gain *
        CACC_Center_B.Add_a[0] + CACC_Center_B.Fcn_k * sin
        (CACC_Center_B.CoordinateTransformationConvers[0]) / CACC_Center_B.Gain *
        CACC_Center_B.Add_a[1];

      // End of Outputs for SubSystem: '<S1>/LA'

      // MinMax: '<S2>/Max1' incorporates:
      //   Constant: '<S2>/Zero'

      if ((!(CACC_Center_B.Switch4 >= CACC_Center_P.Zero_Value)) && (!rtIsNaN
           (CACC_Center_P.Zero_Value))) {
        // Sum: '<S4>/Add' incorporates:
        //   Switch: '<S2>/Switch4'

        CACC_Center_B.Switch4 = CACC_Center_P.Zero_Value;
      }

      // End of MinMax: '<S2>/Max1'
    }

    // End of Switch: '<S2>/Switch4'
  }

  if (rtmIsMajorTimeStep((&CACC_Center_M))) {
    real_T *lastU;

    // Update for Derivative: '<S3>/Derivative2'
    if (CACC_Center_DW.TimeStampA == (rtInf)) {
      CACC_Center_DW.TimeStampA = (&CACC_Center_M)->Timing.t[0];
      lastU = &CACC_Center_DW.LastUAtTimeA;
    } else if (CACC_Center_DW.TimeStampB == (rtInf)) {
      CACC_Center_DW.TimeStampB = (&CACC_Center_M)->Timing.t[0];
      lastU = &CACC_Center_DW.LastUAtTimeB;
    } else if (CACC_Center_DW.TimeStampA < CACC_Center_DW.TimeStampB) {
      CACC_Center_DW.TimeStampA = (&CACC_Center_M)->Timing.t[0];
      lastU = &CACC_Center_DW.LastUAtTimeA;
    } else {
      CACC_Center_DW.TimeStampB = (&CACC_Center_M)->Timing.t[0];
      lastU = &CACC_Center_DW.LastUAtTimeB;
    }

    *lastU = CACC_Center_B.In1_a.v;

    // End of Update for Derivative: '<S3>/Derivative2'

    // Update for Atomic SubSystem: '<S1>/LA'
    if (rtmIsMajorTimeStep((&CACC_Center_M)) &&
        (&CACC_Center_M)->Timing.TaskCounters.TID[1] == 0) {
      // Update for UnitDelay: '<S4>/Unit Delay1'
      CACC_Center_DW.UnitDelay1_DSTATE = rtb_IC1;

      // Update for UnitDelay: '<S4>/Unit Delay'
      CACC_Center_DW.UnitDelay_DSTATE = rtb_IC;
    }

    // Update for Derivative: '<S4>/Derivative'
    if (CACC_Center_DW.TimeStampA_k == (rtInf)) {
      CACC_Center_DW.TimeStampA_k = (&CACC_Center_M)->Timing.t[0];
      lastU = &CACC_Center_DW.LastUAtTimeA_h;
    } else if (CACC_Center_DW.TimeStampB_c == (rtInf)) {
      CACC_Center_DW.TimeStampB_c = (&CACC_Center_M)->Timing.t[0];
      lastU = &CACC_Center_DW.LastUAtTimeB_m;
    } else if (CACC_Center_DW.TimeStampA_k < CACC_Center_DW.TimeStampB_c) {
      CACC_Center_DW.TimeStampA_k = (&CACC_Center_M)->Timing.t[0];
      lastU = &CACC_Center_DW.LastUAtTimeA_h;
    } else {
      CACC_Center_DW.TimeStampB_c = (&CACC_Center_M)->Timing.t[0];
      lastU = &CACC_Center_DW.LastUAtTimeB_m;
    }

    *lastU = CACC_Center_B.curv;

    // End of Update for Derivative: '<S4>/Derivative'
    // End of Update for SubSystem: '<S1>/LA'
    if (rtmIsMajorTimeStep((&CACC_Center_M)) &&
        (&CACC_Center_M)->Timing.TaskCounters.TID[1] == 0) {
      // Update for Memory: '<S2>/Memory'
      CACC_Center_DW.Memory_PreviousInput = CACC_Center_B.Switch2;

      // Update for Memory: '<S2>/Memory1'
      CACC_Center_DW.Memory1_PreviousInput = CACC_Center_B.Switch1;
    }
  }                                    // end MajorTimeStep

  if (rtmIsMajorTimeStep((&CACC_Center_M))) {
    rt_ertODEUpdateContinuousStates(&(&CACC_Center_M)->solverInfo);

    // Update absolute time for base rate
    // The "clockTick0" counts the number of times the code of this task has
    //  been executed. The absolute time is the multiplication of "clockTick0"
    //  and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
    //  overflow during the application lifespan selected.

    ++(&CACC_Center_M)->Timing.clockTick0;
    (&CACC_Center_M)->Timing.t[0] = rtsiGetSolverStopTime(&(&CACC_Center_M)
      ->solverInfo);

    {
      // Update absolute timer for sample time: [0.05s, 0.0s]
      // The "clockTick1" counts the number of times the code of this task has
      //  been executed. The resolution of this integer timer is 0.05, which is the step size
      //  of the task. Size of "clockTick1" ensures timer will not overflow during the
      //  application lifespan selected.

      (&CACC_Center_M)->Timing.clockTick1++;
    }

    rate_scheduler((&CACC_Center_M));
  }                                    // end MajorTimeStep
}

// Derivatives for root system: '<Root>'
void CACC_Center::CACC_Center_derivatives()
{
  XDot_CACC_Center_T *_rtXdot;
  _rtXdot = ((XDot_CACC_Center_T *) (&CACC_Center_M)->derivs);

  // Derivatives for Integrator: '<S2>/Integrator'
  _rtXdot->Integrator_CSTATE = CACC_Center_B.Switch4;
}

// Model initialize function
void CACC_Center::initialize()
{
  // Registration code

  // initialize non-finites
  rt_InitInfAndNaN(sizeof(real_T));

  {
    // Setup solver object
    rtsiSetSimTimeStepPtr(&(&CACC_Center_M)->solverInfo, &(&CACC_Center_M)
                          ->Timing.simTimeStep);
    rtsiSetTPtr(&(&CACC_Center_M)->solverInfo, &rtmGetTPtr((&CACC_Center_M)));
    rtsiSetStepSizePtr(&(&CACC_Center_M)->solverInfo, &(&CACC_Center_M)
                       ->Timing.stepSize0);
    rtsiSetdXPtr(&(&CACC_Center_M)->solverInfo, &(&CACC_Center_M)->derivs);
    rtsiSetContStatesPtr(&(&CACC_Center_M)->solverInfo, (real_T **)
                         &(&CACC_Center_M)->contStates);
    rtsiSetNumContStatesPtr(&(&CACC_Center_M)->solverInfo, &(&CACC_Center_M)
      ->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&(&CACC_Center_M)->solverInfo,
      &(&CACC_Center_M)->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&(&CACC_Center_M)->solverInfo,
      &(&CACC_Center_M)->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&(&CACC_Center_M)->solverInfo,
      &(&CACC_Center_M)->periodicContStateRanges);
    rtsiSetErrorStatusPtr(&(&CACC_Center_M)->solverInfo, (&rtmGetErrorStatus
      ((&CACC_Center_M))));
    rtsiSetRTModelPtr(&(&CACC_Center_M)->solverInfo, (&CACC_Center_M));
  }

  rtsiSetSimTimeStep(&(&CACC_Center_M)->solverInfo, MAJOR_TIME_STEP);
  (&CACC_Center_M)->intgData.y = (&CACC_Center_M)->odeY;
  (&CACC_Center_M)->intgData.f[0] = (&CACC_Center_M)->odeF[0];
  (&CACC_Center_M)->intgData.f[1] = (&CACC_Center_M)->odeF[1];
  (&CACC_Center_M)->intgData.f[2] = (&CACC_Center_M)->odeF[2];
  (&CACC_Center_M)->contStates = ((X_CACC_Center_T *) &CACC_Center_X);
  rtsiSetSolverData(&(&CACC_Center_M)->solverInfo, static_cast<void *>
                    (&(&CACC_Center_M)->intgData));
  rtsiSetIsMinorTimeStepWithModeChange(&(&CACC_Center_M)->solverInfo, false);
  rtsiSetSolverName(&(&CACC_Center_M)->solverInfo,"ode3");
  rtmSetTPtr((&CACC_Center_M), &(&CACC_Center_M)->Timing.tArray[0]);
  (&CACC_Center_M)->Timing.stepSize0 = 0.05;

  // InitializeConditions for Derivative: '<S3>/Derivative2'
  CACC_Center_DW.TimeStampA = (rtInf);
  CACC_Center_DW.TimeStampB = (rtInf);

  // InitializeConditions for Memory: '<S2>/Memory'
  CACC_Center_DW.Memory_PreviousInput = CACC_Center_P.Memory_InitialCondition;

  // InitializeConditions for Integrator: '<S2>/Integrator'
  CACC_Center_X.Integrator_CSTATE = CACC_Center_P.Integrator_IC;

  // InitializeConditions for Memory: '<S2>/Memory1'
  CACC_Center_DW.Memory1_PreviousInput = CACC_Center_P.Memory1_InitialCondition;

  // SystemInitialize for Enabled SubSystem: '<S27>/Enabled Subsystem'
  // SystemInitialize for SignalConversion generated from: '<S29>/In1' incorporates:
  //   Outport: '<S29>/Out1'

  CACC_Center_B.In1_g = CACC_Center_P.Out1_Y0_o;

  // End of SystemInitialize for SubSystem: '<S27>/Enabled Subsystem'

  // SystemInitialize for Enabled SubSystem: '<S31>/Enabled Subsystem'
  // SystemInitialize for SignalConversion generated from: '<S32>/In1' incorporates:
  //   Outport: '<S32>/Out1'

  CACC_Center_B.In1_a = CACC_Center_P.Out1_Y0_k;

  // End of SystemInitialize for SubSystem: '<S31>/Enabled Subsystem'

  // SystemInitialize for Enabled SubSystem: '<S28>/Enabled Subsystem'
  // SystemInitialize for SignalConversion generated from: '<S30>/In1' incorporates:
  //   Outport: '<S30>/Out1'

  CACC_Center_B.In1 = CACC_Center_P.Out1_Y0;

  // End of SystemInitialize for SubSystem: '<S28>/Enabled Subsystem'

  // SystemInitialize for Atomic SubSystem: '<S1>/LA'
  // Start for InitialCondition: '<S4>/IC'
  CACC_Center_DW.IC_FirstOutputTime = true;

  // Start for InitialCondition: '<S4>/IC1'
  CACC_Center_DW.IC1_FirstOutputTime = true;

  // InitializeConditions for UnitDelay: '<S4>/Unit Delay1'
  CACC_Center_DW.UnitDelay1_DSTATE = CACC_Center_P.UnitDelay1_InitialCondition;

  // InitializeConditions for UnitDelay: '<S4>/Unit Delay'
  CACC_Center_DW.UnitDelay_DSTATE = CACC_Center_P.UnitDelay_InitialCondition;

  // InitializeConditions for Derivative: '<S4>/Derivative'
  CACC_Center_DW.TimeStampA_k = (rtInf);
  CACC_Center_DW.TimeStampB_c = (rtInf);

  // SystemInitialize for Enabled SubSystem: '<S6>/Enabled Subsystem'
  // SystemInitialize for SignalConversion generated from: '<S13>/In1' incorporates:
  //   Outport: '<S13>/Out1'

  CACC_Center_B.In1_m = CACC_Center_P.Out1_Y0_a;

  // End of SystemInitialize for SubSystem: '<S6>/Enabled Subsystem'

  // SystemInitialize for Enabled SubSystem: '<S5>/Enabled Subsystem'
  // SystemInitialize for SignalConversion generated from: '<S12>/In1' incorporates:
  //   Outport: '<S12>/Out1'

  CACC_Center_B.In1_h = CACC_Center_P.Out1_Y0_b;

  // End of SystemInitialize for SubSystem: '<S5>/Enabled Subsystem'

  // Start for MATLABSystem: '<S6>/SourceBlock'
  CACC_Center_DW.obj_n.isInitialized = 0;
  CACC_Center_DW.obj_n.matlabCodegenIsDeleted = false;
  CACC_Center_SystemCore_setup_d(&CACC_Center_DW.obj_n);

  // Start for MATLABSystem: '<S5>/SourceBlock'
  CACC_Center_DW.obj_p.isInitialized = 0;
  CACC_Center_DW.obj_p.matlabCodegenIsDeleted = false;
  CACC_Center_SystemCore_setup(&CACC_Center_DW.obj_p);

  // End of SystemInitialize for SubSystem: '<S1>/LA'

  // Start for MATLABSystem: '<S27>/SourceBlock'
  CACC_Center_DW.obj_f.isInitialized = 0;
  CACC_Center_DW.obj_f.matlabCodegenIsDeleted = false;
  CACC_Cente_SystemCore_setup_dfd(&CACC_Center_DW.obj_f);

  // Start for MATLABSystem: '<S31>/SourceBlock'
  CACC_Center_DW.obj.isInitialized = 0;
  CACC_Center_DW.obj.matlabCodegenIsDeleted = false;
  CACC_Cen_SystemCore_setup_dfdzt(&CACC_Center_DW.obj);

  // Start for MATLABSystem: '<S28>/SourceBlock'
  CACC_Center_DW.obj_k.isInitialized = 0;
  CACC_Center_DW.obj_k.matlabCodegenIsDeleted = false;
  CACC_Cent_SystemCore_setup_dfdz(&CACC_Center_DW.obj_k);

  // Start for MATLABSystem: '<S26>/SinkBlock'
  CACC_Center_DW.obj_c.isInitialized = 0;
  CACC_Center_DW.obj_c.matlabCodegenIsDeleted = false;
  CACC_Center_SystemCore_setup_df(&CACC_Center_DW.obj_c);
}

// Model terminate function
void CACC_Center::terminate()
{
  // Terminate for MATLABSystem: '<S27>/SourceBlock'
  if (!CACC_Center_DW.obj_f.matlabCodegenIsDeleted) {
    CACC_Center_DW.obj_f.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<S27>/SourceBlock'

  // Terminate for MATLABSystem: '<S31>/SourceBlock'
  if (!CACC_Center_DW.obj.matlabCodegenIsDeleted) {
    CACC_Center_DW.obj.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<S31>/SourceBlock'

  // Terminate for MATLABSystem: '<S28>/SourceBlock'
  if (!CACC_Center_DW.obj_k.matlabCodegenIsDeleted) {
    CACC_Center_DW.obj_k.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<S28>/SourceBlock'

  // Terminate for Atomic SubSystem: '<S1>/LA'
  // Terminate for MATLABSystem: '<S6>/SourceBlock'
  if (!CACC_Center_DW.obj_n.matlabCodegenIsDeleted) {
    CACC_Center_DW.obj_n.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<S6>/SourceBlock'

  // Terminate for MATLABSystem: '<S5>/SourceBlock'
  if (!CACC_Center_DW.obj_p.matlabCodegenIsDeleted) {
    CACC_Center_DW.obj_p.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<S5>/SourceBlock'
  // End of Terminate for SubSystem: '<S1>/LA'

  // Terminate for MATLABSystem: '<S26>/SinkBlock'
  if (!CACC_Center_DW.obj_c.matlabCodegenIsDeleted) {
    CACC_Center_DW.obj_c.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<S26>/SinkBlock'
}

// Constructor
CACC_Center::CACC_Center() :
  CACC_Center_B(),
  CACC_Center_DW(),
  CACC_Center_X(),
  CACC_Center_M()
{
  // Currently there is no constructor body generated.
}

// Destructor
CACC_Center::~CACC_Center()
{
  // Currently there is no destructor body generated.
}

// Real-Time Model get method
RT_MODEL_CACC_Center_T * CACC_Center::getRTM()
{
  return (&CACC_Center_M);
}

//
// File trailer for generated code.
//
// [EOF]
//
