/*
 * File: ECG_2D.h
 *
 * Code generated for Simulink model 'ECG_2D'.
 *
 * Model version                  : 1.1
 * Simulink Coder version         : 8.14 (R2018a) 06-Feb-2018
 * C/C++ source code generated on : Tue Sep 22 09:44:12 2020
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_ECG_2D_h_
#define RTW_HEADER_ECG_2D_h_
#include <stddef.h>
#include <string.h>
#ifndef ECG_2D_COMMON_INCLUDES_
# define ECG_2D_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* ECG_2D_COMMON_INCLUDES_ */

#include "ECG_2D_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real32_T LP_3ord_08Hz_FILT_STATES[4];/* '<Root>/LP_3ord_0.8Hz' */
  real32_T HP_1ord_05Hz_FILT_STATES[2];/* '<Root>/HP_1ord_0.5Hz' */
} DW_ECG_2D_T;

/* Real-time Model Data Structure */
struct tag_RTM_ECG_2D_T {
  const char_T * volatile errorStatus;
};

/* Block states (default storage) */
extern DW_ECG_2D_T ECG_2D_DW;

/*
 * Exported Global Signals
 *
 * Note: Exported global signals are block signals with an exported global
 * storage class designation.  Code generation will declare the memory for
 * these signals and export their symbols.
 *
 */
extern real32_T In_Signal1;            /* '<Root>/In1' */
extern real32_T In_Signal2;            /* '<Root>/In2' */
extern real32_T Out_Signal1;           /* '<Root>/LP_3ord_0.8Hz' */
extern real32_T Out_Signal2;           /* '<Root>/HP_1ord_0.5Hz' */

/* Model entry point functions */
extern void ECG_2D_initialize(void);
extern void ECG_2D_step_1(void);
extern void ECG_2D_step_2(void);
extern void ECG_2D_terminate(void);

/* Real-time Model object */
extern RT_MODEL_ECG_2D_T *const ECG_2D_M;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'ECG_2D'
 */
#endif                                 /* RTW_HEADER_ECG_2D_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
