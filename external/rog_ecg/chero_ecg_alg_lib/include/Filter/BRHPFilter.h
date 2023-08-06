/*
 * File: BRHPFilter.h
 *
 * Code generated for Simulink model 'BRHPFilter'.
 *
 * Model version                  : 1.5
 * Simulink Coder version         : 9.3 (R2020a) 18-Nov-2019
 * C/C++ source code generated on : Tue May 10 09:48:54 2022
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_BRHPFilter_h_
#define RTW_HEADER_BRHPFilter_h_
#ifndef BRHPFilter_COMMON_INCLUDES_
# define BRHPFilter_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* BRHPFilter_COMMON_INCLUDES_ */

#include "BRHPFilter_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T Delay11_DSTATE;               /* '<S1>/Delay11' */
  real_T Delay21_DSTATE;               /* '<S1>/Delay21' */
  real_T Delay12_DSTATE;               /* '<S1>/Delay12' */
  real_T Delay22_DSTATE;               /* '<S1>/Delay22' */
  real_T Delay13_DSTATE;               /* '<S1>/Delay13' */
  real_T Delay23_DSTATE;               /* '<S1>/Delay23' */
  real_T Delay14_DSTATE;               /* '<S1>/Delay14' */
  real_T Delay24_DSTATE;               /* '<S1>/Delay24' */
  real_T Delay15_DSTATE;               /* '<S1>/Delay15' */
  real_T Delay25_DSTATE;               /* '<S1>/Delay25' */
  real_T Delay16_DSTATE;               /* '<S1>/Delay16' */
} DW_BRHPFilter_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T Input;                        /* '<Root>/Input' */
} ExtU_BRHPFilter_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T Output;                       /* '<Root>/Output' */
} ExtY_BRHPFilter_T;

/* Real-time Model Data Structure */
struct tag_RTM_BRHPFilter_T {
  const char_T * volatile errorStatus;
};

/* Block states (default storage) */
extern DW_BRHPFilter_T BRHPFilter_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_BRHPFilter_T BRHPFilter_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_BRHPFilter_T BRHPFilter_Y;

/* Model entry point functions */
extern void BRHPFilter_initialize(void);
extern void BRHPFilter_step(void);
extern void BRHPFilter_terminate(void);

/* Real-time Model object */
extern RT_MODEL_BRHPFilter_T *const BRHPFilter_M;

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
 * '<Root>' : 'BRHPFilter'
 * '<S1>'   : 'BRHPFilter/BRHPFilter'
 */
#endif                                 /* RTW_HEADER_BRHPFilter_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
