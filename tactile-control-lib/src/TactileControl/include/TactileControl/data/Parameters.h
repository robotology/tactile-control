#ifndef TACTILECONTROL_PARAMETERS_H
#define TACTILECONTROL_PARAMETERS_H

/*** Parameters common to all the tasks ***/
#define PAR_COMMON_TASK_THREAD_PERIOD "taskThreadPeriod"
#define PAR_COMMON_DATA_COLLECTION_THREAD_PERIOD "dataCollectionThreadperiod"
#define PAR_COMMON_PORT_PREFIX "portPrefix"
#define PAR_COMMON_HAND "hand"
#define PAR_COMMON_ICUB "icub"
#define PAR_COMMON_CONTROLLED_JOINTS "controlledJoints"
#define PAR_COMMON_PWM_SIGN "pwmSign"
#define PAR_COMMON_FINGER_SENSITIVITY "fingertipSensitivity"
#define PAR_COMMON_OPEN_HAND_JOINTS "openHandJoints"
#define PAR_COMMON_DISABLE_PWM "disablePwm"
#define PAR_COMMON_TACT_MEDIAN_WINDOW_SIZE "tactileMedianWindowSize"
#define PAR_COMMON_REF_VELOCITY "refVelocity"
#define PAR_COMMON_USING_TWO_HANDS "usingTwoHands"
#define PAR_COMMON_EXPERIMENT_INFO "experimentInfo"
#define PAR_COMMON_EXPERIMENT_OPTIONAL_INFO "experimentOptionalInfo"
#define PAR_COMMON_USE_TACTILE_WEIGHTED_SUM "useTactileWeightedSum"
#define PAR_COMMON_ENABLE_SCREEN_LOGGING "enableScreenLogging"
#define PAR_COMMON_SCREEN_LOGGING_RATE "screenLoggingRate"

/*** Parameters related to the step task ***/
#define PAR_STEP_DURATION "step.duration"

/*** Parameters related to the ramp task ***/
#define PAR_RAMP_DURATION "ramp.duration"
#define PAR_RAMP_DURATION_AFTER_STABILIZATION "ramp.durationAfterStabilization"
#define PAR_RAMP_SLOPE "ramp.slope"
#define PAR_RAMP_INTERCEPT "ramp.intercept"

/*** Parameters related to the approach task ***/
#define PAR_APPR_DURATION "approach.duration"
#define PAR_APPR_VELOCITY "approach.velocities"
#define PAR_APPR_MAX_PWM "approach.maxPwm"
#define PAR_APPR_PWM_LIMIT_ENABLED "approach.pwmLimitEnabled"
#define PAR_APPR_WINDOW_SIZE "approach.windowSize"
#define PAR_APPR_THRESHOLD "approach.threshold"
#define PAR_APPR_TIMEOUT "approach.timeout"

/*** Parameters related to the contol task ***/
#define PAR_CTRL_DURATION "control.duration"
#define PAR_CTRL_DEFAULT_FORCE_TARGET "control.defaultForceTarget"
#define PAR_CTRL_LOW_PID_KP "control.pidKp"
#define PAR_CTRL_LOW_PID_KI "control.pidKi"
#define PAR_CTRL_LOW_PID_WP "control.pidWp"
#define PAR_CTRL_LOW_PID_WI "control.pidWi"
#define PAR_CTRL_LOW_PID_WD "control.pidWd"
#define PAR_CTRL_LOW_PID_N "control.pidN"
#define PAR_CTRL_LOW_PID_WIND_UP_COEFF "control.pidWindUpCoeff"
#define PAR_CTRL_LOW_PID_MIN_SAT_LIM "control.pidMinSatLim"
#define PAR_CTRL_LOW_PID_MAX_SAT_LIM "control.pidMaxSatLim"
#define PAR_CTRL_LOW_PID_SCALE "control.pidScale"
#define PAR_CTRL_LOW_PID_INTEGRAL_DISABLED "control.pidIntegralDisabled"
// Parameters related to the supervisor
#define PAR_CTRL_SUPERVISOR_ENABLED "control.high.enabled"
#define PAR_CTRL_HIGH_PID_KP "control.high.pidKp"
#define PAR_CTRL_HIGH_PID_KI "control.high.pidKi"
#define PAR_CTRL_HIGH_PID_KD "control.high.pidKd"
#define PAR_CTRL_HIGH_PID_WP "control.high.pidWp"
#define PAR_CTRL_HIGH_PID_WI "control.high.pidWi"
#define PAR_CTRL_HIGH_PID_WD "control.high.pidWd"
#define PAR_CTRL_HIGH_PID_N "control.high.pidN"
#define PAR_CTRL_HIGH_PID_WIND_UP_COEFF "control.high.pidWindUpCoeff"
#define PAR_CTRL_HIGH_PID_MIN_SAT_LIM "control.high.pidMinSatLim"
#define PAR_CTRL_HIGH_PID_MAX_SAT_LIM "control.high.pidMaxSatLim"
#define PAR_CTRL_HIGH_PID_SCALE "control.high.pidScale" // 5
#define PAR_CTRL_GRIP_STRENGTH "control.high.gripStrength" // 7
#define PAR_CTRL_THUMB_ABDUCTION_OFFSET "control.high.thumbAbductionOffset" //75
#define PAR_CTRL_MIN_JERK_TRACK_ENABLED "control.high.minJerkTrackingEnabled" // 29
#define PAR_CTRL_MIN_JERK_TRACK_REF_TIME "control.high.minJerkTrackingRefTime" // 30
#define PAR_CTRL_TARGET_OBJECT_POSITION "control.high.targetObjectPosition"
#define PAR_CTRL_SUPERVISOR_MODE "control.high.supervisorMode"
#define PAR_CTRL_GMM_BEST_POSE_LOG_ONE_SHOT "control.high.gmmBestPoseLogOneShot"
#define PAR_CTRL_GMM_JOINTS_REGRESSION_ENABLED "control.high.gmmJointsRegressionEnabled"
#define PAR_CTRL_GMM_JOINTS_MIN_JERK_TRACK_ENABLED "control.high.gmmJointsMinJerkTrackingEnabled" // 65
#define PAR_CTRL_GMM_JOINTS_MIN_JERK_TRACK_REF_TIME "control.high.gmmJointsMinJerkTrackingRefTime" // 66
#define PAR_CTRL_MIN_FORCE_ENABLED "control.high.minForceEnabled" // 77
#define PAR_CTRL_MIN_FORCE "control.high.minForce" // 78 - 80


/*** Parameters not configurable online ***/

#define NUM_FINGERS 5
#define NUM_TAXELS 12
#define NUM_HAND_ENCODERS 16
#define NUM_HAND_JOINTS 8
#define FIRST_HAND_JOINT 8
#define LAST_HAND_JOINT 15
#define STRAIGHT_DISTAL_ANGLE 4.0

#define THUMB_ABDUCTION_JOINT 8
#define THUMB_PROXIMAL_JOINT 9
#define THUMB_DISTAL_JOINT 10
#define INDEX_PROXIMAL_JOINT 11
#define INDEX_DISTAL_JOINT 12
#define MIDDLE_PROXIMAL_JOINT 13
#define MIDDLE_DISTAL_JOINT 14
#define RING_LITTLE_JOINT 15

#define INDEX_FINGERTIP 0
#define MIDDLE_FINGERTIP 1
#define RING_FINGERTIP 2
#define LITTLE_FINGERTIP 3
#define THUMB_FINGERTIP 4

#define TIME_TO_STABILIZE_GRASP 4 // seconds

#endif // TACTILECONTROL_PARAMETERS_H