#ifndef __PLANTIDENTIFICATION_ENUMS_H__
#define __PLANTIDENTIFICATION_ENUMS_H__

namespace iCub {
    namespace plantIdentification {
     
		enum RPCMainCmdName {

			HELP,
			SET,
			TASK,
			VIEW,
			START,
			STOP,
			QUIT
		};

		enum RPCSetCmdArgName {

			FINGER_TO_MOVE,
			JOINT_TO_MOVE,
			PWM_SIGN,

			STEP_LIFESPAN,
			
			CTRL_PID_KPF,
			CTRL_PID_KIF,
			CTRL_PID_KDF,
			CTRL_PID_KPB,
			CTRL_PID_KIB,
			CTRL_PID_KDB,
			CTRL_OP_MODE,
			CTRL_LIFESPAN,

			RAMP_SLOPE,
			RAMP_INTERCEPT,
			RAMP_LIFESPAN,
			RAMP_LIFESPAN_AFTER_STAB
		};
    

		enum RPCTaskCmdArgName {

			ADD,
			EMPTY,
			POP
		};

		enum TaskName {

			STEP,
			CONTROL,
			RAMP,
		};

		enum RPCViewCmdArgName {

			SETTINGS,
			TASKS
		};

		enum FingerJoint {

			PROXIMAL,
			DISTAL
		};

		enum RampTaskState {

			DECREASING,
			STEADY
		};

		enum ControlTaskOpMode {

			GAINS_SET_POS_ERR = 0,
			GAINS_SET_NEG_ERR = 1,
			BOTH_GAINS_SETS = 2
		};

	}
}

#endif
