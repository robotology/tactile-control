#ifndef __OBJECTGRASPING_ENUMS_H__
#define __OBJECTGRASPING_ENUMS_H__

namespace iCub {
    namespace objectGrasping {
     
		enum RPCMainCmdName {

            DEMO,
			HELP,
			SET,
			TASK,
			VIEW,
			START,
			STOP,
			QUIT
		};

		enum RPCSetCmdArgName {

			PWM_SIGN,

			STEP_JOINTS_LIST,
			STEP_LIFESPAN,
			
			CTRL_JOINTS_LIST,
			CTRL_PID_KPF,
			CTRL_PID_KIF,
			CTRL_PID_KDF,
			CTRL_PID_KPB,
			CTRL_PID_KIB,
			CTRL_PID_KDB,
			CTRL_OP_MODE,
			CTRL_PID_RESET_ENABLED,
			CTRL_LIFESPAN,

			RAMP_JOINTS_LIST,
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
            NONE
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

		enum TaskState  {

			SET_ARM_IN_START_POSITION,
			WAIT_TO_START,
			SET_ARM_IN_GRASP_POSITION,
			BEGIN_GRASP_THREAD,
			WAIT_FOR_GRASP_THREAD,
			RAISE_ARM,
            WAIT_WITH_ARM_RAISED,
            SET_ARM_BACK_IN_GRASP_POSITION,
            OPEN_HAND,
            SET_ARM_BACK_IN_START_POSITION,
			WAIT_FOR_CLOSURE
		};
	}
}

#endif
