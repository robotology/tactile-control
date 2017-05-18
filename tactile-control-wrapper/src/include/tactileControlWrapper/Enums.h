#ifndef TACTILECONTROLWRAPPER_ENUMS_H
#define TACTILECONTROLWRAPPER_ENUMS_H

namespace tactileControlWrapper {
     
    enum RPCMainCmdName {

        HELP,
        SET,
        GET,
        TASK,
        SHOW,
        START,
        OPEN,
        ARM,
        GRASP,
        IS_HAND_OPEN,
        IS_HAND_CLOSE,
        SET_GRIP_STRENGTH,
        SET_MIN_FORCE,
        DISABLE_MIN_FORCE,
        QUIT
    };
    
    enum RPCTaskCmdArgName {

        ADD,
        CLEAR
    };

    enum TaskName {

        STEP,
        CONTROL,
        APPROACH
    };

    enum RPCViewCmdArgName {

        SETTINGS,
        TASKS
    };

} // namespace tactileControlWrapper

#endif // TACTILECONTROLWRAPPER_ENUMS_H
