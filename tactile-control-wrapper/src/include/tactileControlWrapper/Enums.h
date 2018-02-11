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
        OBJECT_RECOGNITION,
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

    enum RPCObjRecCmdArgName {

        LOAD_TRAINING_SET,
        SAVE_TRAINING_SET,
        LOAD_OBJECTS,
        SAVE_OBJECTS,
        LOAD_MODEL,
        SAVE_MODEL,
        VIEW_DATA,
        TRAIN,
        DISCARD_LAST_FEATURES,
        CLEAR_COLLECTED_FEATURES,
        PROCESS_COLLECTED_DATA,
        ADD_NEW_OBJECT,
        GET_READY,
        READ_VISUAL_CLASSIFIER_SCORES,
        RESET
    };

} // namespace tactileControlWrapper

#endif // TACTILECONTROLWRAPPER_ENUMS_H
