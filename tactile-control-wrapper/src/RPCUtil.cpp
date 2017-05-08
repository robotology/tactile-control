#include "tactileControlWrapper/RPCUtil.h"

#include <stdexcept>

using tactileControlWrapper::RPCUtil;
using tactileControlWrapper::RPCData;
using tactileControlWrapper::RPCMainCmdName;
using tactileControlWrapper::RPCTaskCmdArgName;
using tactileControlWrapper::RPCViewCmdArgName;
using tactileControlWrapper::TaskName;

using yarp::os::Bottle;

using std::string;
using std::pair;
using std::endl;

RPCUtil::RPCUtil(){}

void RPCUtil::init(RPCData *rpcData){

    this->rpcData = rpcData;

    dbgTag = "RPCUtil: ";
}

bool RPCUtil::processCommand(const Bottle &rpcCmdBottle){

	errMsg.clear();

    try {
        mainCmd = rpcData->mainCmdRevMap.at(rpcCmdBottle.get(0).asString());
    } catch(const std::out_of_range& oor){
		errMsg << "Wrong command" << endl << rpcData->showHelp();
        return false;
    }

    switch (mainCmd){

    case HELP: // do nothing
        break;
	case SET:
		if (rpcCmdBottle.size() < 3){
			wrongSyntaxMessage(SET);
			return false;
		}
		paramName = rpcCmdBottle.get(1).asString();
		argValue = rpcCmdBottle.get(2);
		break;
	case GET:
		if (rpcCmdBottle.size() < 2){
			wrongSyntaxMessage(GET);
			return false;
		}
		paramName = rpcCmdBottle.get(1).asString();
		break;
	case TASK:
		if (!processTaskCommand(rpcCmdBottle)){
			return false;
		}
        break;
    case SHOW:
        if (rpcCmdBottle.size() < 2){
			wrongSyntaxMessage(SHOW);
			return false;
        }
        try {
            viewCmdArg = rpcData->viewCmdArgRevMap[rpcCmdBottle.get(1).asString()];
        } catch(const std::out_of_range& oor){
			wrongSyntaxMessage(SHOW);
			return false;
        }    
		break;
    case START: // do nothing
        break;
    case OPEN:
        if (rpcCmdBottle.size() < 2 ){
			wrongSyntaxMessage(OPEN);
			return false;
        }
		argValue = rpcCmdBottle.get(1);
		if (!argValue.isBool()){
			wrongSyntaxMessage(OPEN);
			return false;
		}
		if (rpcCmdBottle.size() > 2){
			waitValue = rpcCmdBottle.get(2);
			if (!waitValue.isBool()){
				wrongSyntaxMessage(OPEN);
				return false;
			}
		}
		else {
			waitValue == yarp::os::Value::getNullValue();
		}
        break;
    case GRASP:
		if (rpcCmdBottle.size() > 1){
			waitValue = rpcCmdBottle.get(1);
			if (!waitValue.isBool()){
				wrongSyntaxMessage(GRASP);
				return false;
			}
		}
		else {
			waitValue == yarp::os::Value::getNullValue();
		}
        break;
	case IS_HAND_OPEN: // do nothing
		break;
	case IS_HAND_CLOSE: // do nothing
		break;
	case SET_GRIP_STRENGTH:
		if (rpcCmdBottle.size() < 2){
			wrongSyntaxMessage(SET_GRIP_STRENGTH);
			return false;
		}
		argValue = rpcCmdBottle.get(1);
		if (!argValue.isDouble() && !argValue.isInt()){
			wrongSyntaxMessage(SET_GRIP_STRENGTH);
			return false;
		}
		break;
	case SET_MIN_FORCE:
		if (rpcCmdBottle.size() < 2){
			wrongSyntaxMessage(SET_MIN_FORCE);
			return false;
		}
		argValue = rpcCmdBottle.get(1);
		if (!argValue.isDouble() && !argValue.isInt()){
			wrongSyntaxMessage(SET_MIN_FORCE);
			return false;
		}
		break;
	case DISABLE_MIN_FORCE: // do nothing
		break;
	case QUIT: // do nothing
		break;

    };

    return true;
}

bool RPCUtil::processTaskCommand(const Bottle &rpcCmdBottle){

    try {
        taskCmdArg = rpcData->taskCmdArgRevMap[rpcCmdBottle.get(1).asString()];
    } catch(const std::out_of_range& oor){
		wrongSyntaxMessage(TASK);
        return false;
    }
        
    switch (taskCmdArg){

    case ADD:
        if (rpcCmdBottle.size() < 3){
			wrongSyntaxMessage(TASK);
            return false;
        }
        try {
            task = rpcData->taskRevMap[rpcCmdBottle.get(2).asString()];
        } catch(const std::out_of_range& oor){
			errMsg << "wrong task name, available: step (STEP task), appr (APPROACH task), ctrl (CONTROL task)" << std::endl;
            return false;
		}
		if (rpcCmdBottle.size() > 3){
			argValue = rpcCmdBottle.get(3);
		} else {
			if (task == STEP){
				errMsg << "provide target values for the STEP task" << std::endl;
				return false;
			}
			argValue = yarp::os::Value::getNullValue();
		}
        break;
    case CLEAR: // do nothing
        break;
    }

    return true;
}

void RPCUtil::wrongSyntaxMessage(tactileControlWrapper::RPCMainCmdName mainCmd){

	errMsg << "Wrong syntax" << endl << rpcData->showCommandHelp(mainCmd);
}