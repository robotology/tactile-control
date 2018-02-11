#include "tactileControlWrapper/RPCData.h"

#include <iostream>
#include <sstream>
#include <string.h>
#include <stdlib.h>

using tactileControlWrapper::RPCData;
using tactileControlWrapper::RPCMainCmdName;
using tactileControlWrapper::RPCTaskCmdArgName;
using tactileControlWrapper::RPCViewCmdArgName;
using tactileControlWrapper::TaskName;

using std::string;
using std::pair;


RPCData::RPCData(){

    add("help",HELP,"Shows this help");
    add("set", SET, "Sets a parameter (usage: 'set <paramName> <paramValue>')");
    add("get", GET, "Gets a parameter (usage: 'get <paramName>')");
    add("task", TASK, "Manages tasks (usage: 'task [ clear | add <taskType> <targetValueList> ]')");
    add("show",SHOW,"Shows settings/tasks (usage: 'show [ set | tasks ]')");
    add("start",START,"Start tasks");
    add("open",OPEN,"Opens the hand and stops any running task (usage: open <fullyOpen> <wait>)");
    add("arm",ARM,"Sets the arm in home position");
    add("grasp",GRASP,"Executes the grasp task (usage: grasp <wait>)");
    add("is_hand_open",IS_HAND_OPEN,"checks if the hand is open");
    add("is_hand_close",IS_HAND_CLOSE,"checks if the hand is close and the grasp is stable");
    add("set_grip_strength",SET_GRIP_STRENGTH,"sets the desired grip strength (usage: 'set_grip_strength <value>')");
    add("set_min_force", SET_MIN_FORCE, "sets the minimum force reference at the fingertips (usage: 'set_min_force <value>')");
    add("disable_min_force", DISABLE_MIN_FORCE, "disables the minimum force reference mode");
    add("obj_rec", OBJECT_RECOGNITION, "introduces to several commands related to the object recognition task (usage: 'obj_rec <cmd> [param]')");
    add("quit", QUIT, "Closes the module");
    
    // TASK <?>
    add("add",ADD,"ADD TASK");
    add("clear",CLEAR,"CLEAR TASK LIST");

    // TASK ADD <?>
    add("step",STEP,"STEP TASK");
    add("ctrl",CONTROL,"CONTROL TASK");
    add("appr",APPROACH,"APPROACH TASK");

    // VIEW <?>
    add("set",SETTINGS,"SETTINGS");
    add("tasks",TASKS,"TASKS");

    // OBJ_REC <?>
    add("load_train_set", LOAD_TRAINING_SET, "LOAD_TRAINING_SET");
    add("save_train_set", SAVE_TRAINING_SET, "SAVE_TRAINING_SET");
    add("load_objects", LOAD_OBJECTS, "LOAD_OBJECTS");
    add("save_objects", SAVE_OBJECTS, "SAVE_OBJECTS");
    add("load_model", LOAD_MODEL, "LOAD_MODEL");
    add("save_model", SAVE_MODEL, "SAVE_MODEL");
    add("view_data", VIEW_DATA, "VIEW_DATA");
    add("train", TRAIN, "TRAIN");
    add("discard_last", DISCARD_LAST_FEATURES, "DISCARD_LAST_FEATURES");
    add("clear", CLEAR_COLLECTED_FEATURES, "CLEAR_COLLECTED_FEATURES");
    add("process_data", PROCESS_COLLECTED_DATA, "PROCESS_COLLECTED_DATA");
    add("add_new_object", ADD_NEW_OBJECT, "ADD_NEW_OBJECT");
    add("get_ready", GET_READY, "GET_READY");
    add("read_vc_scores", READ_VISUAL_CLASSIFIER_SCORES, "READ_VISUAL_CLASSIFIER_SCORES");
    add("reset", RESET, "RESET");
    
    dbgTag = "RPCData: ";
    
}

void RPCData::add(string rpcLabel,RPCMainCmdName enumLabel,string description){
    mainCmdMap.insert(std::pair<RPCMainCmdName,string>(enumLabel,rpcLabel));
    mainCmdDescMap.insert(std::pair<RPCMainCmdName,string>(enumLabel,description));
    mainCmdRevMap.insert(std::pair<string,RPCMainCmdName>(rpcLabel,enumLabel));
}

void RPCData::add(string rpcLabel,RPCTaskCmdArgName enumLabel,string description){
    taskCmdArgMap.insert(pair<RPCTaskCmdArgName,string>(enumLabel,rpcLabel));
    taskCmdArgDescMap.insert(pair<RPCTaskCmdArgName,string>(enumLabel,description));
    taskCmdArgRevMap.insert(pair<string,RPCTaskCmdArgName>(rpcLabel,enumLabel));
}

void RPCData::add(string rpcLabel, RPCViewCmdArgName enumLabel, string description){
    viewCmdArgMap.insert(pair<RPCViewCmdArgName, string>(enumLabel, rpcLabel));
    viewCmdArgDescMap.insert(pair<RPCViewCmdArgName, string>(enumLabel, description));
    viewCmdArgRevMap.insert(pair<string, RPCViewCmdArgName>(rpcLabel, enumLabel));
}

void RPCData::add(string rpcLabel, RPCObjRecCmdArgName enumLabel, string description){
    objRecCmdArgMap.insert(pair<RPCObjRecCmdArgName, string>(enumLabel, rpcLabel));
    objRecCmdArgDescMap.insert(pair<RPCObjRecCmdArgName, string>(enumLabel, description));
    objRecCmdArgRevMap.insert(pair<string, RPCObjRecCmdArgName>(rpcLabel, enumLabel));
}

void RPCData::add(string rpcLabel, TaskName enumLabel, string description){
    taskMap.insert(pair<TaskName,string>(enumLabel,rpcLabel));
    taskDescMap.insert(pair<TaskName,string>(enumLabel,description));
    taskRevMap.insert(pair<string,TaskName>(rpcLabel,enumLabel));
}

std::string RPCData::getFullDescription(RPCMainCmdName mainCmdName){

    return "'" + mainCmdMap[mainCmdName] + "' - " + mainCmdDescMap[mainCmdName];
}

bool RPCData::setTargets(const yarp::os::Value &value,std::vector<double> &targets){

    if (value.isNull()){

        targets.resize(0);
        return true;

    } else if (value.isInt() || value.isDouble()){

        targets.resize(1,value.asDouble());
        return true;

    } else if (value.isList()){

        yarp::os::Bottle *targetList = value.asList();
        targets.resize(0);
        for(int i = 0; i < targetList->size(); i++){
            targets.push_back(targetList->get(i).asDouble());
        }
        return true;

    } else {

        return false;
    }
}

std::string RPCData::showCommandHelp(tactileControlWrapper::RPCMainCmdName mainCmdName){

    return mainCmdMap[mainCmdName] + ": " + mainCmdDescMap[mainCmdName] + "\n";

}

std::string RPCData::showHelp(){

    std::stringstream help("");

    help << std::endl;
    help << "<<< Available commands >>>" << std::endl;
    help << std::endl;
    help << showCommandHelp(HELP);
    help << showCommandHelp(SET);
    help << showCommandHelp(GET);
    help << showCommandHelp(TASK);
    help << showCommandHelp(SHOW);
    help << showCommandHelp(START);
    help << showCommandHelp(OPEN);
    help << showCommandHelp(ARM);
    help << showCommandHelp(GRASP);
    help << showCommandHelp(IS_HAND_OPEN);
    help << showCommandHelp(IS_HAND_CLOSE);
    help << showCommandHelp(SET_GRIP_STRENGTH);
    help << showCommandHelp(SET_MIN_FORCE);
    help << showCommandHelp(DISABLE_MIN_FORCE);
    help << showCommandHelp(OBJECT_RECOGNITION);
    help << showCommandHelp(QUIT);
    help << std::endl;

    return help.str();
}
