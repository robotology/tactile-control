#ifndef TACTILECONTROLWRAPPER_RPCDATA_H
#define TACTILECONTROLWRAPPER_RPCDATA_H

#include "tactileControlWrapper/Enums.h"

#include <yarp/os/Value.h>

#include <map>
#include <string>
#include <vector>

namespace tactileControlWrapper {

    class RPCData {

    private:

        /* ****** Debug attributes                              ****** */
        std::string dbgTag;

    public:

        std::map<tactileControlWrapper::RPCMainCmdName,std::string> mainCmdMap;
        std::map<tactileControlWrapper::RPCMainCmdName,std::string> mainCmdDescMap;
        std::map<std::string,tactileControlWrapper::RPCMainCmdName> mainCmdRevMap;

        std::map<tactileControlWrapper::RPCTaskCmdArgName,std::string> taskCmdArgMap;
        std::map<tactileControlWrapper::RPCTaskCmdArgName,std::string> taskCmdArgDescMap;
        std::map<std::string,tactileControlWrapper::RPCTaskCmdArgName> taskCmdArgRevMap;

        std::map<tactileControlWrapper::RPCViewCmdArgName,std::string> viewCmdArgMap;
        std::map<tactileControlWrapper::RPCViewCmdArgName,std::string> viewCmdArgDescMap;
        std::map<std::string,tactileControlWrapper::RPCViewCmdArgName> viewCmdArgRevMap;

        std::map<tactileControlWrapper::TaskName,std::string> taskMap;
        std::map<tactileControlWrapper::TaskName,std::string> taskDescMap;
        std::map<std::string,tactileControlWrapper::TaskName> taskRevMap;


        RPCData();

        std::string getFullDescription(tactileControlWrapper::RPCMainCmdName mainCmdName);

        bool setTargets(const yarp::os::Value &value,std::vector<double> &targets);

		std::string showCommandHelp(tactileControlWrapper::RPCMainCmdName mainCmdName);
		std::string showHelp();

    private:

        void add(std::string mapKey,tactileControlWrapper::RPCMainCmdName mapValue,std::string description);

        void add(std::string mapKey,tactileControlWrapper::RPCTaskCmdArgName mapValue,std::string description);

        void add(std::string mapKey,tactileControlWrapper::RPCViewCmdArgName mapValue,std::string description);

        void add(std::string mapKey,tactileControlWrapper::TaskName mapValue,std::string description);

    };

} // namespace tactileControlWrapper

#endif // TACTILECONTROLWRAPPER_RPCDATA_H

