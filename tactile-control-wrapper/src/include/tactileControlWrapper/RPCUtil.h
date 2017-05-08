#ifndef TACTILECONTROLWRAPPER_RPCUTIL_H
#define TACTILECONTROLWRAPPER_RPCUTIL_H

#include "tactileControlWrapper/Enums.h"
#include "tactileControlWrapper/RPCData.h"

#include <yarp/os/Bottle.h>
#include <yarp/os/Value.h>
#include <yarp/os/ConstString.h>

#include <map>
#include <string>
#include <sstream>

namespace tactileControlWrapper {

    class RPCUtil {

    private:

        tactileControlWrapper::RPCData *rpcData;

        /* ****** Debug attributes                              ****** */
        std::string dbgTag;

    public:

        tactileControlWrapper::RPCMainCmdName mainCmd;
        tactileControlWrapper::RPCTaskCmdArgName taskCmdArg;
        tactileControlWrapper::RPCViewCmdArgName viewCmdArg;
        tactileControlWrapper::TaskName task;
		yarp::os::ConstString paramName;
		std::stringstream errMsg;

		yarp::os::Value argValue;
		yarp::os::Value waitValue;


        RPCUtil();

        void init(tactileControlWrapper::RPCData *rpcData);

        bool processCommand(const yarp::os::Bottle &rpcCmdBottle);

    private:

        bool processTaskCommand(const yarp::os::Bottle &rpcCmdBottle);

		void wrongSyntaxMessage(tactileControlWrapper::RPCMainCmdName mainCmd);

    };

} //namespace  // namespace tactileControlWrapper

#endif // TACTILECONTROLWRAPPER_RPCUTIL_H

