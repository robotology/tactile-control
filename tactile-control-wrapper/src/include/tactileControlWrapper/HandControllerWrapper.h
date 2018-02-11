#ifndef TACTILECONTROLWRAPPER_HANDCONTROLLERWRAPPER_H
#define TACTILECONTROLWRAPPER_HANDCONTROLLERWRAPPER_H

#include "TactileControl/HandController.h"
#include "TactileControl/../../../../robotology/tactile-control/tactile-control-lib/src/TactileControl/include/TactileControl/ObjectRecognitionManager.h"
#include "tactileControlWrapper/RPCUtil.h"
#include "tactileControlWrapper/RPCData.h"

#include <yarp/os/RFModule.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Value.h>
#include <yarp/os/ConstString.h>

#include <string>
#include <sstream>

namespace tactileControlWrapper {

    class HandControllerWrapper : public yarp::os::RFModule {

        private:

            tactileControl::HandController handController;
            tactileControl::ObjectRecognitionManager objRecManager;

            /* ****** Module attributes                             ****** */
            int moduleThreadPeriod;
            std::string moduleName;
            std::string robotName;
            bool closing;
            bool tasksRunning;
            std::stringstream errMsg;
            std::stringstream screenMsg;

            /* ****** RPC Ports                                     ****** */
            yarp::os::RpcServer portPlantIdentificationRPC;

            tactileControlWrapper::RPCUtil rpcCmdUtil;
            tactileControlWrapper::RPCData rpcCmdData;

            /* ****** Debug attributes                              ****** */
            std::string dbgTag;
           
        public:

            HandControllerWrapper();
            virtual ~HandControllerWrapper();
            virtual double getPeriod();
            virtual bool configure(yarp::os::ResourceFinder &rf);
            virtual bool updateModule();
            virtual bool interruptModule();
            virtual bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
            virtual bool close();

            /* ****** RPC Methods                                  ****** */
            bool start();
            bool open(const yarp::os::Value &paramValue,const yarp::os::Value &waitValue);
            bool arm(const yarp::os::Value &paramValue);
            bool grasp(const yarp::os::Value &waitValue);
            bool quit();
            bool set(const yarp::os::ConstString &paramName,const yarp::os::Value &paramValue);
            bool get(const yarp::os::ConstString &paramName);
            bool task(tactileControlWrapper::RPCTaskCmdArgName paramName, tactileControlWrapper::TaskName taskName, const yarp::os::Value &paramValue);
            bool show(tactileControlWrapper::RPCViewCmdArgName paramName);
            bool isHandOpen();
            bool isHandClose();
            bool setGripStrength(const yarp::os::Value &paramValue);
            bool setMinForce(const yarp::os::Value &paramValue);
            bool disableMinForce();
            bool objectRecognition(tactileControlWrapper::RPCObjRecCmdArgName paramName, const yarp::os::Value &paramValue);
            bool help();

    };

} // namespace tactileControlWrapper

#endif // TACTILECONTROLWRAPPER_HANDCONTROLLERWRAPPER_H

