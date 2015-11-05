#include "iCub/objectGrasping/util/ControllersUtil.h"

#include <vector>
#include <ctime>
#include <yarp/os/Time.h>
#include <yarp/os/Value.h>
#include <yarp/sig/Vector.h>

using iCub::objectGrasping::ControllersUtil;
using iCub::objectGrasping::FingerJoint;

using std::string;
using std::cout;
using yarp::os::Property;


ControllersUtil::ControllersUtil(){

	dbgTag = "ControllersUtil: ";
}

bool ControllersUtil::init(yarp::os::ResourceFinder &rf){
	using std::vector;
	using yarp::os::Value;

	//TODO use constants
	jointsStoredControlMode.resize(8,VOCAB_CM_POSITION);

	string robotName = rf.check("robot", Value("icub"), "The robot name.").asString().c_str();
    whichHand = rf.check("whichHand", Value("right"), "The hand to be used for the grasping.").asString().c_str();

	 /* ******* Joint interfaces                     ******* */
    string arm = whichHand + "_arm";
    Property options;
    options.put("robot", robotName.c_str()); 
    options.put("device", "remote_controlboard");
    options.put("part", arm.c_str());
    options.put("local", ("/objectGrasping/" + arm).c_str());
    options.put("remote", ("/" + robotName + "/" + arm).c_str());
    
    // Open driver
    if (!clientArm.open(options)) {
        cout << dbgTag << "could not open driver\n";
		return false;
    }
    // Open interfaces
    clientArm.view(iEncs);
    if (!iEncs) {
		cout << dbgTag << "could not open encoders interface\n";
        return false;
    }
	clientArm.view(iCtrl);
    if (!iCtrl) {
		cout << dbgTag << "could not open control mode interface\n";
        return false;
    }
	clientArm.view(iPos);
    if (!iPos) {
		cout << dbgTag << "could not open position interface\n";
        return false;
    }
	clientArm.view(iVel);
    if (!iVel) {
		cout << dbgTag << "could not open velocity interface\n";
        return false;
    }

	iPos->getAxes(&armJointsNum);
	
	// Set reference speeds
    vector<double> refSpeeds(armJointsNum, 0);
    iPos->getRefSpeeds(&refSpeeds[0]);
    for (int i = 11; i < 15; ++i) {
        refSpeeds[i] = 50;
    }
    iPos->setRefSpeeds(&refSpeeds[0]);

    
    Property cartContrOptions;
    cartContrOptions.put("device","cartesiancontrollerclient");
    cartContrOptions.put("remote","/" + robotName + "/cartesianController/right_arm");
    cartContrOptions.put("local","/objectGrasping/client/right_arm");
    
    clientArmCartContr.open(cartContrOptions);

    if (clientArmCartContr.isValid()) {
       clientArmCartContr.view(iCart);
    }
    if (!iCart) {
		cout << dbgTag << "could not open cartesian controller interface\n";
    }

	return true;
}


bool ControllersUtil::saveCurrentArmPosition(){
	using yarp::os::Time;

    armStoredPosition.resize(armJointsNum);
    
	bool encodersDataAcquired = false;
    while(!encodersDataAcquired) {

        encodersDataAcquired = iEncs->getEncoders(armStoredPosition.data());

//#ifndef NODEBUG
        cout << "DEBUG: " << dbgTag << "Encoder data is not available yet. \n";
//#endif

		Time::delay(0.1);
    }

#ifndef NODEBUG
    cout << "\n";
    cout << "DEBUG: " << dbgTag << "Stored initial arm positions are: ";
    for (size_t i = 0; i < armStoredPosition.size(); ++i) {
        cout << armStoredPosition[i] << " ";
    }
    cout << "\n";
#endif

	return true;
}

bool ControllersUtil::saveCurrentControlMode(){

	// save control mode of joints 8 9 10 11 12 13 14 15
	for(size_t i = 0; i < 8; i++){
		if (!iCtrl->getControlMode(8 + i,&jointsStoredControlMode[i])){
			cout << dbgTag << "could not get current control mode\n";
			return false;
		}
	}
	return true;
}


/* ******* Place arm in grasping position                                   ********************************************** */ 
bool ControllersUtil::setArmInStartPosition(bool cartesianMode,bool back){

    cout << dbgTag << "Reaching arm grasp position ... \t";
    

    if (cartesianMode){

        if (!back){

            setPositionControlModeToArm(true,true);

	        // Arm
	        iPos->positionMove(0 , -11);
            iPos->positionMove(1 , 29);
            iPos->positionMove(2 , -22);
            iPos->positionMove(3 , 93);
            
            iPos->positionMove(4 , 0);// 0
            iPos->positionMove(5 , -14);// 1
            iPos->positionMove(6 , 9);// 1
            iPos->positionMove(7 , 14);
            
	        // Hand
            iPos->positionMove(8 , 72);
            iPos->positionMove(9 , 0);
            iPos->positionMove(10, 0);
            iPos->positionMove(11, 0);
            iPos->positionMove(12, 0);
            iPos->positionMove(13, 0);
            iPos->positionMove(14, 0);
            iPos->positionMove(15, 0);

      
        } else {

            incrementEndEffectorPosition(0.08,0.05,0,3.0);

        }


    } else {

        setPositionControlModeToArm(true,true);

	    // Arm
	    iPos->positionMove(0 ,-29);
        iPos->positionMove(1 , 54);
        iPos->positionMove(2 , -22);
        iPos->positionMove(3 , 45);
            
        iPos->positionMove(4 , -3);
        iPos->positionMove(5 , 17);
        iPos->positionMove(6 , 7);
        iPos->positionMove(7 , 15);
            
	    // Hand
        iPos->positionMove(8 , 72);
        iPos->positionMove(9 , 0);
        iPos->positionMove(10, 0);
        iPos->positionMove(11, 0);
        iPos->positionMove(12, 0);
        iPos->positionMove(13, 0);
        iPos->positionMove(14, 0);
        iPos->positionMove(15, 0);

    }

    // Check motion done
    waitMoveDone(10, 1);
	cout << "Done. \n";

	return true;
}
/* *********************************************************************************************************************** */


/* ******* Place arm in grasping position                                   ********************************************** */ 
bool ControllersUtil::testCartesianController() {
    using yarp::sig::Vector;

    cout << dbgTag << "Testing cartesian controller ... \t\n";
    
//	iVel->stop();

    Vector x0,o0;
    x0.resize(3);
    o0.resize(4);

    iCart->getPose(x0,o0);
    
    cout << dbgTag << "2\n";

    if (!iCart->getPose(x0,o0)) cout << "could not read pose!\n";
    else { cout << "pose read! n is " << x0.size() << "\n";}
    
    for (size_t i = 0; i < x0.size(); i++){
        std::cout << x0[i] << "\n";
    }    

    for (size_t i = 0; i < o0.size(); i++){
        std::cout << o0[i] << "\n";
    }    

    std::cout << "\n";


	Vector xd = x0;
	Vector od = o0;

	xd[0] += -0.06;
	xd[1] += -0.03;

	bool exec = false;
	exec = true;
	if (exec){

		iCart->setTrajTime(3.0);
		//iCart->setInTargetTol(0.001);

		iCart->goToPoseSync(xd,od);   // send request and wait for reply


		bool done=false;
		while (!done) {
		   iCart->checkMotionDone(&done);
		   yarp::os::Time::delay(0.04);
		}

		iCart->goToPoseSync(x0,o0);   // send request and wait for reply

		done=false;
		while (!done) {
		   iCart->checkMotionDone(&done);
		   yarp::os::Time::delay(0.04);
		}
	}

    // Check motion done
	//    waitMoveDone(10, 1);
	cout << "Done. \n";

	return true;
}
/* *********************************************************************************************************************** */

bool ControllersUtil::incrementEndEffectorPosition(double incrX,double incrY,double incrZ,double seconds){
	using yarp::sig::Vector;

	Vector x0,o0;
    if (!iCart->getPose(x0,o0)) {
	
		cout << "could not read pose!\n";
		return false;
	
	} else { 

		Vector xd = x0;

		xd[0] += incrX;
		xd[1] += incrY;
		xd[2] += incrZ;

		iCart->setTrajTime(seconds);
		//iCart->setInTargetTol(0.001);

		iCart->goToPoseSync(xd,o0);   // send request and wait for reply
		bool done=false;
		while (!done) {
			iCart->checkMotionDone(&done);
			yarp::os::Time::delay(0.04);
		}


		return true;
	}
}


/* ******* Place arm in grasping position                                   ********************************************** */ 
bool ControllersUtil::setArmInGraspPosition(bool cartesianMode,bool back) {

    cout << dbgTag << "Reaching arm grasp position ... \t";
    
	if (cartesianMode){

		double sign = (whichHand == "left" ? 1 : -1);

        if (!back){
			
            incrementEndEffectorPosition(-0.08,sign*0.05,0,3.0);

        } else {

            incrementEndEffectorPosition(0.02,0,-0.13,4.0);

        }
		return true;

	} else {
	
        setPositionControlModeToArm(true,true);

		// Arm
		iPos->positionMove(0 ,-36);
		iPos->positionMove(1 , 30);
		iPos->positionMove(2 , -5);
		iPos->positionMove(3 , 45);
        
		iPos->positionMove(4 , -1);
		iPos->positionMove(5 , 18);
		iPos->positionMove(6 , 7);
		iPos->positionMove(7 , 15);
        
		// Hand
	//    iPos->positionMove(8 , 79);
	//    iPos->positionMove(9 , 0);
	//    iPos->positionMove(10, 0);// 29
	//    iPos->positionMove(11, 0);
	//    iPos->positionMove(12, 0);
	//    iPos->positionMove(13, 0);
	//    iPos->positionMove(14, 0);//15
	//    iPos->positionMove(15, 0);

		// Check motion done
		waitMoveDone(10, 1,true);
		cout << "Done. \n";

		return true;
	}
}
/* *********************************************************************************************************************** */

/* ******* Place arm in grasping position                                   ********************************************** */ 
bool ControllersUtil::raiseArm(bool cartesianMode) {

    cout << dbgTag << "Reaching arm grasp position ... \t";
    
	if (cartesianMode){

        incrementEndEffectorPosition(-0.02,0,0.13,4.0);

		return true;
	} else {

        setPositionControlModeToArm(true,true);

		// Arm
		iPos->positionMove(0 ,-36);
		iPos->positionMove(1 , 30);
		iPos->positionMove(2 , -5);
		iPos->positionMove(3 , 90);
        
		iPos->positionMove(4 , -20);
		iPos->positionMove(5 , 18);
		iPos->positionMove(6 , 7);
		iPos->positionMove(7 , 15);
        
		// Hand
		//iPos->positionMove(8 , 79);
		//iPos->positionMove(9 , 2);
		//iPos->positionMove(10, 29);
		//iPos->positionMove(11, 0);
		//iPos->positionMove(12, 0);
		//iPos->positionMove(13, 25);
		//iPos->positionMove(14, 15);
		//iPos->positionMove(15, 1);

		// Check motion done
		waitMoveDone(10, 1,true);
		cout << "Done. \n";

		return true;
	}
}
/* *********************************************************************************************************************** */

bool ControllersUtil::restorePreviousArmPosition(){
	
    setPositionControlModeToArm(true,true);

	// Stop interfaces
    if (iVel) {
        iVel->stop();
    }
    if (iPos) {
        iPos->stop();
        // Restore initial robot position
        iPos->positionMove(armStoredPosition.data());
    }

	return true;
}

bool ControllersUtil::restorePreviousControlMode(){

	// restore control modes from joints 8 9 10 11 12 13 14 15
	for(size_t i = 0; i < 8; i++){
		if (!setControlMode(8 + i,jointsStoredControlMode[i],true)){
			cout << dbgTag << "could not set all control modes\n";
			return false;
		}
	}

	return true;
}

bool ControllersUtil::setPositionControlModeToArm(bool excludeHand,bool checkCurrent){

    for(size_t i = 0; i <= 6 || ((i > 6 && i <= 15) && !excludeHand); i++){
        setControlMode(i,VOCAB_CM_POSITION,checkCurrent);
    }

    return true;
}


bool ControllersUtil::setControlMode(int joint,int controlMode,bool checkCurrent){

	if (checkCurrent){
		int currentControlMode = -1;
		if (iCtrl->getControlMode(joint,&currentControlMode)){

			if (currentControlMode != controlMode){
				if  (iCtrl->setControlMode(joint,controlMode)){
					cout << dbgTag << "CONTROL MODE SET TO " << controlMode << " ON JOINT " << joint << "   PREV: " << currentControlMode << "  OPEN: " << VOCAB_CM_OPENLOOP << "\n";
					return true;
				} else {
                    cout << dbgTag << "failed to SET control mode on joint " << joint << "\n";                   
                    return false;
                }	
			} else {
                // cout << dbgTag << ".control mode already set (" << currentControlMode << ")\n";
                return true;
            }
		} else {
            cout << dbgTag << "failed to GET control mode from joint " << joint << " (controlMode appears to be " << currentControlMode << ")\n";           
            return false;
        }
	} else return iCtrl->setControlMode(joint,controlMode);
	
    return true;
}

/* ******* Wait for motion to be completed.                                 ********************************************** */
bool ControllersUtil::waitMoveDone(const double &i_timeout, const double &i_delay) {
    using yarp::os::Time;
    
    bool ok = false;
    
    double start = Time::now();

    while (!ok && (start - Time::now() <= i_timeout)) {
        iPos->checkMotionDone(&ok);
        Time::delay(i_delay);
    }

    return ok;
}
/* *********************************************************************************************************************** */

/* ******* Wait for motion to be completed.                                 ********************************************** */
bool ControllersUtil::waitMoveDone(const double &i_timeout, const double &i_delay, bool excludeHand) {
    using yarp::os::Time;
    
    bool ok = false;
    
    double start = Time::now();

    if (excludeHand){

        for (size_t i = 0; i <= 6; i++){
            ok = false;            
            while (!ok && (Time::now() - start <= i_timeout)) {
                iPos->checkMotionDone(i,&ok);
                if (!ok) Time::delay(i_delay);
            }
        }

    } else {

        while (!ok && (Time::now() - start <= i_timeout)) {
            iPos->checkMotionDone(&ok);
            Time::delay(i_delay);
        }

    }

    return ok;
}
/* *********************************************************************************************************************** */


bool ControllersUtil::release(){

    // Close driver
    if (!clientArm.close()){
		cout << dbgTag << "could not close driver\n";	
		return false;
	}
	return true;
}

/* *********************************************************************************************************************** */

/* *********************************************************************************************************************** */
/* ******* Open hand                                                        ********************************************** */
bool ControllersUtil::openHand() {
    
	cout << dbgTag << "Opening hand ... \t";
    
    iVel->stop();

	// Hand
    iPos->positionMove(8 , 79);
    iPos->positionMove(9 , 0);
    iPos->positionMove(10, 0);// 29
    iPos->positionMove(11, 0);
    iPos->positionMove(12, 0);
    iPos->positionMove(13, 0);
    iPos->positionMove(14, 0);//15
    iPos->positionMove(15, 0);

    // Check motion done
    waitMoveDone(10, 1);
    cout << "Done. \n";

	return true;
}

/* *********************************************************************************************************************** */

bool ControllersUtil::moveFingers() {
    
    iPos->positionMove(10, 15);
    iPos->positionMove(12, 30);
    iPos->positionMove(14, 30);

	return true;
}

