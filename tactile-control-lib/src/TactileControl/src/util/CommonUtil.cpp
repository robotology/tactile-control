#include "TactileControl/util/CommonUtil.h"

#include <yarp/sig/Vector.h>

using tactileControl::CommonUtil;
using yarp::sig::Vector;

void CommonUtil::initMinJerkTrajectory(iCub::ctrl::minJerkTrajGen* minJerkTrajectory,double referenceTime,double currentPosition){

    Vector initPosition(1,currentPosition);

    minJerkTrajectory->init(initPosition);
    
    minJerkTrajectory->setT(referenceTime);
}

void CommonUtil::getMinJerkFilteredPosition(iCub::ctrl::minJerkTrajGen* minJerkTrajectory,double targetPosition,double &filteredTargetPosition){

    Vector targetPositionVect(1,targetPosition);

    minJerkTrajectory->computeNextValues(targetPositionVect);

    Vector filteredTargetPositionVect = minJerkTrajectory->getPos();

    filteredTargetPosition = filteredTargetPositionVect[0];
}

void CommonUtil::putDataIntoVector(const double *dataIn,int size,yarp::sig::Vector &dataOut){

    dataOut.resize(size);

    for(int i = 0; i < size; i++){
        dataOut[i] = dataIn[i];
    }
}

void CommonUtil::putDataIntoMatrix(const double *dataIn,int rows,int columns,yarp::sig::Matrix &dataOut){

    dataOut.resize(rows,columns);

    for(int i = 0; i < rows; i++){
        for(int j = 0; j < columns; j++){
            dataOut[i][j] = dataIn[columns*i + j];
        }
    }
}


void CommonUtil::putSelectedElementsIntoVector(const yarp::sig::Vector &dataIn,const std::vector<int> &selectedIndexes,yarp::sig::Vector &dataOut){

    dataOut.resize(selectedIndexes.size());

    for(int i = 0; i < dataOut.size(); i++){
        dataOut[i] = dataIn[selectedIndexes[i]];
    }
}

void CommonUtil::putSelectedElementsIntoMatrix(const yarp::sig::Matrix &dataIn,const std::vector<int> &selectedRowIndexes,const std::vector<int> &selectedColumnIndexes,yarp::sig::Matrix &dataOut){

    dataOut.resize(selectedRowIndexes.size(),selectedColumnIndexes.size());

    for(int i = 0; i < dataOut.rows(); i++){
        for(int j = 0; j < dataOut.cols(); j++){
            dataOut[i][j] = dataIn[selectedRowIndexes[i]][selectedColumnIndexes[j]];
        }
    }
}
