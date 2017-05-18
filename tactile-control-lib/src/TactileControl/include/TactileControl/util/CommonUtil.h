#ifndef TACTILECONTROL_COMMONUTIL_H
#define TACTILECONTROL_COMMONUTIL_H

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <iCub/ctrl/minJerkCtrl.h>

#include <vector>

namespace tactileControl {

    class CommonUtil {

    public:

        static void initMinJerkTrajectory(iCub::ctrl::minJerkTrajGen* minJerkTrajectory,double referenceTime,double currentPosition);

        static void getMinJerkFilteredPosition(iCub::ctrl::minJerkTrajGen* minJerkTrajectory,double targetPosition,double &filteredTargetPosition);

        static void putDataIntoVector(const double *dataIn,int size,yarp::sig::Vector &dataOut);
        static void putDataIntoMatrix(const double *dataIn,int rows,int columns,yarp::sig::Matrix &dataOut);
        static void putSelectedElementsIntoVector(const yarp::sig::Vector &dataIn,const std::vector<int> &selectedIndexes,yarp::sig::Vector &dataOut);
        static void putSelectedElementsIntoMatrix(const yarp::sig::Matrix &dataIn,const std::vector<int> &selectedRowIndexes,const std::vector<int> &selectedColumnIndexes,yarp::sig::Matrix &dataOut);

    };

} //namespace tactileControl

#endif // TACTILECONTROL_COMMONUTIL_H

