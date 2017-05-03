#ifndef TACTILECONTROL_GMMDATA_H
#define TACTILECONTROL_GMMDATA_H

#include "TactileControl/data/Enums.h"

#include <string>
#include <vector>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

namespace tactileControl {

    class GMMData {

        private:

            /* ******* Debug attributes.                ******* */
            std::string dbgTag;

            /* ******* Module attributes.               ******* */
            int numComponents;

            std::vector<yarp::sig::Vector> mu;
            std::vector<yarp::sig::Matrix> sigma;

            std::vector<yarp::sig::Vector> muQ;
            std::vector<yarp::sig::Vector> muR;
            std::vector<yarp::sig::Matrix> sigmaQQ;
            std::vector<yarp::sig::Matrix> sigmaRQ;
            std::vector<yarp::sig::Matrix> sigmaRR;
            std::vector<double> componentsPrior;
                
            std::vector<yarp::sig::Matrix> sigmaQQInv;
            std::vector<double> sigmaQQDet;

            double calculateGMProbability(yarp::sig::Vector &queryPoint, int gmmComponent);


        public:

            GMMData(tactileControl::GMMType gmmType);

            void runGaussianMixtureRegression(yarp::sig::Vector &queryPoint,yarp::sig::Vector &output);
                
            void buildQRStructures(std::vector<int> &qIndexes,std::vector<int> &rIndexes);
                

    };

} //namespace tactileControl

#endif // TACTILECONTROL_GMMDATA_H

