#include "TactileControl/data/GMMData.h"

#include "TactileControl/util/CommonUtil.h"

#include <yarp/math/Math.h>
#include <math.h>

using tactileControl::GMMData;

using yarp::os::Bottle;


GMMData::GMMData(tactileControl::GMMType gmmType) {

    int numDimensions;

    switch(gmmType){

    case STANDARD_GMM:
        numComponents = 2;
        numDimensions = 8;
        break;
    default:
        break;
    }

    mu.resize(numComponents);
    muQ.resize(numComponents);
    muR.resize(numComponents);
    sigma.resize(numComponents);
    sigmaQQ.resize(numComponents);
    sigmaQQInv.resize(numComponents);
    sigmaQQDet.resize(numComponents);
    sigmaRQ.resize(numComponents);
    sigmaRR.resize(numComponents);
    componentsPrior.resize(numComponents);
    
    if (gmmType == STANDARD_GMM){

        double muArray_0[] = {  145.3093,2.3186,7.1824,16.4359,37.2166,37.7803,54.502,-3.5809   };
        double muArray_1[] = {  119.0347,1.0004,10.5113,16.4967,33.4223,33.0245,60.7816,-1.7424 };

        double sigmaArray_0[] = {   71.8871,1.1888,-6.3919,2.1759,-12.8791,-9.7401,-4.9752,-9.5203,1.1888,24.3141,-0.43667,2.3628,7.1202,-2.0591,5.276,-17.3452,-6.3919,-0.43667,1.749,-0.48403,0.34381,1.4717,-0.36001,-0.61024,2.1759,2.3628,-0.48403,1.3684,2.4958,0.27596,1.6541,-1.0475,-12.8791,7.1202,0.34381,2.4958,13.709,5.0835,4.6905,-4.9262,-9.7401,-2.0591,1.4717,0.27596,5.0835,4.4143,-0.20451,-1.2702,-4.9752,5.276,-0.36001,1.6541,4.6905,-0.20451,7.5505,1.6798,-9.5203,-17.3452,-0.61024,-1.0475,-4.9262,-1.2702,1.6798,34.082  };
        double sigmaArray_1[] = {   332.521,-19.9406,-52.7226,53.0436,103.4661,100.5445,66.3989,49.9555,-19.9406,16.4703,3.0369,0.25231,-1.6621,-6.3955,-11.2062,-2.2165,-52.7226,3.0369,10.1815,-9.6832,-18.6553,-18.4258,-11.1193,-8.6307,53.0436,0.25231,-9.6832,17.2169,27.3674,25.0647,11.6776,10.4163,103.4661,-1.6621,-18.6553,27.3674,51.9909,44.9633,25.5945,23.0528,100.5445,-6.3955,-18.4258,25.0647,44.9633,45.9465,24.8972,20.5616,66.3989,-11.2062,-11.1193,11.6776,25.5945,24.8972,24.1067,15.775,49.9555,-2.2165,-8.6307,10.4163,23.0528,20.5616,15.775,20.5605 };


        CommonUtil::putDataIntoVector(muArray_0,numDimensions,mu[0]);
        CommonUtil::putDataIntoVector(muArray_1,numDimensions,mu[1]);


        CommonUtil::putDataIntoMatrix(sigmaArray_0,numDimensions,numDimensions,sigma[0]);
        CommonUtil::putDataIntoMatrix(sigmaArray_1,numDimensions,numDimensions,sigma[1]);

        componentsPrior[0] = 0.4707;
        componentsPrior[1] = 0.5293;
    
    }

    dbgTag = "GMMData: ";

}


double GMMData::calculateGMProbability(yarp::sig::Vector &queryPoint,int gmComponent){
    using namespace yarp::math;

    const double e = 2.71828183;

    double exp = -0.5 * dot(queryPoint - muQ[gmComponent],sigmaQQInv[gmComponent] * (queryPoint - muQ[gmComponent]));

    return 1/sqrt(sigmaQQDet[gmComponent]* pow(2*3.1415,(double)queryPoint.length())) * pow(e,exp);

}

void GMMData::runGaussianMixtureRegression(yarp::sig::Vector &queryPoint,yarp::sig::Vector &output){
    using namespace yarp::math;

    std::vector<double> h;
    std::vector<double> hNumerator;

    int numGMComponents = muQ.size();

    h.resize(numGMComponents);
    hNumerator.resize(numGMComponents);

    for (size_t i = 0; i < numGMComponents; i++){

        hNumerator[i] = componentsPrior[i] * calculateGMProbability(queryPoint,i);
    }

    double hDenominator = 0;
    for (size_t i = 0; i < numGMComponents; i++){

        hDenominator = hDenominator + hNumerator[i];
    }

    for (size_t i = 0; i < numGMComponents; i++){
    
        h[i] = hNumerator[i]/hDenominator;
    }

    
    output.resize(muR[0].length(),0.0);

    for (size_t i = 0; i < numGMComponents; i++){

        output = output + h[i] * (muR[i] + (sigmaRQ[i] * (sigmaQQInv[i] * (queryPoint - muQ[i]))));
    }

}

void GMMData::buildQRStructures(std::vector<int> &qIndexes,std::vector<int> &rIndexes){

    int dimQ = qIndexes.size();
    int dimR = rIndexes.size();

    for(size_t i = 0; i < numComponents; i++){
        
        CommonUtil::putSelectedElementsIntoVector(mu[i],qIndexes,muQ[i]);
        CommonUtil::putSelectedElementsIntoVector(mu[i],rIndexes,muR[i]);

        CommonUtil::putSelectedElementsIntoMatrix(sigma[i],qIndexes,qIndexes,sigmaQQ[i]);
        CommonUtil::putSelectedElementsIntoMatrix(sigma[i],rIndexes,qIndexes,sigmaRQ[i]);
        CommonUtil::putSelectedElementsIntoMatrix(sigma[i],rIndexes,rIndexes,sigmaRR[i]);

        sigmaQQInv[i] = yarp::math::luinv(sigmaQQ[i]);
        sigmaQQDet[i] = yarp::math::det(sigmaQQ[i]);
    }

}
