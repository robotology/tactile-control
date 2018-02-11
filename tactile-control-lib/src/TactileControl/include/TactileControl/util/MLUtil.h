#ifndef TACTILECONTROL_MLUTIL_H
#define TACTILECONTROL_MLUTIL_H

#include "TactileControl/data/TaskData.h"
#include "TactileControl/data/MLData.h"
#include "TactileControl/util/PortUtil.h"

#include <gurls++/kernelrlswrapper.h>

#include <yarp/os/ResourceFinder.h>

#include <string>
namespace tactileControl {

    class MLUtil {

    private:

        std::map<tactileControl::ClassifierType, tactileControl::MLData*> objRecDataMap;

        std::string modelFileName;
        std::string objectNamesFileName;
        std::string trainingSetXFileName;
        std::string trainingSetYFileName;
        std::string testSetXFileName;
        std::string testSetYFileName;

        // dynamic learning data
        std::vector<std::vector<double> > collectedFeatures;
        std::vector<int> objectIDs;

        tactileControl::PortUtil *portUtil;
        tactileControl::TaskData *taskData;

        /* ******* Debug attributes.                ******* */
        std::string dbgTag;

        int getArgMin(const gurls::gMat2D<double> &mat, int rowNum);

        bool getStandardPredictionsFrom1vsAll(const gurls::gMat2D<double> &predictions1vsAll, std::vector<int> &predictions);

        bool sendDetectedObjectToPort(int objectCode, tactileControl::ClassifierType classifierType);

        bool resizeMLMatrix(gurls::gMat2D<double> &mat, int rows, int cols);

        tactileControl::ClassifierType getClassifierType();

    public:

        MLUtil();

        bool init(tactileControl::TaskData *taskData, tactileControl::PortUtil *portUtil);

        bool loadTrainingSetFromFile(std::string fileSuffix);
        bool saveTrainingSetToFile(std::string fileSuffix);

        bool loadObjectNamesFromFile(std::string fileSuffix);
        bool saveObjectNamesToFile(std::string fileSuffix);

        bool loadModelFromFile(std::string fileSuffix);
        bool saveModelToFile(std::string fileSuffix);

        bool viewData();

        bool trainClassifier();

        bool testClassifierOneShot(const std::vector<double> &features, std::vector<double> &outputScores, bool sayResult, tactileControl::ClassifierType classifierType);

        bool reset();

        bool release();

        // methods related to dynamic learning
        bool clearCollectedFeatures();
        bool addCollectedFeatures(const std::vector<double> &features, int objectID);
        bool discardLastCollectedFeatures();
        bool processCollectedData();
        bool addNewObject(std::string objectName);

    };

} //namespace tactileControl

#endif // TACTILECONTROL_MLUTIL_H

