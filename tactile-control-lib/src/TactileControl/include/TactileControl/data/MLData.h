#ifndef TACTILECONTROL_MLDATA_H
#define TACTILECONTROL_MLDATA_H

#include <gurls++/kernelrlswrapper.h>

#include <string>
#include <map>

namespace tactileControl {

    class MLData {

    public:

        bool trainingSetLoaded;
        bool classifierTrained;

        gurls::gMat2D<double> xTr, yTr;
        std::map<int, std::string> objectsMap;

        gurls::KernelRLSWrapper<double> *wrapper;

        /* ****** Debug attributes                              ****** */
        std::string dbgTag;

    public:

        MLData();

        bool init();

        bool reset();

        bool release();

    };

} //namespace tactileControl

#endif // TACTILECONTROL_TASKDATA_H

