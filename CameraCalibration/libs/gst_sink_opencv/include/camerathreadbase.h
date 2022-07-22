#pragma once

#include <QThread>

#include <opencv2/core/core.hpp>
#include <utility>

class CameraThreadBase : public QThread
{
public:
    virtual double getBufPerc() = 0;
    virtual void dataConsumed() = 0;
    virtual cv::Size getSize() = 0;
    virtual std::pair<int, int> getFps() = 0;

    virtual void requestInterruption()
    {
        QThread::requestInterruption();
    }
};

