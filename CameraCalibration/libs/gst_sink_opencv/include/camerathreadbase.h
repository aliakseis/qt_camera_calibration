#pragma once

#include <QThread>

class CameraThreadBase : public QThread
{
public:
    virtual double getBufPerc() = 0;
    virtual void dataConsumed() = 0;

    virtual void requestInterruption()
    {
        QThread::requestInterruption();
    }
};
