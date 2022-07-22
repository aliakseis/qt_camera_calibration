#pragma once

#include "camerathreadbase.h"

#include <QMutex>
#include <QWaitCondition>

#include <string>
#include <utility>
#include <atomic>

struct AVFormatContext;
struct AVCodecContext;

class FFmpegThread : public CameraThreadBase
{
    enum { MAX_QUEUE_SIZE = 10 };

    Q_OBJECT

public:
    FFmpegThread(std::string url, std::string inputFormat)
        : m_url(std::move(url)), m_inputFormat(std::move(inputFormat))
    {}

    ~FFmpegThread();

    bool init();

    double getBufPerc() override { return m_queueSize / (double)MAX_QUEUE_SIZE; }
    void dataConsumed() override;
    cv::Size getSize() override;
    std::pair<int, int> getFps() override;
    void requestInterruption() override;

signals:
    void newImage(cv::Mat frame);
    void cameraDisconnected(bool ok);

protected:
    void run() override;

private:
    std::string m_url;
    std::string m_inputFormat;

    AVFormatContext* m_formatContext{};
    AVCodecContext* m_codecContext{};
    int m_streamNumber;

    std::atomic_int m_queueSize{0};
    QMutex m_mtxQueueSize;
    QWaitCondition m_cvQueueSize;
};
