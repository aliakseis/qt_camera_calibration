#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QLabel>
#include <QProgressBar>
#include <QProcess>
#include <QThreadPool>
#include <QSound>
#include <QMutex>

//#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>

#include "cameraman.h"

#include <memory>

class CameraThreadBase;
class QOpenCVScene;

class QCameraCalibrate;

namespace Ui
{
class MainWindow;
}

namespace dso
{
    class FullSystem;
    class Undistort;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

protected:
    QString updateOpenCvVer();
    QStringList updateCameraInfo();
    bool startGstProcess();
    bool killGstLaunch();
    bool startCamera();
    void stopCamera();

    void startDso();
    void stopDso();

    void addCloudPoints(const std::vector<std::array<float, 3>>& pointsCloud)
    {
        m_pointsCloud.insert(m_pointsCloud.end(), pointsCloud.begin(), pointsCloud.end());
    }

public slots:
    void onNewImage(cv::Mat frame);
    void onNewCbImage(cv::Mat cbImage);
    void onCbDetected();
    void onNewCameraParams(cv::Mat K, cv::Mat D, const cv::Size& imgSize, bool refining, double calibReprojErr );

protected slots:
    void onCameraConnected();
    void onCameraDisconnected(bool ok);
    void onProcessReadyRead();

    void updateParamGUI(cv::Mat K, cv::Mat D, const cv::Size& size);
    void updateCbParams();
    void setNewCameraParams();

private slots:
    void on_pushButton_update_camera_list_clicked();
    void on_comboBox_camera_currentIndexChanged(int index);
    void on_pushButton_camera_connect_disconnect_clicked(bool checked);

    void on_lineEdit_fx_editingFinished();
    void on_lineEdit_K_01_editingFinished();
    void on_lineEdit_cx_editingFinished();
    void on_lineEdit_K_10_editingFinished();
    void on_lineEdit_fy_editingFinished();
    void on_lineEdit_cy_editingFinished();
    void on_lineEdit_K_20_editingFinished();
    void on_lineEdit_K_21_editingFinished();
    void on_lineEdit_scale_editingFinished();
    void on_lineEdit_k1_editingFinished();
    void on_lineEdit_k2_editingFinished();
    void on_lineEdit_k3_editingFinished();
    void on_lineEdit_k4_editingFinished();
    void on_lineEdit_k5_editingFinished();
    void on_lineEdit_k6_editingFinished();
    void on_lineEdit_p1_editingFinished();
    void on_lineEdit_p2_editingFinished();

    void on_pushButton_calibrate_clicked(bool checked);

    void on_pushButton_reset_params_clicked();
    void on_pushButton_load_params_clicked();
    void on_pushButton_save_params_clicked();

    void on_pushButton_SavePointCloud_clicked();

    void on_horizontalSlider_alpha_valueChanged(int value);

    void on_checkBox_fisheye_clicked(bool checked);

    void on_pushButton_StartDSO_clicked(bool checked);

private:
    Ui::MainWindow *ui;

    QLabel mOpenCvVer;
    QLabel mGitVer;
    QLabel mCalibInfo;

    QProgressBar mCamBuffer;

    QProcess mGstProcess;

    QString mGstProcessOutput;
    QMutex mGstProcessOutputMutex;

    std::vector<CameraDesc> mCameras;
    CameraThreadBase* mCameraThread;
    bool mCameraConnected;

    QOpenCVScene* mCameraSceneRaw;
    QOpenCVScene* mCameraSceneCheckboard;
    QOpenCVScene* mCameraSceneUndistorted;

    cv::Mat mLastFrame;

    QString mLaunchLine;

    QString mSrcFormat;
    int mSrcWidth{};
    int mSrcHeight{};
    double mSrcFps{};
    int mSrcFpsNum{};
    int mSrcFpsDen{};

    cv::Size mCbSize{};
    float mCbSizeMm{};

    QThreadPool mElabPool;

    QCameraCalibrate* mCameraCalib;

    QSound* mCbDetectedSnd;

    bool mDsoInitialized = false;

    bool mDsoInitializationPostponed = false;

    std::unique_ptr<dso::FullSystem> fullSystem;
    std::unique_ptr<dso::Undistort> undistorter;
    int frameID = 0;

    std::vector<std::array<float, 3>> m_pointsCloud;

    bool mParametersReset = false;
};

#endif // MAINWINDOW_H
