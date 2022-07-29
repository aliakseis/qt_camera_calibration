#include "mainwindow.h"

#include "camerathread.h"
#include "ffmpeg_sink.h"
#include "qopencvscene.h"

// DSO stuff
#include "util/settings.h"
#include "FullSystem/FullSystem.h"
#include "util/Undistort.h"

#include "IOWrapper/Output3DWrapper.h"
#include "IOWrapper/ImageDisplay.h"


#include "util/settings.h"
#include "util/globalFuncs.h"
//#include "util/DatasetReader.h"
#include "util/globalCalib.h"

#include "util/NumType.h"
#include "FullSystem/FullSystem.h"
#include "OptimizationBackend/MatrixAccumulators.h"
#include "FullSystem/PixelSelector2.h"

#include "IOWrapper/Pangolin/PangolinDSOViewer.h"
//#include "IOWrapper/OutputWrapper/SampleOutputWrapper.h"

#include "ui_mainwindow.h"

#include "config.h"

#include <QCameraInfo>
#include <QGLWidget>
#include <QMessageBox>
#include <QFileDialog>
#include <QSound>
#include <QSettings>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <opencv2/core/eigen.hpp>

extern "C" {
#include <libavformat/avformat.h>
#include <libavdevice/avdevice.h>
}

#include <vector>

#include "qchessboardelab.h"
#include "qcameracalibrate.h"

#include "CustomOutputWrapper.h"

#include <iostream>
#include <functional>


const auto SETTING_CB_COLS = QStringLiteral("ChessboardCols");
const auto SETTING_CB_ROWS = QStringLiteral("ChessboardRows");
const auto SETTING_CB_SIZE = QStringLiteral("ChessboardSize");
const auto SETTING_CB_MAX_COUNT = QStringLiteral("ChessboardMaxCount");

const auto SETTING_FISHEYE = QStringLiteral("Fisheye");

const auto SETTING_USE_FFMPEG = QStringLiteral("UseFFmpeg");

const auto SETTING_FFMPEG_URL = QStringLiteral("FFmpegUrl");
const auto SETTING_FFMPEG_INPUT_FORMAT = QStringLiteral("FFmpegInputFormat");



const auto SETTING_DESIRED_IMMATURE_DENSITY = QStringLiteral("setting_desiredImmatureDensity");
const auto SETTING_DESIRED_POINT_DENSITY = QStringLiteral("setting_desiredPointDensity");
const auto SETTING_MIN_FRAMES = QStringLiteral("setting_minFrames");
const auto SETTING_MAX_FRAMES = QStringLiteral("setting_maxFrames");
const auto SETTING_MAX_OPT_ITERATIONS = QStringLiteral("setting_maxOptIterations");
const auto SETTING_MIN_OPT_ITERATIONS = QStringLiteral("setting_minOptIterations");

/*

setting_desiredImmatureDensity
setting_desiredPointDensity
setting_minFrames
setting_maxFrames
setting_maxOptIterations
setting_minOptIterations

SETTING_DESIRED_IMMATURE_DENSITY
SETTING_DESIRED_POINT_DENSITY
SETTING_MIN_FRAMES
SETTING_MAX_FRAMES
SETTING_MAX_OPT_ITERATIONS
SETTING_MIN_OPT_ITERATIONS

*/


using namespace std;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    mCameraThread(nullptr),
    mCameraSceneRaw(nullptr),
    mCameraSceneCheckboard(nullptr),
    mCameraSceneUndistorted(nullptr),
    mCameraCalib(nullptr),
    mCbDetectedSnd(nullptr)
{
    qRegisterMetaType<cv::Size>("cv::Size");
    qRegisterMetaType<std::vector<std::array<float, 3>>>("std::vector<std::array<float, 3>>");

    setWindowIcon(QIcon(":/icon.ico"));

    ui->setupUi(this);

    killGstLaunch();

    mCameraConnected = false;

    mCbDetectedSnd = new QSound( "://sound/cell-phone-1-nr0.wav", this);

    // >>>>> OpenCV version
    QString ocvVers = updateOpenCvVer();
    mOpenCvVer.setText( ocvVers );
    mOpenCvVer.setFrameStyle(QFrame::Panel | QFrame::Sunken);
    ui->statusBar->addPermanentWidget( &mOpenCvVer );
    // <<<<< OpenCV version

    // Git version
    mGitVer.setText(PROJECT_VERSION_GIT_FULL);
    mGitVer.setFrameStyle(QFrame::Panel | QFrame::Sunken);
    ui->statusBar->addPermanentWidget(&mGitVer);

    mCalibInfo.setText(tr("Ready"));
    mCalibInfo.setFrameStyle(QFrame::Panel | QFrame::Sunken);
    // >>>>> Calibration INFO
    ui->statusBar->addWidget( &mCalibInfo );
    // <<<<< Calibration INFO

    mCamBuffer.setFormat("Camera buffer: %p%");
    ui->statusBar->addWidget(&mCamBuffer);

    on_pushButton_update_camera_list_clicked();

    // >>>>> Stream rendering
    mCameraSceneRaw = new QOpenCVScene();
    mCameraSceneCheckboard = new QOpenCVScene();
    mCameraSceneUndistorted = new QOpenCVScene();

    ui->graphicsView_raw->setViewport( new QGLWidget );
    ui->graphicsView_raw->setScene( mCameraSceneRaw );
    ui->graphicsView_raw->setBackgroundBrush( QBrush( QColor(100,100,100) ) );

    ui->graphicsView_checkboard->setViewport( new QGLWidget );
    ui->graphicsView_checkboard->setScene( mCameraSceneCheckboard );
    ui->graphicsView_checkboard->setBackgroundBrush( QBrush( QColor(50,150,150) ) );

    ui->graphicsView_undistorted->setViewport( new QGLWidget );
    ui->graphicsView_undistorted->setScene( mCameraSceneUndistorted );
    ui->graphicsView_undistorted->setBackgroundBrush( QBrush( QColor(150,50,50) ) );

    ui->splitter->setStretchFactor(0, 1);
    ui->splitter->setStretchFactor(1, 1);

    // <<<<< Stream rendering

    ui->lineEdit_cb_cols->setValidator(new QIntValidator(2, 99, this));
    ui->lineEdit_cb_rows->setValidator(new QIntValidator(2, 99, this));
    ui->lineEdit_cb_mm->setValidator(new QIntValidator(1, 999, this));
    ui->lineEdit_cb_max_count->setValidator(new QIntValidator(10, 99, this));

    ui->lineEdit_size_x->setValidator(new QIntValidator(10, 9999, this));
    ui->lineEdit_size_y->setValidator(new QIntValidator(10, 9999, this));

    ui->lineEdit_setting_desiredImmatureDensity->setValidator(new QIntValidator(10, 99999, this));
    ui->lineEdit_setting_desiredPointDensity->setValidator(new QIntValidator(10, 99999, this));
    ui->lineEdit_setting_minFrames->setValidator(new QIntValidator(1, 99, this));
    ui->lineEdit_setting_maxFrames->setValidator(new QIntValidator(1, 99, this));
    ui->lineEdit_setting_maxOptIterations->setValidator(new QIntValidator(1, 99, this));
    ui->lineEdit_setting_minOptIterations->setValidator(new QIntValidator(1, 99, this));


    auto validationErrorLam = [this] {
        QMessageBox::warning(this, tr("Validation error"), tr("Please provide a correct value."));
    };

    for (auto edit : {
            ui->lineEdit_cb_cols,
            ui->lineEdit_cb_rows,
            ui->lineEdit_cb_mm,
            ui->lineEdit_cb_max_count,
            ui->lineEdit_size_x,
            ui->lineEdit_size_y,
            ui->lineEdit_setting_desiredImmatureDensity,
            ui->lineEdit_setting_desiredPointDensity,
            ui->lineEdit_setting_minFrames,
            ui->lineEdit_setting_maxFrames,
            ui->lineEdit_setting_maxOptIterations,
            ui->lineEdit_setting_minOptIterations
        })
    {
        connect(edit, &CustomLineEdit::validationError, validationErrorLam);
    }

    avdevice_register_all();

    QStringList inputFormats;
    inputFormats.push_back({});
    void *i{};
    while (auto fmt = av_demuxer_iterate(&i))
    {
        inputFormats.push_back(fmt->name);
    }
    std::sort(inputFormats.begin(), inputFormats.end());
    ui->comboBox_InputFormat->addItems(inputFormats);


    QSettings settings;
    ui->lineEdit_cb_cols->setText(settings.value(SETTING_CB_COLS, 10).toString());
    ui->lineEdit_cb_rows->setText(settings.value(SETTING_CB_ROWS, 7).toString());
    ui->lineEdit_cb_mm->setText(settings.value(SETTING_CB_SIZE, 25).toString());
    ui->lineEdit_cb_max_count->setText(settings.value(SETTING_CB_MAX_COUNT, 10).toString());

    ui->checkBox_fisheye->setChecked(settings.value(SETTING_FISHEYE, true).toBool());

    ui->lineEdit_URL->setText(settings.value(SETTING_FFMPEG_URL).toString());
    ui->comboBox_InputFormat->setCurrentText(settings.value(SETTING_FFMPEG_INPUT_FORMAT).toString());

    (settings.value(SETTING_USE_FFMPEG, false).toBool()
        ? ui->ffmpegSource : ui->gstreamerSource)->setChecked(true);


    // DSO

    //setting_desiredImmatureDensity = 1000;
    //setting_desiredPointDensity = 1200;
    //setting_minFrames = 5;
    //setting_maxFrames = 7;
    //setting_maxOptIterations = 4;
    //setting_minOptIterations = 1;

    ui->lineEdit_setting_desiredImmatureDensity->setText(settings.value(SETTING_DESIRED_IMMATURE_DENSITY, 1000).toString());
    ui->lineEdit_setting_desiredPointDensity->setText(settings.value(SETTING_DESIRED_POINT_DENSITY, 1200).toString());
    ui->lineEdit_setting_minFrames->setText(settings.value(SETTING_MIN_FRAMES, 5).toString());
    ui->lineEdit_setting_maxFrames->setText(settings.value(SETTING_MAX_FRAMES, 7).toString());
    ui->lineEdit_setting_maxOptIterations->setText(settings.value(SETTING_MAX_OPT_ITERATIONS, 4).toString());
    ui->lineEdit_setting_minOptIterations->setText(settings.value(SETTING_MIN_OPT_ITERATIONS, 1).toString());

    mElabPool.setMaxThreadCount( 3 );
}

MainWindow::~MainWindow()
{
    QSettings settings;
    settings.setValue(SETTING_CB_COLS, ui->lineEdit_cb_cols->text());
    settings.setValue(SETTING_CB_ROWS, ui->lineEdit_cb_rows->text());
    settings.setValue(SETTING_CB_SIZE, ui->lineEdit_cb_mm->text());
    settings.setValue(SETTING_CB_MAX_COUNT, ui->lineEdit_cb_max_count->text());

    settings.setValue(SETTING_FISHEYE, ui->checkBox_fisheye->isChecked());

    settings.setValue(SETTING_USE_FFMPEG, ui->ffmpegSource->isChecked());

    settings.setValue(SETTING_FFMPEG_URL, ui->lineEdit_URL->text());
    settings.setValue(SETTING_FFMPEG_INPUT_FORMAT, ui->comboBox_InputFormat->currentText());

    settings.setValue(SETTING_DESIRED_IMMATURE_DENSITY, ui->lineEdit_setting_desiredImmatureDensity->text());
    settings.setValue(SETTING_DESIRED_POINT_DENSITY, ui->lineEdit_setting_desiredPointDensity->text());
    settings.setValue(SETTING_MIN_FRAMES, ui->lineEdit_setting_minFrames->text());
    settings.setValue(SETTING_MAX_FRAMES, ui->lineEdit_setting_maxFrames->text());
    settings.setValue(SETTING_MAX_OPT_ITERATIONS, ui->lineEdit_setting_maxOptIterations->text());
    settings.setValue(SETTING_MIN_OPT_ITERATIONS, ui->lineEdit_setting_minOptIterations->text());

    killGstLaunch();

    while( mGstProcess.state() == QProcess::Running )
    {
        mGstProcess.kill();
        QApplication::processEvents( QEventLoop::AllEvents, 50 );
    }

    mElabPool.clear();

    delete ui;
    if (mCameraThread)
    {
        mCameraThread->requestInterruption();
        mCameraThread->wait();
        delete mCameraThread;
    }
    delete mCameraSceneRaw;
    delete mCameraSceneCheckboard;
    delete mCameraSceneUndistorted;
    delete mCameraCalib;
}

QString MainWindow::updateOpenCvVer()
{
    QString ocvVers = tr("OpenCV %1.%2.%3").arg(CV_MAJOR_VERSION).arg(CV_MINOR_VERSION).arg(CV_SUBMINOR_VERSION);

    return ocvVers;
}

QStringList MainWindow::updateCameraInfo()
{
    QStringList res;

    mCameras = getCameraDescriptions();
    for (const auto& cameraInfo : mCameras)
    {
        res.push_back(cameraInfo.id);
    }

    return res;
}

void MainWindow::on_pushButton_update_camera_list_clicked()
{
    ui->comboBox_camera->clear();
    ui->comboBox_camera->addItems( updateCameraInfo() );
}

void MainWindow::on_comboBox_camera_currentIndexChanged(int index)
{
    if( mCameras.empty() ) {
        return;
    }

    if( index>mCameras.size()-1  ) {
        return;
    }

    if( index<0 )
    {
        ui->label_camera->setText( tr("No camera info") );
    }
    else
    {
        ui->label_camera->setText(mCameras.at(index).description);

        ui->comboBox_camera_res->clear();

        for (const auto& mode : mCameras[index].modes)
        {
            ui->comboBox_camera_res->addItem(mode.getDescr());
        }

    }
}

bool MainWindow::startCamera()
{
    if(!killGstLaunch()) {
        return false;
    }

    if (mCameraThread)
    {
        mCameraThread->requestInterruption();
        mCameraThread->wait();
        delete mCameraThread;
        mCameraThread = nullptr;
    }

    if (ui->ffmpegSource->isChecked())
    {
        const auto ffmpegThread = new FFmpegThread(
            ui->lineEdit_URL->text().toStdString(),
            ui->comboBox_InputFormat->currentText().toStdString());

        if (!ffmpegThread->init())
        {
            delete ffmpegThread;
            return false;
        }

        mCameraThread = ffmpegThread;

        connect(ffmpegThread, &FFmpegThread::cameraDisconnected, this, &MainWindow::onCameraDisconnected);
        connect(ffmpegThread, &FFmpegThread::newImage, this, &MainWindow::onNewImage);
    }
    else
    {
        if (!startGstProcess()) {
            return false;
        }

        const auto& mode = mCameras[ui->comboBox_camera->currentIndex()].modes[ui->comboBox_camera_res->currentIndex()];

        const auto cameraThread = new CameraThread(mode.fps());
        mCameraThread = cameraThread;

        connect(cameraThread, &CameraThread::cameraConnected, this, &MainWindow::onCameraConnected);
        connect(cameraThread, &CameraThread::cameraDisconnected, this, &MainWindow::onCameraDisconnected);
        connect(cameraThread, &CameraThread::newImage, this, &MainWindow::onNewImage);
    }

    mCameraThread->start();

    return true;
}

void MainWindow::stopCamera()
{
    killGstLaunch();

    if( mCameraThread )
    {
        //disconnect( mCameraThread, &CameraThread::cameraConnected,
        //            this, &MainWindow::onCameraConnected );
        //disconnect( mCameraThread, &CameraThread::cameraDisconnected,
        //            this, &MainWindow::onCameraDisconnected );
        //disconnect( mCameraThread, &CameraThread::newImage,
        //            this, &MainWindow::onNewImage );

        mCameraThread->requestInterruption();
        mCameraThread->wait();
        delete mCameraThread;
        mCameraThread = nullptr;
    }
}

void MainWindow::onCameraConnected()
{
    mCameraConnected = true;
}

void MainWindow::onCameraDisconnected(bool ok)
{
    mCameraConnected = false;

    ui->pushButton_camera_connect_disconnect->setChecked(false);
    ui->pushButton_camera_connect_disconnect->setText( tr("Start Camera") );
    stopCamera();

    ui->lineEdit_cb_cols->setEnabled(true);
    ui->lineEdit_cb_rows->setEnabled(true);
    ui->lineEdit_cb_mm->setEnabled(true);
    ui->lineEdit_cb_max_count->setEnabled(true);

    ui->comboBox_camera->setEnabled(true);
    ui->comboBox_camera_res->setEnabled(true);

    ui->pushButton_load_params->setEnabled(true);
    //ui->pushButton_save_params->setEnabled(false);

    mGstProcessOutputMutex.lock();
    QString output = mGstProcessOutput;
    mGstProcessOutputMutex.unlock();

    if (!ok)
    {
        QMessageBox::warning(this, tr("Camera disconnected"),
            tr("If the camera has been just started please verify\n"
                "the correctness of Width, Height and FPS\n"
                "Process output:\n") + output.right(1000).trimmed());
    }
}

void MainWindow::onNewImage( cv::Mat frame )
{
    if (auto source = dynamic_cast<CameraThreadBase*>(sender()))
    {
        source->dataConsumed();
    }

    static int frmCnt=0;
    static int frameW = 0;
    static int frameH = 0;
    static int viewH_raw = 0;
    static int viewH_checkboard = 0;

    if( frameW != frame.cols ||
            frameH != frame.rows ||
            ui->graphicsView_raw->height() != viewH_raw ||
        ui->graphicsView_checkboard->height() != viewH_checkboard)
    {
        ui->graphicsView_raw->fitInView(QRectF(0,0, frame.cols, frame.rows),
                                        Qt::KeepAspectRatio );
        ui->graphicsView_checkboard->fitInView(QRectF(0,0, frame.cols, frame.rows),
                                               Qt::KeepAspectRatio );
        //ui->graphicsView_undistorted->fitInView(QRectF(0,0, frame.cols, frame.rows),
        //                                        Qt::KeepAspectRatio );
        frameW = frame.cols;
        frameH = frame.rows;
        viewH_raw = ui->graphicsView_raw->height();
        viewH_checkboard = ui->graphicsView_checkboard->height();
    }

    const auto pixmap = QOpenCVScene::cvMatToQPixmap(frame);

    mCameraSceneRaw->setFgImage(pixmap);

    frmCnt++;

    

    if( ui->pushButton_calibrate->isChecked() && frmCnt%((int)mSrcFps) == 0 )
    {
        if (mParametersReset)
        {
            mParametersReset = false;

            ui->lineEdit_size_x->setText(QString::number(mSrcWidth));
            ui->lineEdit_size_y->setText(QString::number(mSrcHeight));

            mCameraCalib->setImageSize(mSrcWidth, mSrcHeight);
        }

        auto* elab = new QChessboardElab( this, frame, mCbSize, mCbSizeMm, mCameraCalib );
        mElabPool.tryStart(elab);
    }

    //
    std::unique_ptr<dso::ImageAndExposure> undistImg;
    if (undistorter)
    {
        cv::Mat imgGray;
        if (frame.channels() == 1)
        {
            imgGray = frame;
        }
        else
        {
            cv::cvtColor(frame, imgGray, cv::COLOR_BGR2GRAY);
        }

        // TODO do we need it?
        if (imgGray.cols != undistorter->getOriginalSize()[0] || imgGray.rows != undistorter->getOriginalSize()[1])
        {
            cv::resize(imgGray, imgGray,
                { undistorter->getOriginalSize()[0], undistorter->getOriginalSize()[1] },
                0, 0, cv::INTER_LINEAR);
        }

        dso::MinimalImageB minImg(imgGray.cols, imgGray.rows, imgGray.data);
        undistImg.reset(undistorter->undistort<unsigned char>(&minImg, 1, 0, 1.0F));
    }

    cv::Mat rectified;
    if (undistImg)
    {
        const cv::Mat src(undistImg->h, undistImg->w, CV_32FC1, undistImg->image);
        src.convertTo(rectified, CV_8U);
    }
    else
    {
        rectified = mCameraCalib->undistort(frame);
        if (!rectified.empty())
        {
            // use rectified
            cv::Mat imgGray;
            if (rectified.channels() == 1)
            {
                imgGray = rectified;
            }
            else
            {
                cv::cvtColor(rectified, imgGray, cv::COLOR_BGR2GRAY);
            }

            cv::Mat imgFloat;
            imgGray.convertTo(imgFloat, CV_32F);

            //dso::MinimalImageB minImg(imgGray.cols, imgGray.rows, imgGray.data);
            //undistImg.reset(undistorter->undistort<unsigned char>(&minImg, 1, 0, 1.0f));
            undistImg = std::make_unique<dso::ImageAndExposure>(imgGray.cols, imgGray.rows);
            cv::Mat dst(undistImg->h, undistImg->w, CV_32FC1, undistImg->image);
            //imgGray.convertTo(dst, CV_32F);
            normalize(imgFloat, dst, 0, 255, cv::NORM_MINMAX);
        }
    }

    auto fitUndistorted = [this](int w, int h) {
        static int frameW = 0;
        static int frameH = 0;
        static int viewH = 0;

        if (frameW != w || frameH != h || ui->graphicsView_undistorted->height() != viewH)
        {
            ui->graphicsView_undistorted->fitInView(QRectF(0, 0, w, h),
                Qt::KeepAspectRatio);
            frameW = w;
            frameH = h;
            viewH = ui->graphicsView_undistorted->height();
        }
    };

    if( rectified.empty() )
    {
        fitUndistorted(pixmap.width(), pixmap.height());
        mCameraSceneUndistorted->setFgImage(pixmap);
        ui->graphicsView_undistorted->setBackgroundBrush( QBrush( QColor(150,50,50) ) );
    }
    else
    {
        fitUndistorted(rectified.cols, rectified.rows);
        mCameraSceneUndistorted->setFgImage(rectified);
        ui->graphicsView_undistorted->setBackgroundBrush( QBrush( QColor(50,150,50) ) );
    }

    if(mCameraThread)
    {
        double perc = mCameraThread->getBufPerc();

        int percInt = static_cast<int>(perc*100);

        mCamBuffer.setValue(percInt);
    }

    // DSO stuff
    //assert(cv_ptr->image.type() == CV_8U);
    //assert(cv_ptr->image.channels() == 1);

    using namespace dso;

    if (mDsoInitialized)
    {
        if (setting_fullResetRequested)
        {
            auto wraps = fullSystem->outputWrapper;
            //delete fullSystem;
            fullSystem.reset();
            for (auto ow : wraps) {
                ow->reset();
            }
            fullSystem = std::make_unique<FullSystem>();
            fullSystem->linearizeOperation = false;
            fullSystem->outputWrapper = wraps;
            if (undistorter->photometricUndist != nullptr) {
                fullSystem->setGammaFunction(undistorter->photometricUndist->getG());
            }
            setting_fullResetRequested = false;
        }

        //cv::Mat imgGray;
        //if (frame.channels() == 1)
        //{
        //    imgGray = frame;
        //}
        //else
        //{
        //    cv::cvtColor(frame, imgGray, cv::COLOR_BGR2GRAY);
        //}
        //MinimalImageB minImg(imgGray.cols, imgGray.rows, imgGray.data);

        //ImageAndExposure* undistImg = undistorter->undistort<unsigned char>(&minImg, 1, 0, 1.0f);
        // TODO?
        //undistImg->timestamp = img->header.stamp.toSec(); // relay the timestamp to dso

        if (undistImg)
        {
            try {
                fullSystem->addActiveFrame(undistImg.get(), frameID);
                frameID++;
            }
            catch (const std::exception& ex) {
                qDebug() << __FUNCTION__ << ": exception: " << typeid(ex).name() << ": " << ex.what();
            }
        }
        //delete undistImg;
    }
}

void MainWindow::onNewCbImage(cv::Mat cbImage)
{
    mCameraSceneCheckboard->setFgImage(cbImage);

    ui->lineEdit_cb_count->setText( QString::number(mCameraCalib->getCbCount()) );
}

void MainWindow::onCbDetected()
{
    //qDebug() << tr("Beep");

    mCbDetectedSnd->play();
}

void MainWindow::onNewCameraParams(cv::Mat K, cv::Mat D, const cv::Size& imgSize, bool refining, double calibReprojErr)
{
    if( refining )
    {
        mCalibInfo.setText( tr("Refining existing Camera parameters") );
    }
    else
    {
        mCalibInfo.setText( tr("Estimating new Camera parameters") );
    }

    ui->lineEdit_calib_reproj_err->setText(QString::number(calibReprojErr));

    if(calibReprojErr<=0.5 )
    {
        ui->lineEdit_calib_reproj_err->setStyleSheet("QLineEdit { background: rgb(50, 250, 50);}");
    }
    else if(calibReprojErr<=1.0 && calibReprojErr>0.5)
    {
        ui->lineEdit_calib_reproj_err->setStyleSheet("QLineEdit { background: rgb(250, 250, 50);}");
    }
    else
    {
        ui->lineEdit_calib_reproj_err->setStyleSheet("QLineEdit { background: rgb(250, 50, 50);}");
    }

    if( !K.empty() && !D.empty() )
    {
        updateParamGUI( K, D, imgSize);
    }
}

void MainWindow::on_pushButton_camera_connect_disconnect_clicked(bool checked)
{
    if( checked )
    {
        if (!mCameras.empty())
        {
            const auto& mCamera = mCameras[ui->comboBox_camera->currentIndex()];
            const auto& mode = mCamera.modes[ui->comboBox_camera_res->currentIndex()];

            mLaunchLine = mCamera.launchLine;

            mSrcFormat = mode.format;
            mSrcWidth = mode.w;
            mSrcHeight = mode.h;
            mSrcFps = mode.fps();
            mSrcFpsNum = mode.num;
            mSrcFpsDen = mode.den;
        }

        updateCbParams();

        if(mCameraCalib)
        {
            disconnect( mCameraCalib, &QCameraCalibrate::newCameraParams,
                        this, &MainWindow::onNewCameraParams );

            delete mCameraCalib;
        }

        bool fisheye = ui->checkBox_fisheye->isChecked();

        const int maxCount = ui->lineEdit_cb_max_count->text().toInt();

        const bool cameraStarted = startCamera();

        if (cameraStarted && ui->ffmpegSource->isChecked())
        {
            auto size = mCameraThread->getSize();
            mSrcWidth = size.width;
            mSrcHeight = size.height;

            std::tie(mSrcFpsDen, mSrcFpsNum) = mCameraThread->getFps();
            mSrcFps = static_cast<double>(mSrcFpsDen) / mSrcFpsNum;
        }

        mCameraCalib = new QCameraCalibrate( cv::Size(mSrcWidth, mSrcHeight), mCbSize, mCbSizeMm, fisheye, maxCount );

        setNewCameraParams();

        connect( mCameraCalib, &QCameraCalibrate::newCameraParams,
                 this, &MainWindow::onNewCameraParams );

        /*
        cv::Size imgSize;
        cv::Mat K;
        cv::Mat D;
        double alpha;
        mCameraCalib->getCameraParams( imgSize, K, D, alpha, fisheye );

        ui->checkBox_fisheye->setChecked(fisheye);
        ui->horizontalSlider_alpha->setValue( static_cast<int>( alpha*ui->horizontalSlider_alpha->maximum() ) );

        updateParamGUI(K,D);
        */

        if (cameraStarted)
        {
            ui->pushButton_camera_connect_disconnect->setText( tr("Stop Camera") );

            ui->lineEdit_cb_cols->setEnabled(false);
            ui->lineEdit_cb_rows->setEnabled(false);
            ui->lineEdit_cb_mm->setEnabled(false);
            ui->lineEdit_cb_max_count->setEnabled(false);

            ui->comboBox_camera->setEnabled(false);
            ui->comboBox_camera_res->setEnabled(false);

            ui->pushButton_load_params->setEnabled(false);
            //ui->pushButton_save_params->setEnabled(true);
        }
        else
        {
            ui->pushButton_camera_connect_disconnect->setText( tr("Start Camera") );
            ui->pushButton_camera_connect_disconnect->setChecked(false);

            ui->lineEdit_cb_cols->setEnabled(true);
            ui->lineEdit_cb_rows->setEnabled(true);
            ui->lineEdit_cb_mm->setEnabled(true);
            ui->lineEdit_cb_max_count->setEnabled(true);

            ui->comboBox_camera->setEnabled(true);
            ui->comboBox_camera_res->setEnabled(true);

            ui->pushButton_load_params->setEnabled(true);
            //ui->pushButton_save_params->setEnabled(false);
        }

        if (mDsoInitializationPostponed)
        {
            startDso();
        }
    }
    else
    {
        ui->pushButton_camera_connect_disconnect->setText( tr("Start Camera") );
        stopCamera();

        ui->lineEdit_cb_cols->setEnabled(true);
        ui->lineEdit_cb_rows->setEnabled(true);
        ui->lineEdit_cb_mm->setEnabled(true);
        ui->lineEdit_cb_max_count->setEnabled(true);

        ui->comboBox_camera->setEnabled(true);
        ui->comboBox_camera_res->setEnabled(true);

        ui->pushButton_load_params->setEnabled(true);
        //ui->pushButton_save_params->setEnabled(false);
    }
}

void MainWindow::onProcessReadyRead()
{
    while( mGstProcess.bytesAvailable() )
    {
        QByteArray line = mGstProcess.readLine();

        qDebug() << line;

        QApplication::processEvents( QEventLoop::AllEvents, 5 );
    }
}

bool MainWindow::killGstLaunch( )
{
    if (mGstProcess.state() != QProcess::Running) {
        return true;
}

    // >>>>> Kill gst-launch-1.0 processes

#ifdef Q_OS_WIN

    QProcess killer;
    killer.start("taskkill /im gst-launch-1.0.exe /f /t");
    killer.waitForFinished(1000);

#else

    QProcess killer;
    QProcess checker;

    int count = 0;
    bool done = false;
    do
    {
        killer.start( "pkill gst-launch" );
        killer.waitForFinished( 1000 );

        checker.start( "pgrep gst-launch" );
        checker.waitForFinished( 1000 );

        done = checker.readAll().size()==0;
        count++;

        if( count==10 )
        {
            qDebug() << tr("Cannot kill gst-launch active process(es)" );

            return false;
        }

    }
    while( !done );
    // <<<<< Kill gst-launch-1.0 processes
#endif // Q_OS_WIN

    return true;
}

bool MainWindow::startGstProcess( )
{
    // handle command line analogously to https://github.com/GStreamer/gst-plugins-base/blob/master/tools/gst-device-monitor.c
    if(mCameras.empty()) {
        return false;
    }

    QString launchStr;

#ifdef USE_ARM
    launchStr = QStringLiteral(
                "gst-launch-1.0 %1 do-timestamp=true ! "
                "\"video/x-raw,format=%2,width=%3,height=%4,framerate=%5/%6\" ! nvvidconv ! "
                "\"video/x-raw(memory:NVMM),width=%2,height=%3\" ! "
                //"omxh264enc low-latency=true insert-sps-pps=true ! "
                "omxh264enc insert-sps-pps=true ! "
                "rtph264pay config-interval=1 pt=96 mtu=9000 ! queue ! "
                "udpsink host=127.0.0.1 port=5000 sync=false async=false -e"
                ).arg(mLaunchLine).arg(mSrcFormat).arg(mSrcWidth).arg(mSrcHeight).arg(mSrcFpsDen).arg(mSrcFpsNum);
#elif defined(Q_OS_WIN)
    launchStr =
        QStringLiteral("gst-launch-1.0.exe %1 ! "
            "video/x-raw,format=%2,width=%3,height=%4,framerate=%5/%6 ! videoconvert ! "
            //"videoscale ! \"video/x-raw,width=%5,height=%6\" ! "
            "x264enc key-int-max=1 tune=zerolatency ! "//bitrate=8000 ! "
            "rtph264pay config-interval=1 pt=96 mtu=9000 ! queue ! "
            "udpsink host=127.0.0.1 port=5000 sync=false async=false -e")
        .arg(mLaunchLine).arg(mSrcFormat).arg(mSrcWidth).arg(mSrcHeight).arg(mSrcFpsDen).arg(mSrcFpsNum);
#else
    launchStr =
        QStringLiteral("gst-launch-1.0 %1 ! "
               "\"video/x-raw,format=%2,width=%3,height=%4,framerate=%5/%6\" ! videoconvert ! "
               //"videoscale ! \"video/x-raw,width=%5,height=%6\" ! "
               "x264enc key-int-max=1 tune=zerolatency bitrate=8000 ! "
               "rtph264pay config-interval=1 pt=96 mtu=9000 ! queue ! "
               "udpsink host=127.0.0.1 port=5000 sync=false async=false -e")
            .arg(mLaunchLine).arg(mSrcFormat).arg(mSrcWidth).arg(mSrcHeight).arg(mSrcFpsDen).arg(mSrcFpsNum);
#endif

    qDebug() << tr("Starting pipeline: \n %1").arg(launchStr);

    mGstProcess.setProcessChannelMode( QProcess::MergedChannels );

    mGstProcessOutput.clear();

    connect(&mGstProcess, &QProcess::readyReadStandardOutput, [this]() {
        QString output = mGstProcess.readAllStandardOutput();
        qDebug() << "Child process trace: " << output;
        QMutexLocker locker(&mGstProcessOutputMutex);
        mGstProcessOutput += output;
    });

    mGstProcess.start( launchStr );

    if( !mGstProcess.waitForStarted( 5000 ) )
    {
        // TODO Camera error message
        qDebug() << "Timed out starting a child process";
        return false;
    }

    return true;
}

void MainWindow::updateCbParams()
{
    mCbSize.width = ui->lineEdit_cb_cols->text().toInt();
    mCbSize.height = ui->lineEdit_cb_rows->text().toInt();
    mCbSizeMm = ui->lineEdit_cb_mm->text().toFloat();
}

void MainWindow::updateParamGUI( cv::Mat K, cv::Mat D, const cv::Size& size)
{
    double fx = K.ptr<double>(0)[0];
    double fy = K.ptr<double>(1)[1];
    double cx = K.ptr<double>(0)[2];
    double cy = K.ptr<double>(1)[2];
    double scale = K.ptr<double>(2)[2];

    ui->lineEdit_fx->setText( QString::number(fx) );
    ui->lineEdit_fy->setText( QString::number(fy) );
    ui->lineEdit_cx->setText( QString::number(cx) );
    ui->lineEdit_cy->setText( QString::number(cy) );
    ui->lineEdit_scale->setText( QString::number(scale) );

    double k1 = D.ptr<double>(0)[0];
    double k2 = D.ptr<double>(1)[0];

    if( ui->checkBox_fisheye->isChecked() )
    {
        double k3 = D.ptr<double>(2)[0];
        double k4 = D.ptr<double>(3)[0];

        ui->lineEdit_k1->setText( QString::number(k1) );
        ui->lineEdit_k2->setText( QString::number(k2) );
        ui->lineEdit_k3->setText( QString::number(k3) );
        ui->lineEdit_k4->setText( QString::number(k4) );

        ui->lineEdit_k5->setVisible(false);
        ui->lineEdit_k6->setVisible(false);
        ui->lineEdit_p1->setVisible(false);
        ui->lineEdit_p2->setVisible(false);
    }
    else
    {
        double p1 = D.ptr<double>(2)[0];
        double p2 = D.ptr<double>(3)[0];

        double k3 = D.ptr<double>(4)[0];
        double k4 = D.ptr<double>(5)[0];
        double k5 = D.ptr<double>(6)[0];
        double k6 = D.ptr<double>(7)[0];

        ui->lineEdit_k1->setText( QString::number(k1) );
        ui->lineEdit_k2->setText( QString::number(k2) );
        ui->lineEdit_p1->setText( QString::number(p1) );
        ui->lineEdit_p2->setText( QString::number(p2) );
        ui->lineEdit_k3->setText( QString::number(k3) );
        ui->lineEdit_k4->setText( QString::number(k4) );
        ui->lineEdit_k5->setText( QString::number(k5) );
        ui->lineEdit_k6->setText( QString::number(k6) );

        ui->lineEdit_k5->setVisible(true);
        ui->lineEdit_k6->setVisible(true);
        ui->lineEdit_p1->setVisible(true);
        ui->lineEdit_p2->setVisible(true);
    }

    ui->lineEdit_size_x->setText(QString::number(size.width));
    ui->lineEdit_size_y->setText(QString::number(size.height));
}

void MainWindow::setNewCameraParams()
{
    if( !mCameraCalib )
    {
        return;
    }

    cv::Mat K(3, 3, CV_64F, cv::Scalar::all(0.0F) );
    cv::Mat D( 8, 1, CV_64F, cv::Scalar::all(0.0F) );

    K.ptr<double>(0)[0] = ui->lineEdit_fx->text().toDouble();
    K.ptr<double>(0)[1] = ui->lineEdit_K_01->text().toDouble();
    K.ptr<double>(0)[2] = ui->lineEdit_cx->text().toDouble();
    K.ptr<double>(1)[0] = ui->lineEdit_K_10->text().toDouble();
    K.ptr<double>(1)[1] = ui->lineEdit_fy->text().toDouble();
    K.ptr<double>(1)[2] = ui->lineEdit_cy->text().toDouble();
    K.ptr<double>(2)[0] = ui->lineEdit_K_20->text().toDouble();
    K.ptr<double>(2)[1] = ui->lineEdit_K_21->text().toDouble();
    K.ptr<double>(2)[2] = ui->lineEdit_scale->text().toDouble();

    D.ptr<double>(0)[0] = ui->lineEdit_k1->text().toDouble();
    D.ptr<double>(1)[0] = ui->lineEdit_k2->text().toDouble();

    if(ui->checkBox_fisheye->isChecked())
    {
        D.ptr<double>(2)[0] = ui->lineEdit_k3->text().toDouble();
        D.ptr<double>(3)[0] = ui->lineEdit_k4->text().toDouble();
        //D.ptr<double>(4)[0] = 0.0;
        //D.ptr<double>(5)[0] = 0.0;
        //D.ptr<double>(6)[0] = 0.0;
        //D.ptr<double>(7)[0] = 0.0;
    }
    else
    {
        D.ptr<double>(2)[0] = ui->lineEdit_p1->text().toDouble();
        D.ptr<double>(3)[0] = ui->lineEdit_p2->text().toDouble();
        D.ptr<double>(4)[0] = ui->lineEdit_k3->text().toDouble();
        D.ptr<double>(5)[0] = ui->lineEdit_k4->text().toDouble();
        D.ptr<double>(6)[0] = ui->lineEdit_k5->text().toDouble();
        D.ptr<double>(7)[0] = ui->lineEdit_k6->text().toDouble();
    }

    double alpha = static_cast<double>(ui->horizontalSlider_alpha->value())/ui->horizontalSlider_alpha->maximum();
    bool fisheye = ui->checkBox_fisheye->isChecked();

    int additionalFlags = 0;
    if (ui->checkBox_fix_k1->isChecked()) {
        additionalFlags |= cv::CALIB_FIX_K1;
    }
    if (ui->checkBox_fix_k2->isChecked()) {
        additionalFlags |= cv::CALIB_FIX_K2;
    }
    if (ui->checkBox_fix_k3->isChecked()) {
        additionalFlags |= cv::CALIB_FIX_K3;
    }
    if (ui->checkBox_fix_k4->isChecked()) {
        additionalFlags |= cv::CALIB_FIX_K4;
    }
    if (ui->checkBox_fix_k5->isChecked()) {
        additionalFlags |= cv::CALIB_FIX_K5;
    }
    if (ui->checkBox_fix_k6->isChecked()) {
        additionalFlags |= cv::CALIB_FIX_K6;
    }

    const int width = ui->lineEdit_size_x->text().toInt();
    const int height = ui->lineEdit_size_y->text().toInt();

    mCameraCalib->setCameraParams( cv::Size(width, height), K, D, alpha, fisheye, additionalFlags);
}

void MainWindow::on_lineEdit_fx_editingFinished()
{
    setNewCameraParams();
}

void MainWindow::on_lineEdit_K_01_editingFinished()
{
    setNewCameraParams();
}

void MainWindow::on_lineEdit_cx_editingFinished()
{
    setNewCameraParams();
}

void MainWindow::on_lineEdit_K_10_editingFinished()
{
    setNewCameraParams();
}

void MainWindow::on_lineEdit_fy_editingFinished()
{
    setNewCameraParams();
}

void MainWindow::on_lineEdit_cy_editingFinished()
{
    setNewCameraParams();
}

void MainWindow::on_lineEdit_K_20_editingFinished()
{
    setNewCameraParams();
}

void MainWindow::on_lineEdit_K_21_editingFinished()
{
    setNewCameraParams();
}

void MainWindow::on_lineEdit_scale_editingFinished()
{
    setNewCameraParams();
}

void MainWindow::on_lineEdit_k1_editingFinished()
{
    setNewCameraParams();
}

void MainWindow::on_lineEdit_k2_editingFinished()
{
    setNewCameraParams();
}

void MainWindow::on_lineEdit_k3_editingFinished()
{
    setNewCameraParams();
}

void MainWindow::on_lineEdit_k4_editingFinished()
{
    setNewCameraParams();
}

void MainWindow::on_lineEdit_k5_editingFinished()
{
    setNewCameraParams();
}

void MainWindow::on_lineEdit_k6_editingFinished()
{
    setNewCameraParams();
}

void MainWindow::on_lineEdit_p1_editingFinished()
{
    setNewCameraParams();
}

void MainWindow::on_lineEdit_p2_editingFinished()
{
    setNewCameraParams();
}

void MainWindow::on_pushButton_calibrate_clicked(bool checked)
{
    ui->groupBox_params->setEnabled(!checked);
}

void MainWindow::on_pushButton_reset_params_clicked()
{
    mParametersReset = true;

    updateCbParams();

    stopDso();
    undistorter.reset();

    if (mCameraCalib)
    {
        disconnect(mCameraCalib, &QCameraCalibrate::newCameraParams,
            this, &MainWindow::onNewCameraParams);

        delete mCameraCalib;
    }

    bool fisheye = ui->checkBox_fisheye->isChecked();

    const int maxCount = ui->lineEdit_cb_max_count->text().toInt();

    mCameraCalib = new QCameraCalibrate(cv::Size(mSrcWidth, mSrcHeight), mCbSize, mCbSizeMm, fisheye, maxCount);

    connect(mCameraCalib, &QCameraCalibrate::newCameraParams,
        this, &MainWindow::onNewCameraParams);

    cv::Size imgSize;
    cv::Mat K;
    cv::Mat D;
    double alpha;
    mCameraCalib->getCameraParams( imgSize, K, D, alpha, fisheye );

    ui->checkBox_fisheye->setChecked(fisheye);
    ui->horizontalSlider_alpha->setValue( static_cast<int>( alpha*ui->horizontalSlider_alpha->maximum() ) );

    updateParamGUI(K, D, imgSize);
}


class UndistortPinholePlain : public dso::Undistort
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    UndistortPinholePlain(
        float fx,
        float fy,
        float cx,
        float cy,
        float k1,
        float k2,
        float r1,
        float r2,
        int w_, int h_)
    {
        parsOrg = dso::VecX(8);

        parsOrg[0] = fx;
        parsOrg[1] = fy;
        parsOrg[2] = cx;
        parsOrg[3] = cy;

        parsOrg[4] = k1;
        parsOrg[5] = k2;
        parsOrg[6] = r1;
        parsOrg[7] = r2;

        w = wOrg = w_;
        h = hOrg = h_;

        remapX = new float[w*h];
        remapY = new float[w*h];

        //if (outputCalibration[0] == -1)
        //    makeOptimalK_crop();
        //else if (outputCalibration[0] == -2)
        //    makeOptimalK_full();
        //else if (outputCalibration[0] == -3)
        {
            if (w != wOrg || h != hOrg)
            {
                printf("ERROR: rectification mode none requires input and output dimenstions to match!\n\n");
                exit(1);
            }
            K.setIdentity();
            K(0, 0) = parsOrg[0];
            K(1, 1) = parsOrg[1];
            K(0, 2) = parsOrg[2];
            K(1, 2) = parsOrg[3];
            passthrough = true;
        }
        //else
        //{


        //    if (outputCalibration[2] > 1 || outputCalibration[3] > 1)
        //    {
        //        printf("\n\n\nWARNING: given output calibration (%f %f %f %f) seems wrong. It needs to be relative to image width / height!\n\n\n",
        //            outputCalibration[0], outputCalibration[1], outputCalibration[2], outputCalibration[3]);
        //    }


        //    K.setIdentity();
        //    K(0, 0) = outputCalibration[0] * w;
        //    K(1, 1) = outputCalibration[1] * h;
        //    K(0, 2) = outputCalibration[2] * w - 0.5;
        //    K(1, 2) = outputCalibration[3] * h - 0.5;
        //}

        //if (benchmarkSetting_fxfyfac != 0)
        //{
        //    K(0, 0) = fmax(benchmarkSetting_fxfyfac, (float)K(0, 0));
        //    K(1, 1) = fmax(benchmarkSetting_fxfyfac, (float)K(1, 1));
        //    passthrough = false; // cannot pass through when fx / fy have been overwritten.
        //}


        for (int y = 0; y < h; y++) {
            for (int x = 0; x < w; x++)
            {
                remapX[x + y * w] = x;
                remapY[x + y * w] = y;
            }
        }

        distortCoordinates(remapX, remapY, remapX, remapY, h*w);


        for (int y = 0; y < h; y++) {
            for (int x = 0; x < w; x++)
            {
                // make rounding resistant.
                float ix = remapX[x + y * w];
                float iy = remapY[x + y * w];

                if (ix == 0) { 
                    ix = 0.001;
                }
                if (iy == 0) { 
                    iy = 0.001;
                }
                if (ix == wOrg - 1) { 
                    ix = wOrg - 1.001;
                }
                if (iy == hOrg - 1) { 
                    ix = hOrg - 1.001;
                }

                if (ix > 0 && iy > 0 && ix < wOrg - 1 && iy < wOrg - 1)
                {
                    remapX[x + y * w] = ix;
                    remapY[x + y * w] = iy;
                }
                else
                {
                    remapX[x + y * w] = -1;
                    remapY[x + y * w] = -1;
                }
            }
        }

        valid = true;

    }
    //~UndistortPinholePlain();
    void distortCoordinates(float* in_x, float* in_y, float* out_x, float* out_y, int n) const override
    {
        // current camera parameters
    // RADTAN
        float fx = parsOrg[0];
        float fy = parsOrg[1];
        float cx = parsOrg[2];
        float cy = parsOrg[3];
        float k1 = parsOrg[4];
        float k2 = parsOrg[5];
        float r1 = parsOrg[6];
        float r2 = parsOrg[7];

        float ofx = K(0, 0);
        float ofy = K(1, 1);
        float ocx = K(0, 2);
        float ocy = K(1, 2);

        for (int i = 0; i < n; i++)
        {
            float x = in_x[i];
            float y = in_y[i];

            // RADTAN
            float ix = (x - ocx) / ofx;
            float iy = (y - ocy) / ofy;
            float mx2_u = ix * ix;
            float my2_u = iy * iy;
            float mxy_u = ix * iy;
            float rho2_u = mx2_u + my2_u;
            float rad_dist_u = k1 * rho2_u + k2 * rho2_u * rho2_u;
            float x_dist = ix + ix * rad_dist_u + 2.0 * r1 * mxy_u + r2 * (rho2_u + 2.0 * mx2_u);
            float y_dist = iy + iy * rad_dist_u + 2.0 * r2 * mxy_u + r1 * (rho2_u + 2.0 * my2_u);
            float ox = fx * x_dist + cx;
            float oy = fy * y_dist + cy;


            out_x[i] = ox;
            out_y[i] = oy;
        }
    }

//private:
//    float inputCalibration[8];
};


void MainWindow::on_pushButton_load_params_clicked()
{
    QString filter1 = tr("OpenCV YAML (*.yaml *.yml)");
    QString filter2 = tr("XML (*.xml)");
    QString filter3 = tr("TXT (*.txt)");

    QString fileName = QFileDialog::getOpenFileName(this,
                                                    tr("Save Camera Calibration Parameters"), QDir::homePath(),
                                                    tr("%1;;%2;;%3").arg(filter1, filter2, filter3) );

    if( fileName.isEmpty() ) {
        return;
    }

    const bool forDso = fileName.endsWith(".txt", Qt::CaseInsensitive);

    // Not using the function from CameraUndistort to verify that they are coherent before setting them

    bool fisheye = false;
    double alpha = 0;

    cv::Mat K;
    cv::Mat D;

    cv::Size imgSize;

    if (forDso)
    { 
        undistorter.reset(dso::Undistort::getUndistorterForFile(QFile::encodeName(fileName).constData(), {}, {}));
        auto k = undistorter->getK();
        printf("\nUsed Kamera Matrix:\n");
        std::cout << k << "\n\n";
        cv::eigen2cv(k, K);
        D = cv::Mat(8, 1, CV_64F, cv::Scalar::all(0.0F));
        imgSize.width = undistorter->getSize()[0];
        imgSize.height = undistorter->getSize()[1];
    }
    else
    {
        cv::FileStorage fs( fileName.toLocal8Bit().toStdString(), cv::FileStorage::READ||cv::FileStorage::FORMAT_AUTO );
        if (!fs.isOpened())
        {
            return;
        }

        fs["FishEye"] >> fisheye;
        fs["Alpha"] >> alpha;

        fs["CameraMatrix"] >> K;

        //int w;
        //int h;

        fs["Width"] >> imgSize.width;
        fs["Height"] >> imgSize.height;

        fs["DistCoeffs"] >> D;

        /*
        if (!fisheye)
        {
            double k1 = D.ptr<double>(0)[0];
            double k2 = D.ptr<double>(1)[0];

            double p1 = D.ptr<double>(2)[0];
            double p2 = D.ptr<double>(3)[0];

            undistorter = std::make_unique<UndistortPinholePlain>(
                K.at<double>(0, 0),
                K.at<double>(1, 1),
                K.at<double>(0, 2),
                K.at<double>(1, 2),
                k1, k2, p1, p2,
                imgSize.width, imgSize.height);
            undistorter->loadPhotometricCalibration({}, {}, {});

            auto k = undistorter->getK();
            printf("\nUsed Kamera Matrix:\n");
            std::cout << k << "\n\n";
            //cv::eigen2cv(k, K);
            //D = cv::Mat(8, 1, CV_64F, cv::Scalar::all(0.0F));
        }
        else
        //*/
        {
            if (!ui->ffmpegSource->isChecked())
            {
                //int w;
                //int h;

                //fs["Width"] >> w;
                //fs["Height"] >> h;

                bool matched = false;

                if (!mCameras.empty())
                {
                    const auto& camera = mCameras[ui->comboBox_camera->currentIndex()];

                    for (int i = 0; i < camera.modes.size(); i++)
                    {
                        const auto& mode = camera.modes[ui->comboBox_camera_res->currentIndex()];

                        if (mode.w == imgSize.width && mode.h == imgSize.height)
                        {
                            matched = true;
                            ui->comboBox_camera_res->setCurrentIndex(i);
                            break;
                        }
                    }
                }

                if (!matched)
                {
                    QMessageBox::warning(this, tr("Warning"), tr("Current camera does not support the resolution\n"
                        "%1x%2 loaded from the file:\n"
                        "%3").arg(imgSize.width).arg(imgSize.height).arg(fileName));
                    //return;
                }
            }

            //fs["DistCoeffs"] >> D;
        }
    }

    ui->checkBox_fisheye->setChecked(fisheye);

    ui->horizontalSlider_alpha->setValue( static_cast<int>(alpha*ui->horizontalSlider_alpha->maximum()) );

    updateParamGUI(K, D, imgSize);

    setNewCameraParams();
}

void MainWindow::on_pushButton_save_params_clicked()
{
    if( !mCameraCalib ) {
        return;
    }

    QString selFilter;

    QString filter1 = tr("OpenCV YAML (*.yaml *.yml)");
    QString filter2 = tr("XML (*.xml)");

    QString fileName = QFileDialog::getSaveFileName(this,
                                                    tr("Save Camera Calibration Parameters"), QDir::homePath(),
                                                    tr("%1;;%2").arg(filter1).arg(filter2), &selFilter);

    if( fileName.isEmpty() ) {
        return;
    }

    if( !fileName.endsWith( ".yaml", Qt::CaseInsensitive) &&
            !fileName.endsWith( ".yml", Qt::CaseInsensitive) &&
            !fileName.endsWith( ".xml", Qt::CaseInsensitive) )
    {
        if( selFilter == filter2 )
        {
            fileName += ".xml";
        }
        else
        {
            fileName += ".yaml";
        }
    }

    cv::FileStorage fs( fileName.toLocal8Bit().toStdString(), cv::FileStorage::WRITE );

    if( fs.isOpened() )
    {
        cv::Size imgSize;
        cv::Mat K;
        cv::Mat D;
        bool fisheye;
        double alpha;

        mCameraCalib->getCameraParams( imgSize,K,D,alpha,fisheye );

        fs << "Width" << imgSize.width;
        fs << "Height" << imgSize.height;
        fs << "FishEye" << fisheye;
        fs << "Alpha" << alpha;
        fs << "CameraMatrix" << K;
        fs << "DistCoeffs" << D;
    }
}


void MainWindow::on_horizontalSlider_alpha_valueChanged(int value)
{
    if( mCameraCalib )
    {
        double alpha = (double)value/(ui->horizontalSlider_alpha->maximum());
        mCameraCalib->setNewAlpha(alpha);
    }
}

void MainWindow::on_checkBox_fisheye_clicked(bool checked)
{
    if( mCameraCalib )
    {
        mCameraCalib->setFisheye( checked );

        cv::Size imgSize;
        cv::Mat K;
        cv::Mat D;
        bool fisheye;
        double alpha;

        mCameraCalib->getCameraParams(imgSize, K, D, alpha, fisheye);
        updateParamGUI(K, D, imgSize);
    }
}

void MainWindow::on_pushButton_StartDSO_clicked(bool checked)
{

    //mCbDetectedSnd->play();

    //if (!undistorter)
    //    return;

    if (checked)
    {
        if (!undistorter && !ui->pushButton_camera_connect_disconnect->isChecked())
        {
            mDsoInitializationPostponed = true;
            return;
        }

        startDso();
    }
    else
    {
        stopDso();
    }
}

void MainWindow::startDso()
{
    if (mDsoInitialized) {
        return;
    }


    using namespace dso;

    //setting_desiredImmatureDensity = 1000;
    //setting_desiredPointDensity = 1200;
    //setting_minFrames = 5;
    //setting_maxFrames = 7;
    //setting_maxOptIterations = 4;
    //setting_minOptIterations = 1;

    setting_desiredImmatureDensity = ui->lineEdit_setting_desiredImmatureDensity->text().toInt();
    setting_desiredPointDensity = ui->lineEdit_setting_desiredPointDensity->text().toInt();
    setting_minFrames = ui->lineEdit_setting_minFrames->text().toInt();
    setting_maxFrames = ui->lineEdit_setting_maxFrames->text().toInt();
    setting_maxOptIterations = ui->lineEdit_setting_maxOptIterations->text().toInt();
    setting_minOptIterations = ui->lineEdit_setting_minOptIterations->text().toInt();

    setting_logStuff = false;
    setting_kfGlobalWeight = 1.3;


    //printf("MODE WITH CALIBRATION, but without exposure times!\n");
    //setting_photometricCalibration = 2;
    setting_photometricCalibration = 0;
    setting_affineOptModeA = 0;
    setting_affineOptModeB = 0;

    int w;
    int h;
    dso::Mat33 k;
    if (undistorter)
    {
        w = undistorter->getSize()[0];
        h = undistorter->getSize()[1];
        k = undistorter->getK();
    }
    else
    {
        cv::Size imgSize;
        cv::Mat K;
        cv::Mat D;
        double alpha;
        bool fisheye;
        mCameraCalib->getCameraParams(imgSize, K, D, alpha, fisheye);
        w = imgSize.width;
        h = imgSize.height;
        cv::cv2eigen(K, k);
    }

    if (!w) {
        w = mSrcWidth;
    }
    if (!h) {
        h = mSrcHeight;
    }

    printf("\nUsed Kamera Matrix:\n");
    std::cout << k << "\n\n";

    setGlobalCalib(
        //(int)undistorter->getSize()[0],
        //(int)undistorter->getSize()[1],
        //undistorter->getK()
        w, h,
        k.cast<float>());


    fullSystem = std::make_unique<FullSystem>();
    fullSystem->linearizeOperation = false;

    m_pointsCloud.clear();

    if (!disableAllDisplay) {
        auto stoppedCallback = [this] {
            QMetaObject::invokeMethod(this, &MainWindow::stopDso);
        };

        auto addPointsCallback = [this](const std::vector<std::array<float, 3>>& points) {
            QMetaObject::invokeMethod(this, [this, points] { addCloudPoints(points); });
        };

        fullSystem->outputWrapper.push_back(new IOWrap::PangolinDSOViewer(w, h, true, stoppedCallback));
        fullSystem->outputWrapper.push_back(new CustomOutputWrapper(addPointsCallback));
    }
            //(int)undistorter->getSize()[0],
            //(int)undistorter->getSize()[1]));


    //if (useSampleOutput)
    //    fullSystem->outputWrapper.push_back(new IOWrap::SampleOutputWrapper());


    if (undistorter && undistorter->photometricUndist != nullptr) {
        fullSystem->setGammaFunction(undistorter->photometricUndist->getG());
    }



    //
    mDsoInitialized = true;
}

void MainWindow::stopDso()
{
    if (mDsoInitialized)
    {
        auto wraps = fullSystem->outputWrapper;
        fullSystem.reset();
        for (auto ow : wraps) {
            delete ow;
        }
    }
    mDsoInitialized = false;
    mDsoInitializationPostponed = false;

    ui->pushButton_StartDSO->setChecked(false);
}

void MainWindow::on_pushButton_SavePointCloud_clicked()
{
    if (!mCameraCalib) {
        return;
    }

    QString selFilter;

    QString filter1 = tr("PCD (*.pcd)");

    QString fileName = QFileDialog::getSaveFileName(this,
        tr("Save Point Cloud"), QDir::homePath(),
        tr("%1").arg(filter1), &selFilter);

    if (fileName.isEmpty()) {
        return;
    }

    if (!fileName.endsWith(".pcd", Qt::CaseInsensitive))
    {
        if (selFilter == filter1)
        {
            fileName += ".pcd";
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    for (const auto& v : m_pointsCloud)
    {
        pcl::PointXYZ point(v[0], v[1], v[2]);
        cloud->points.push_back(point);
    }

    cloud->height = 1;
    cloud->width = cloud->size();

    pcl::io::savePCDFile(fileName.toLocal8Bit().toStdString(), *cloud, true);
}
