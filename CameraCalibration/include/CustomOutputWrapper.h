#pragma once

#include "IOWrapper/Output3DWrapper.h"

#include <array>
#include <vector>
//#include <mutex>

//struct Coord3d
//{
//    float x, y, z;
//};


class CustomOutputWrapper : public dso::IOWrap::Output3DWrapper
{
public:
    typedef std::vector<std::array<float, 3>> PointsArray;

    CustomOutputWrapper(std::function<void(const PointsArray&)> pointsCallback);
    ~CustomOutputWrapper();
    void publishKeyframes(std::vector<dso::FrameHessian*> &frames, bool final, dso::CalibHessian* HCalib) override;

private:
    //std::vector<Coord3d> m_coords;
    //std::mutex m_mtxCoords;
    std::function<void(const PointsArray&)> m_pointsCallback;
};
