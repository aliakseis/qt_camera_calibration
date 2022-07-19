#pragma once

#include "IOWrapper/Output3DWrapper.h"

#include <vector>
#include <mutex>


struct Coord3d
{
    float x, y, z;
};

class CustomOutputWrapper : public dso::IOWrap::Output3DWrapper
{
public:
    void publishKeyframes(std::vector<dso::FrameHessian*> &frames, bool final, dso::CalibHessian* HCalib) override;
    ~CustomOutputWrapper();

private:
    std::vector<Coord3d> m_coords;
    std::mutex m_mtxCoords;
};
