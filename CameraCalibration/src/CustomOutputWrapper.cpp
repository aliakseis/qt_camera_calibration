#include "CustomOutputWrapper.h"

#include <utility>

#include "FullSystem/HessianBlocks.h"


CustomOutputWrapper::CustomOutputWrapper(std::function<void(const PointsArray&)> pointsCallback)
    : m_pointsCallback(std::move(pointsCallback))
{
}

CustomOutputWrapper::~CustomOutputWrapper() = default;

// https://blog.csdn.net/huaweijian0324/article/details/80547782
void CustomOutputWrapper::publishKeyframes(std::vector<dso::FrameHessian*> &frames, bool final, dso::CalibHessian* HCalib)
{
    if (final)
    {
        const auto fx = HCalib->fxl();
        const auto fy = HCalib->fyl();
        const auto cx = HCalib->cxl();
        const auto cy = HCalib->cyl();
        const auto fxi = 1 / fx;
        const auto fyi = 1 / fy;
        const auto cxi = -cx / fx;
        const auto cyi = -cy / fy;

        PointsArray coords;

        for (auto f : frames)
        {
            auto camToWorld = f->PRE_camToWorld;

            for (auto p : f->pointHessiansMarginalized)
            {

                const auto x = p->u;
                const auto y = p->v;
                const auto depth = 1 / p->idepth;

                auto ptSource = Sophus::Vector3d((x*fxi + cxi), (y*fyi + cyi), 1);
                auto pt = camToWorld * (ptSource * depth);
                coords.push_back({ static_cast<float>(pt[0]), static_cast<float>(pt[1]), static_cast<float>(pt[2]) });
            }
        }

        m_pointsCallback(coords);
    }
}
