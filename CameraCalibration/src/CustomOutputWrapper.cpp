#include "CustomOutputWrapper.h"

#include "FullSystem/HessianBlocks.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

CustomOutputWrapper::~CustomOutputWrapper()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    for (const auto& v : m_coords)
    {
        pcl::PointXYZ point(v.x, v.y, v.z);
        cloud->points.push_back(point);
    }

    cloud->height = 1;
    cloud->width = cloud->size();

    pcl::io::savePCDFile("c:/out/cloud.pcd", *cloud, true);
}

void CustomOutputWrapper::publishKeyframes(std::vector<dso::FrameHessian*> &frames, bool final, dso::CalibHessian* HCalib)
{
    if (final)
    {
        //*
        const auto fx = HCalib->fxl();
        const auto fy = HCalib->fyl();
        const auto cx = HCalib->cxl();
        const auto cy = HCalib->cyl();
        const auto fxi = 1 / fx;
        const auto fyi = 1 / fy;
        const auto cxi = -cx / fx;
        const auto cyi = -cy / fy;

        //*/

//*



    //w[0] = wG[0];
    //h[0] = hG[0];

        /*
        dso::Mat33f K[PYR_LEVELS];
        dso::Mat33f Ki[PYR_LEVELS];
        float fx[PYR_LEVELS];
        float fy[PYR_LEVELS];
        float fxi[PYR_LEVELS];
        float fyi[PYR_LEVELS];
        float cx[PYR_LEVELS];
        float cy[PYR_LEVELS];
        float cxi[PYR_LEVELS];
        float cyi[PYR_LEVELS];


    fx[0] = HCalib->fxl();
    fy[0] = HCalib->fyl();
    cx[0] = HCalib->cxl();
    cy[0] = HCalib->cyl();
    

    enum { pyrLevelsUsed = 2 };

    for (int level = 1; level < pyrLevelsUsed; ++ level)
    {
        //w[level] = w[0] >> level;
        //h[level] = h[0] >> level;
        fx[level] = fx[level-1] * 0.5;
        fy[level] = fy[level-1] * 0.5;
        cx[level] = (cx[0] + 0.5) / ((int)1<<level) - 0.5;
        cy[level] = (cy[0] + 0.5) / ((int)1<<level) - 0.5;
    }

    for (int level = 0; level < pyrLevelsUsed; ++ level)
    {
        K[level]  << fx[level], 0.0, cx[level], 0.0, fy[level], cy[level], 0.0, 0.0, 1.0;
        Ki[level] = K[level].inverse();
        fxi[level] = Ki[level](0,0);
        fyi[level] = Ki[level](1,1);
        cxi[level] = Ki[level](0,2);
        cyi[level] = Ki[level](1,2);
    }




//*/











        std::vector<Coord3d> coords;

        for (auto f : frames)
        {
            //printf("OUT: KF %d (%s) (id %d, tme %f): %d active, %d marginalized, %d immature points. CameraToWorld:\n",
            //    f->frameID,
            //    final ? "final" : "non-final",
            //    f->shell->incoming_id,
            //    f->shell->timestamp,
            //    (int)f->pointHessians.size(), (int)f->pointHessiansMarginalized.size(), (int)f->immaturePoints.size());
            //std::cout << f->shell->camToWorld.matrix3x4() << "\n";
            //int maxWrite = 5;

            auto camToWorld = f->PRE_camToWorld;

            //auto KRKi = (K[1] * camToWorld.rotationMatrix().cast<float>() * Ki[0]);
            //auto Kt = (K[1] * camToWorld.translation().cast<float>());

            for (auto p : f->pointHessiansMarginalized)
            {

                const auto x = p->u;
                const auto y = p->v;
                const auto depth = 1 / p->idepth;

                auto ptSource = Sophus::Vector3d((x*fxi + cxi), (y*fyi + cyi), 1);
                auto pt = camToWorld * (ptSource * depth);
                coords.push_back({ static_cast<float>(pt[0]), static_cast<float>(pt[1]), static_cast<float>(pt[2]) });


                //dso::Vec3f ptp = KRKi * dso::Vec3f(p->u, p->v, 1) + Kt * p->idepth_scaled;
                //coords.push_back({ ptp[0], ptp[1], ptp[2] });

                //printf("OUT: Example Point x=%.1f, y=%.1f, idepth=%f, idepth std.dev. %f, %d inlier-residuals\n",
                //    p->u, p->v, p->idepth_scaled, sqrt(1.0f / p->idepth_hessian), p->numGoodResiduals);
                //maxWrite--;
                //if (maxWrite == 0) break;

                /*
                int dx = 0;
                int dy = 0;

                const auto depth = 1 / p->idepth;

                coords.push_back({ 
                    ((p->u + dx)*fxi + cxi) * depth,
                    ((p->v + dy)*fyi + cyi) * depth,
                    depth });

                */
            }
        }

        std::lock_guard<std::mutex> guard(m_mtxCoords);
        m_coords.insert(m_coords.end(), coords.begin(), coords.end());
    }
}
