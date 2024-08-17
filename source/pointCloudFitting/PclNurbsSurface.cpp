//
// Created by HoldenSun on 2024/8/16.
//

#include "PclNurbsSurface.h"


void  PclNurbsSurface::pointCloud2Vector3d(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::on_nurbs::vector_vec3d &data) {
    for (const auto &p : *cloud) {
        if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
            data.emplace_back(p.x, p.y, p.z);
    }
}

void PclNurbsSurface::CreateCylinderPoints (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::on_nurbs::vector_vec3d &data, unsigned npoints,
                      double alpha, double h, double r)
{
    for (unsigned i = 0; i < npoints; i++)
    {
        double da = alpha * double (rand ()) / RAND_MAX;
        double dh = h * (double (rand ()) / RAND_MAX - 0.5);

        pcl::PointXYZ p;
        p.x = float (r * std::cos (da));
        p.y = float (r * sin (da));
        p.z = float (dh);

        data.push_back (Eigen::Vector3d (p.x, p.y, p.z));
        cloud->push_back(p);
    }
}

