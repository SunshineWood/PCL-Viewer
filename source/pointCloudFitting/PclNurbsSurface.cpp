//
// Created by HoldenSun on 2024/8/16.
//

#include "PclNurbsSurface.h"

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/surface/on_nurbs/fitting_curve_2d_asdm.h>
#include <pcl/surface/on_nurbs/fitting_surface_tdm.h>
#include <pcl/surface/on_nurbs/triangulation.h>
#include <pcl/common/random.h>


void  PclNurbsSurface::pointCloud2Vector3d(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::on_nurbs::vector_vec3d &data) {
    for (const auto &p : *cloud) {
        if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
            data.emplace_back(p.x, p.y, p.z);
    }
}

void PclNurbsSurface::visualizeCurve(ON_NurbsCurve &curve, ON_NurbsSurface &surface, pcl::visualization::PCLVisualizer &viewer)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr curve_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::on_nurbs::Triangulation::convertCurve2PointCloud(curve, surface, curve_cloud, 4);
    for (std::size_t i = 0; i < curve_cloud->size() - 1; i++) {
        pcl::PointXYZRGB &p1 = curve_cloud->at(i);
        pcl::PointXYZRGB &p2 = curve_cloud->at(i + 1);
        std::ostringstream os;
        os << "line" << i;
        viewer.removeShape(os.str());
        viewer.addLine<pcl::PointXYZRGB>(p1, p2, 1.0, 0.0, 0.0, os.str());
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr curve_cps(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (int i = 0; i < curve.CVCount(); i++) {
        ON_3dPoint p1;
        curve.GetCV(i, p1);

        double pnt[3];
        surface.Evaluate(p1.x, p1.y, 0, 3, pnt);
        pcl::PointXYZRGB p2;
        p2.x = float(pnt[0]);
        p2.y = float(pnt[1]);
        p2.z = float(pnt[2]);

        p2.r = 255;
        p2.g = 0;
        p2.b = 0;

        curve_cps->push_back(p2);
    }
    viewer.removePointCloud("cloud_cps");
    viewer.addPointCloud(curve_cps, "cloud_cps");
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
        cloud->push_back (p);
    }
}

void PclNurbsSurface::generateTestCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,int width, int height)
{
    cloud->width = width;
    cloud->height = height;
    cloud->points.resize(width * height);

    pcl::common::UniformGenerator<float> rand(-0.01f, 0.01f);

    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            float u = static_cast<float>(x) / (width - 1);
            float v = static_cast<float>(y) / (height - 1);

            pcl::PointXYZ& point = cloud->at(x, y);
            point.x = u;
            point.y = v;
            // 生成一个简单的曲面 z = sin(πx) * cos(πy)
            point.z = std::sin(M_PI * u) * std::cos(M_PI * v) + rand.run();
        }
    }
}

