//
// Created by HoldenSun on 2024/8/16.
//

#ifndef PCLNURBSSURFACE_H
#define PCLNURBSSURFACE_H

#include <pcl/point_cloud.h>
#include <pcl/surface/on_nurbs/triangulation.h>


class PclNurbsSurface {
public:
    static void pointCloud2Vector3d(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::on_nurbs::vector_vec3d &data);

    static void CreateCylinderPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::on_nurbs::vector_vec3d &data,
                                     unsigned npoints,
                                     double alpha, double h, double r);

    static void generateTestCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int width, int height);
};


#endif //PCLNURBSSURFACE_H
