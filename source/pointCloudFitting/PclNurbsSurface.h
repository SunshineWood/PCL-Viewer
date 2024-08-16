//
// Created by HoldenSun on 2024/8/16.
//

#ifndef PCLNURBSSURFACE_H
#define PCLNURBSSURFACE_H

#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/surface/on_nurbs/fitting_curve_2d_asdm.h>
#include <pcl/surface/on_nurbs/triangulation.h>


class PclNurbsSurface {
public:
    static void pointCloud2Vector3d(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::on_nurbs::vector_vec3d &data);

    static void visualizeCurve(ON_NurbsCurve &curve, ON_NurbsSurface &surface,
                               pcl::visualization::PCLVisualizer &viewer);

    static void CreateCylinderPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::on_nurbs::vector_vec3d &data,
                                     unsigned npoints,
                                     double alpha, double h, double r);

    static void generateTestCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int width, int height);
};


#endif //PCLNURBSSURFACE_H
