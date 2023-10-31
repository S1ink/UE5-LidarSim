#pragma once

#include <Modules/ModuleManager.h>
THIRD_PARTY_INCLUDES_START
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>
THIRD_PARTY_INCLUDES_END;


template<typename Point_T = pcl::PointXYZ>
struct PCLHelper {

    using PointCloud_T = pcl::PointCloud<Point_T>;
    using SegModel_T = pcl::SACSegmentation<Point_T>;


    template<int Method_T = pcl::SAC_RANSAC>
    static void segmodel_perpendicular(
        SegModel_T& seg,
        double dist_thresh,
        double angle_thresh,
        const Eigen::Vector3f& poip_vec = Eigen::Vector3f(0, 0, 1)
    ) {
        seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
        seg.setMethodType(Method_T);
        seg.setAxis(poip_vec);
        seg.setDistanceThreshold(dist_thresh);
        seg.setEpsAngle(angle_thresh);
    }
    template<int Method_T = pcl::SAC_RANSAC>
    static inline SegModel_T segmodel_perpendicular(
        double dist_thresh,
        double angle_thresh,
        const Eigen::Vector3f& poip_vec = Eigen::Vector3f(0, 0, 1)
    ) {
        pcl::SACSegmentation<Point_T> seg{};
        segmodel_perpendicular<Method_T>(seg, dist_thresh, angle_thresh, poip_vec);
        return seg;
    }

    static void filter_single(
        const typename PointCloud_T::Ptr cloud,
        SegModel_T& segmodel,
        pcl::PointIndices& inliers,
        pcl::ModelCoefficients& coeffs = {}
    ) {
        segmodel.setInputCloud(cloud);
        segmodel.segment(inliers, coeffs);
    }
    static PointCloud_T::Ptr filter_single(
        const typename PointCloud_T::Ptr cloud,
        SegModel_T& segmodel,
        PointCloud_T::Ptr filtered = new PointCloud_T,
        pcl::ModelCoefficients& coeffs = {},
        pcl::PointIndices& inliers = {},
        pcl::ExtractIndices<Point_T>& extractor = {}
    ) {
        filter_single(cloud, segmodel, inliers, coeffs);
        extractor.setInputCloud(cloud);
        extractor.setIndices(inliers);
        extractor.filter(filtered);
        return filtered;
    }

    //static PointCloud_T::Ptr filter_loop(
    //    const PointCloud_T::Ptr cloud,
    //    SegModel_T& segmodel,
    //    double angle_thresh,
    //    const Eigen::Vector3f& poip_vec = Eigen::Vector3f(0, 0, 1),
    //    PointCloud_T::Ptr filtered = new PointCloud_T,
    //    pcl::ModelCoefficients& coeffs = {},
    //    pcl::PointIndices& inliers = {},
    //    pcl::ExtractIndices<Point_T>& extractor = {}
    //) {
    //    PointCloud_T::Ptr buff{ new PointCloud_T };
    //    segmodel.setInputCloud(cloud);
    //    extractor.setInputCloud(cloud);
    //    for (;;) {
    //        segmodel.segment(inliers, coeffs);
    //        extractor.setIndices(inliers);
    //        const Eigen::Vector3f norm{ coeffs.values.data() };
    //        if (norm.dot(poip_vec) <= angle_thresh) {
    //            // plane is sufficient
    //            extractor.setNegative(true);
    //            extractor.filter(buff);
    //            filtered += buff;
    //            break;
    //        }
    //        else {
    //            // plane is not sufficient
    //            extractor.filter(buff);
    //            filtered += buff;
    //            extractor.setNegative(true);
    //            extractor.filter(buff);
    //            segmodel.setInputCloud(buff);
    //            extractor.setInputCloud(buff);
    //            // need alternate exit condition
    //        }
    //    }
    //    return filtered;
    //}

    template<typename Scalar_T = float>
    static inline PointCloud_T::Ptr translate(
        const PointCloud_T& cloud,
        const Eigen::Matrix4<Scalar_T>& translation,
        PointCloud_T::Ptr output = { new PointCloud_T }
    ) {
        pcl::transformPointCloud(cloud, *output, translation);
        return output;
    }
    template<typename Scalar_T = float>
    static inline PointCloud_T& translate_inline(
        PointCloud_T& cloud,
        const Eigen::Matrix4<Scalar_T>& translation
    ) {
        pcl::transformPointCloud(cloud, cloud, translation);
        return cloud;
    }
    template<typename Scalar_T = float>
    static inline PointCloud_T::Ptr translate_inline(
        PointCloud_T::Ptr cloud,
        const Eigen::Matrix4<Scalar_T>& translation
    ) {
        return translate(*cloud, translation, cloud);
    }


};