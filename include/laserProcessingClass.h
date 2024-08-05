// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro
#ifndef _LASER_PROCESSING_CLASS_H_
#define _LASER_PROCESSING_CLASS_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>

#include "lidar.h"

#include <sensor_msgs/Image.h>

#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Dense>

//points covariance class
class Double2d{
public:
	int id;
	double value;
	Double2d(int id_in, double value_in);
};
//points info class
class PointsInfo{
public:
	int layer;
	double time;
	PointsInfo(int layer_in, double time_in);
};


class LaserProcessingClass 
{
    public:
    	LaserProcessingClass();
		void init(lidar::Lidar lidar_param_in);
		void featureExtraction( pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, 
								pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_edge, 
								pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_first,
								sensor_msgs::ImageConstPtr& image_msg, 
                                Eigen::Matrix<double, 3, 4>& matrix_3Dto2D);
		void featureExtractionFromSector(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, 
											   std::vector<Double2d>& cloudCurvature, 
											   pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_edge,
											   pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_surf
											   );	
		void pointcloudtodepth(pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in,
                       sensor_msgs::ImageConstPtr& image_msg, 
                       Eigen::Matrix<double, 3, 4>& matrix_3Dto2D,
                       Eigen::Matrix3d& result,
                       Eigen::Matrix3d& RR,
                       Eigen::Vector3d& tt,
					   pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_first,
                       pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_surf
                       );
		void planeDetect(sensor_msgs::ImageConstPtr& image_msg, int windowSize, cv::Mat& depthImage, std::vector<cv::Point>& planePixels);

		void downSamplingToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_pc_out);
	private:
     	lidar::Lidar lidar_param;
		pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterSurf;
};



#endif // _LASER_PROCESSING_CLASS_H_

