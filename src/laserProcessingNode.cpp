// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

//c++ lib
#include <cmath>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>

//ros lib
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

//pcl lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//local lib
#include "lidar.h"
#include "laserProcessingClass.h"

//後來加的
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include "laserProcessingClass.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

LaserProcessingClass laserProcessing;
std::mutex mutex_lock;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudBuf;
std::queue<sensor_msgs::ImageConstPtr> imageBuf;
lidar::Lidar lidar_param;

ros::Publisher pubEdgePoints;
ros::Publisher pubSurfPoints;
ros::Publisher pubLaserCloudFiltered;
ros::Publisher pubImage;

Eigen::Matrix<double, 3, 4> matrix_3Dto2D; //相乘的值
Eigen::Matrix3d result;
Eigen::Matrix3d RR;
Eigen::Vector3d tt;

void velodyneHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg, const sensor_msgs::ImageConstPtr &laserImageMsg)
{
    mutex_lock.lock();
    pointCloudBuf.push(laserCloudMsg);
    imageBuf.push(laserImageMsg);
    mutex_lock.unlock();
}

double total_time =0;
int total_frame=0;

void laser_processing(){
    while(1){
        if(!pointCloudBuf.empty() && !imageBuf.empty()){
            //read data
            mutex_lock.lock();
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::fromROSMsg(*pointCloudBuf.front(), *pointcloud_in);
            sensor_msgs::ImageConstPtr image_msg = imageBuf.front();

            ros::Time pointcloud_time = (pointCloudBuf.front())->header.stamp;
            pointCloudBuf.pop();
            imageBuf.pop();
            mutex_lock.unlock();

            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_edge(new pcl::PointCloud<pcl::PointXYZI>());          
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_surf(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::PointCloud<pcl::PointXYZI>::Ptr surf_first(new pcl::PointCloud<pcl::PointXYZI>());

            std::chrono::time_point<std::chrono::system_clock> start, end;
            start = std::chrono::system_clock::now();
            // laserProcessing.featureExtraction(pointcloud_in, pointcloud_edge, image_msg, matrix_3Dto2D);
            laserProcessing.featureExtraction(pointcloud_in, pointcloud_edge, surf_first, image_msg, matrix_3Dto2D);
            laserProcessing.pointcloudtodepth(pointcloud_in, image_msg, matrix_3Dto2D, result, RR, tt, surf_first, pointcloud_surf);
            end = std::chrono::system_clock::now();
            std::chrono::duration<float> elapsed_seconds = end - start;
            total_frame++;
            float time_temp = elapsed_seconds.count() * 1000;
            total_time+=time_temp;
            ROS_INFO("average laser processing time %f ms \n \n", total_time/total_frame);

            sensor_msgs::PointCloud2 laserCloudFilteredMsg;
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_filtered(new pcl::PointCloud<pcl::PointXYZI>());  
            *pointcloud_filtered+=*pointcloud_edge;
            *pointcloud_filtered+=*pointcloud_surf;
            pcl::toROSMsg(*pointcloud_filtered, laserCloudFilteredMsg);
            laserCloudFilteredMsg.header.stamp = pointcloud_time;
            laserCloudFilteredMsg.header.frame_id = "base_link";
            pubLaserCloudFiltered.publish(laserCloudFilteredMsg);

            sensor_msgs::PointCloud2 edgePointsMsg;
            pcl::toROSMsg(*pointcloud_edge, edgePointsMsg);
            edgePointsMsg.header.stamp = pointcloud_time;
            edgePointsMsg.header.frame_id = "base_link";
            pubEdgePoints.publish(edgePointsMsg);

            sensor_msgs::PointCloud2 surfPointsMsg;
            pcl::toROSMsg(*pointcloud_surf, surfPointsMsg);
            surfPointsMsg.header.stamp = pointcloud_time;
            surfPointsMsg.header.frame_id = "base_link";
            pubSurfPoints.publish(surfPointsMsg);

            sensor_msgs::Image image_publish_msg = *image_msg;
            image_publish_msg.header.stamp = pointcloud_time;  // 或者使用你需要的時間
            image_publish_msg.header.frame_id = "base_link";  // 使用你需要的 frame_id
            pubImage.publish(image_publish_msg);

        }
        //sleep 2 ms every time
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;

    int scan_line = 64;
    double vertical_angle = 2.0;
    double scan_period= 0.1;
    double max_dis = 60.0;
    double min_dis = 2.0;

    int sequence_number = 4;

    nh.getParam("/scan_period", scan_period); 
    nh.getParam("/vertical_angle", vertical_angle); 
    nh.getParam("/max_dis", max_dis);
    nh.getParam("/min_dis", min_dis);
    nh.getParam("/scan_line", scan_line);
    nh.getParam("/sequence_number", sequence_number);

    lidar_param.setScanPeriod(scan_period);
    lidar_param.setVerticalAngle(vertical_angle);
    lidar_param.setLines(scan_line);
    lidar_param.setMaxDistance(max_dis);
    lidar_param.setMinDistance(min_dis);

    laserProcessing.init(lidar_param);

    // ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 100, velodyneHandler);
    // ros::Subscriber subImageLeft = nh.subscribe<sensor_msgs::Image>("/image_left", 100, imageLeftHandler);

    Eigen::Matrix<double, 3, 4> Project_matrix; //內參
    Eigen::Matrix4d rotation_matrix = Eigen::Matrix4d::Zero(); //不知道做啥的矩陣 乘就對了
    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Zero(); //外參
    // Eigen::Matrix<double, 3, 4> matrix_3Dto2D; //相乘的值
    Eigen::Matrix3d Project_matrix_3x3;
    Eigen::Matrix3d rotation_matrix_3x3;

    if(sequence_number >= 0 && sequence_number <=2){
        Project_matrix << 7.188560e+02, 0.000000e+00, 6.071928e+02, 0.000000e+00, 
                          0.000000e+00, 7.188560e+02, 1.852157e+02, 0.000000e+00,
                          0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00;

        rotation_matrix << 9.999454e-01, 7.259129e-03, -7.519551e-03, 0,
                          -7.292213e-03, 9.999638e-01, -4.381729e-03, 0,
                           7.487471e-03, 4.436324e-03,  9.999621e-01, 0,
                                      0,            0,             0, 1;
        
        transformation_matrix << 7.967514e-03, -9.999679e-01, -8.462264e-04, -1.377769e-02,
                                -2.771053e-03,  8.241710e-04, -9.999958e-01, -5.542117e-02,
                                 9.999644e-01,  7.969825e-03, -2.764397e-03, -2.918589e-01,
                                            0,             0,             0,             1;

        // 提取前三行前三列
        Project_matrix_3x3 = Project_matrix.block<3, 3>(0, 0);
        rotation_matrix_3x3 = rotation_matrix.block<3, 3>(0, 0);

        // 求逆並相乘
        result = rotation_matrix_3x3.inverse() * Project_matrix_3x3.inverse();                

        RR = transformation_matrix.block<3, 3>(0, 0); 
        tt << transformation_matrix(0, 3), transformation_matrix(1, 3), transformation_matrix(2, 3);

        matrix_3Dto2D = Project_matrix * rotation_matrix * transformation_matrix;
    }
    else if(sequence_number == 3){
        Project_matrix << 7.215377e+02, 0.000000e+00, 6.095593e+02, 0.000000e+00, 
                          0.000000e+00, 7.215377e+02, 1.728540e+02, 0.000000e+00, 
                          0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00;
        
        rotation_matrix << 9.999239e-01, 9.837760e-03, -7.445048e-03, 0, 
                          -9.869795e-03, 9.999421e-01, -4.278459e-03, 0,
                           7.402527e-03, 4.351614e-03,  9.999631e-01, 0,
                                      0,            0,             0, 1;

        transformation_matrix << 7.533745e-03, -9.999714e-01, -6.166020e-04, -4.069766e-03,
                                 1.480249e-02,  7.280733e-04, -9.998902e-01, -7.631618e-02,
                                 9.998621e-01,  7.523790e-03,  1.480755e-02, -2.717806e-01,
                                            0,             0,             0,             1;

        // 提取前三行前三列
        Project_matrix_3x3 = Project_matrix.block<3, 3>(0, 0);
        rotation_matrix_3x3 = rotation_matrix.block<3, 3>(0, 0);

        RR = transformation_matrix.block<3, 3>(0, 0); 
        tt << transformation_matrix(0, 3), transformation_matrix(1, 3), transformation_matrix(2, 3);

        // 求逆並相乘
        result = rotation_matrix_3x3.inverse() * Project_matrix_3x3.inverse();            
        matrix_3Dto2D = Project_matrix * rotation_matrix * transformation_matrix;
    }
    else{
        Project_matrix << 7.070912e+02, 0.000000e+00, 6.018873e+02, 0.000000e+00, 
                          0.000000e+00, 7.070912e+02, 1.831104e+02, 0.000000e+00, 
                          0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00;
        
        rotation_matrix << 9.999280e-01, 8.085985e-03, -8.866797e-03, 0,
                          -8.123205e-03, 9.999583e-01, -4.169750e-03, 0,
                           8.832711e-03, 4.241477e-03,  9.999520e-01, 0,
                                      0,            0,             0, 1;
        
        transformation_matrix << 7.027555e-03, -9.999753e-01,  2.599616e-05, -7.137748e-03,
                                -2.254837e-03, -4.184312e-05, -9.999975e-01, -7.482656e-02,
                                 9.999728e-01,  7.027479e-03, -2.255075e-03, -3.336324e-01,
                                            0,             0,             0,             1; 


        // 提取前三行前三列
        Project_matrix_3x3 = Project_matrix.block<3, 3>(0, 0);
        rotation_matrix_3x3 = rotation_matrix.block<3, 3>(0, 0);

        RR = transformation_matrix.block<3, 3>(0, 0); 
        tt << transformation_matrix(0, 3), transformation_matrix(1, 3), transformation_matrix(2, 3);

        // 求逆並相乘
        result = rotation_matrix_3x3.inverse() * Project_matrix_3x3.inverse();             
        matrix_3Dto2D = Project_matrix * rotation_matrix * transformation_matrix;
    }


    message_filters::Subscriber<sensor_msgs::PointCloud2> subLaserCloud(nh , "/velodyne_points", 100);
    message_filters::Subscriber<sensor_msgs::Image> subImageLeft(nh, "/image_left", 100);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> SyncPolicy;
    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(30), subLaserCloud, subImageLeft);

    sync.registerCallback(boost::bind(&velodyneHandler, _1, _2));

    pubLaserCloudFiltered = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points_filtered", 100);
    pubEdgePoints = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_edge", 100);
    pubSurfPoints = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf", 100); 
    pubImage = nh.advertise<sensor_msgs::Image>("/processed_image", 100);

    std::thread laser_processing_process{laser_processing};

    ros::spin();

    return 0;
}