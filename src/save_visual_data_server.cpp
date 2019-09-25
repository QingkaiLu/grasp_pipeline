#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>


#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <string>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <grasp_pipeline/SaveVisualData.h>
#include <math.h>

using namespace cv;
using namespace std;
// Max depth in mm
#define MAX_DEPTH 8000
#define MAX_16U 65535

/**
 * Get depth image from point cloud.
 */
void getDepthFromPcd(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &raw_cloud_ptr, Mat &depth_img)
{
    depth_img.create(raw_cloud_ptr->height, raw_cloud_ptr->width, CV_32FC1);
    //ROS_INFO("(raw_cloud_ptr->height, width) = (%d, %d)", raw_cloud_ptr->height , raw_cloud_ptr->width);
    //ROS_INFO("Iterating through cloud");
    for(int r = 0;r < raw_cloud_ptr->height; ++r){
        for (int c = 0; c < raw_cloud_ptr->width; ++c){
            float depth_value = raw_cloud_ptr->at(c, r).z;
            if(!isnan(depth_value) && !isinf(depth_value)){
                depth_img.at<float>(r,c) = depth_value;
            }
            else{
                depth_img.at<float>(r,c) = 0.;
            }
        }
    }
    return;
}

/**
 * Convert the depth image to a format suitable for saving
 */
inline void convertDepthImageToSaveFormat(Mat& depth_image_in, Mat& depth_image_out)
{
    depth_image_out.create(depth_image_in.size(), CV_16UC1);
    depth_image_in.convertTo(depth_image_out, CV_16UC1);
}

/**
 * Convert the depth image from the save format to a useful float representation
 */
inline void convertDepthImageFromSaveFormatToFloat(Mat& depth_image_in, Mat& depth_image_out)
{
    depth_image_out.create(depth_image_in.size(), CV_32FC1);
    depth_image_in.convertTo(depth_image_out, CV_32FC1);
}

/**
 * Convenience function to convert a depth image into a suitable format for saving and writes it to
 * disk
 *
 * @param depth_image Image to save
 * @param depth_image_save_path The location to save the image
 */
inline void saveDepthImage(cv::Mat& depth_image, std::string depth_image_save_path)
{
  cv::Mat depth_save_image;
  convertDepthImageToSaveFormat(depth_image, depth_save_image);
  imwrite(depth_image_save_path, depth_save_image);
}

/**
 * Read a depth image from disk and return it as a floating point image
 *
 * @param depth_image floating point image returned by the function
 * @param depth_image_read_path path to read image form disk
 */
inline void readDepthImage(cv::Mat& depth_image, std::string depth_image_read_path)
{
  cv::Mat depth_read_image;
  depth_read_image = cv::imread(depth_image_read_path, CV_LOAD_IMAGE_UNCHANGED);
  convertDepthImageFromSaveFormatToFloat(depth_read_image, depth_image);
}


/**
 * Convert the rgb image to a format suitable for saving
 */
inline void convertRgbImageToSaveFormat(Mat& rgb_image_in, Mat& rgb_image_out)
{
    rgb_image_out.create(rgb_image_in.size(), CV_16UC3);
    rgb_image_in.convertTo(rgb_image_out, CV_16UC3);
}

/**
 * Convert the rgb image from the save format to a useful float representation
 */
inline void convertRgbImageFromSaveFormatToFloat(Mat& rgb_image_in, Mat& rgb_image_out)
{
    rgb_image_out.create(rgb_image_in.size(), CV_32FC3);
    rgb_image_in.convertTo(rgb_image_out, CV_32FC3);
}

/**
 * Convenience function to convert a rgb image into a suitable format for saving and writes it to
 * disk
 *
 * @param rgb_image Image to save
 * @param rgb_image_save_path The location to save the image
 */
inline void saveRgbImage(cv::Mat& rgb_image, std::string rgb_image_save_path)
{
  cv::Mat rgb_save_image;
  convertRgbImageToSaveFormat(rgb_image, rgb_save_image);
  imwrite(rgb_image_save_path, rgb_save_image);
}
/**
 * Read a rgb image from disk and return it as a floating point image
 *
 * @param rgb_image floating point image returned by the function
 * @param rgb_image_read_path path to read image form disk
 */
inline void readRgbImage(cv::Mat& rgb_image, std::string rgb_image_read_path)
{
  cv::Mat rgb_read_image;
  rgb_read_image = cv::imread(rgb_image_read_path, CV_LOAD_IMAGE_UNCHANGED);
  convertRgbImageFromSaveFormatToFloat(rgb_read_image, rgb_image);
}

//void plotFingerTipPalmCenter(cv::Mat& rgb_img, double* finger_tip_locs, double* palm_loc)
void plotFingerTipPalmCenter(cv::Mat& rgb_img, const vector<double> &finger_tip_locs, const vector<double> &palm_loc)
{
    Point palm_loc_pnt(palm_loc[0], palm_loc[1]);
    circle(rgb_img, palm_loc_pnt, 6, Scalar(255, 0, 0));
    int fingers_num = 4;
    for(int i = 0; i < fingers_num; ++i)
    {
        //cout << "finger: " << finger_tip_locs[2*i] << " " << finger_tip_locs[2*i + 1] << endl; 
        Point finger_tip_loc_pnt(finger_tip_locs[2*i], finger_tip_locs[2*i + 1]);
        circle(rgb_img, finger_tip_loc_pnt, 6, Scalar(0, 0, 255));
        //line(rgb_img, palm_loc_pnt, finger_tip_loc_pnt, Scalar(0, 255, 0), 2);
    }
    return;
}

/**
 * Get the normals of sd cloud.
 * @param sd_cloud_ptr The sd cloud pointer.
 * @return the normal cloud pointer.
 */
pcl::PointCloud<pcl::Normal>::Ptr getSdCloudNormals(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &sd_cloud_ptr)
{
    pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setInputCloud(sd_cloud_ptr);

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
    ne.setSearchMethod(tree);

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_ptr(new pcl::PointCloud<pcl::Normal>);

    int normals_k_neighbors = 50;
    ne.setKSearch(normals_k_neighbors);

    ne.compute(*cloud_normals_ptr);
    
    return cloud_normals_ptr;
}

void createDirectory(const string &file_path)
{
    boost::filesystem::path dir(file_path);
    std::cout << file_path << " " << boost::filesystem::exists(dir) << std::endl;
    if (!boost::filesystem::exists(dir))
    {
        boost::filesystem::create_directory(dir);
        string dir_creation_info = "Directory: " + file_path + " created!";
        ROS_INFO("%s\n", dir_creation_info.c_str());
    }
    return;
}

/**
 * Save point cloud, rgb and depth image.
 * @param req SaveVisualData srv request.
 * @param res SaveVisualData srv response.
 * @return successful saving or not.
 */
bool saveVisualData(grasp_pipeline::SaveVisualData::Request &req,
                grasp_pipeline::SaveVisualData::Response &res)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(req.scene_cloud, *raw_cloud_ptr);
    // ASCII
    // pcl::io::savePCDFile(req.scene_cloud_save_path, *raw_cloud_ptr, false);
    // pcl::io::savePCDFileASCII(req.scene_cloud_save_path, *raw_cloud_ptr);
    // Binary
    pcl::io::savePCDFile(req.scene_cloud_save_path, *raw_cloud_ptr, true);
    
    // pcl::PointCloud<pcl::Normal>::Ptr cloud_normal_ptr = getSdCloudNormals(raw_cloud_ptr);
    // pcl::toROSMsg(*cloud_normal_ptr, res.scene_cloud_normal);
    // pcl::io::savePCDFile(req.cloud_normal_save_path, *cloud_normal_ptr, true);

    Mat rgb_image;
    cv_bridge::CvImagePtr cv_rgb_msg = cv_bridge::toCvCopy(req.scene_rgb_img, sensor_msgs::image_encodings::BGR8);
    imwrite(req.rgb_image_save_path, cv_rgb_msg->image);

    Mat depth_image;
    cv_bridge::CvImagePtr cv_depth_msg = cv_bridge::toCvCopy(req.scene_depth_img);
    cv_depth_msg->image.convertTo(depth_image, CV_32FC1);
    // cv::imshow("depth msg", depth_image*(1./MAX_DEPTH));
    // cv::waitKey();
    saveDepthImage(depth_image, req.depth_image_save_path);
    // imshow("depth_image_pre_save", depth_image * (1. / MAX_DEPTH));
    // cv::Mat depth_image_read;
    // readDepthImage(depth_image_read, req.depth_image_save_path);
    
    // Print max and min depth.
    // double min_value = 100., max_value = 0.;
    // minMaxLoc(depth_image, &min_value, &max_value);
    // std::cout << "min_value: " << min_value << " max_value: " << max_value << std::endl;

    if (req.real_kinect2)
    {
        Mat sd_rgb_image;
        cv_bridge::CvImagePtr cv_sd_rgb_msg = cv_bridge::toCvCopy(req.scene_sd_rgb_img, sensor_msgs::image_encodings::BGR8);
        imwrite(req.sd_rgb_image_save_path, cv_sd_rgb_msg->image);

        Mat sd_depth_image;
        cv_bridge::CvImagePtr cv_sd_depth_msg = cv_bridge::toCvCopy(req.scene_sd_depth_img);
        cv_sd_depth_msg->image.convertTo(sd_depth_image, CV_32FC1);
        saveDepthImage(sd_depth_image, req.sd_depth_image_save_path);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_sd_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(req.scene_sd_cloud, *raw_sd_cloud_ptr);
        pcl::io::savePCDFile(req.scene_sd_cloud_save_path, *raw_sd_cloud_ptr, true);

        // pcl::PointCloud<pcl::Normal>::Ptr sd_cloud_normal_ptr = getSdCloudNormals(raw_sd_cloud_ptr);
        // pcl::toROSMsg(*sd_cloud_normal_ptr, res.scene_sd_cloud_normal);
        // pcl::io::savePCDFile(req.sd_cloud_normal_save_path, *sd_cloud_normal_ptr, true);
    }
    
    // bool view_normals = false;
    // if (view_normals) 
    // {
    //     boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Cloud Viewer"));
    //     viewer->setBackgroundColor (0, 0, 0);
    //     pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(raw_sd_cloud_ptr);
    //     viewer->addPointCloud<pcl::PointXYZRGB> (raw_sd_cloud_ptr, rgb, "scene cloud");
    //     viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "scene cloud");
    //     viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (raw_sd_cloud_ptr, sd_cloud_normal_ptr, 
    //                                                                 10, 0.05, "scene_cloud_normals");
    //     viewer->addCoordinateSystem (1.0);
    //     viewer->initCameraParameters ();
    // 
    //     while (!viewer->wasStopped ())
    //         viewer->spinOnce (100);
    // }

    res.save_visual_data_success = true;
    return true;
}

/**
 * Callback method to debug depth image formatting and saving compared to PCD file conversion
 *
 * @param msg Incoming depth image msg
 */
void testDepthImageCB(const sensor_msgs::ImageConstPtr& depth_msg)
{
  try
  {
    cv_bridge::CvImagePtr cv_depth_msg = cv_bridge::toCvCopy(depth_msg);
    cv::Mat depth_frame;
    cv_depth_msg->image.convertTo(depth_frame, CV_32FC1);
    double min_val=1000000, max_val=0.0;
    cv::minMaxLoc(depth_frame, &min_val, &max_val);
    cv::imshow("depth_img_u", cv_depth_msg->image);
    cv::imshow("depth_img", depth_frame/max_val);
    ROS_INFO_STREAM("Depth image dimensions " << depth_frame.rows << ", " << depth_frame.cols);
    ROS_INFO_STREAM("Min value is " << min_val << "\tmax value is " << max_val);
    // Test reading and writing of images
    saveDepthImage(depth_frame, "/home/thermans/sandbox/test_depth.ppm");
    cv::Mat depth_frame_from_disk;
    readDepthImage(depth_frame_from_disk, "/home/thermans/sandbox/test_depth.ppm");
    cv::imshow("depth_img from disk", depth_frame_from_disk/max_val);
    cv::waitKey(0);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Failed to convert depth image to opencv image! %s" , e.what());
  }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "save_visual_data_server");
    ros::NodeHandle n;
    
    ros::ServiceServer service = n.advertiseService("/save_visual_data", saveVisualData);
    ROS_INFO("Service save_visual_data_server:");
    ROS_INFO("Ready to save visual data.");

    // image_transport::ImageTransport it(n);
    // image_transport::Subscriber sub = it.subscribe("/kinect2/sd/image_depth_rect", 1, testDepthImageCB);
    ros::spin();

    return 0;
}
