#ifndef CONVERTER_H
#define CONVERTER_H
#include<iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include<Eigen/Dense>
#include<Eigen/Core>
#include "kvaser/CANPacket.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include <math.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <deque>
#include <vector>
#include <mutex>

using namespace std;
using namespace cv;
class converter;
class can_converter;
class img_converter;
class converter
{
public:
  converter(int row, int col, string &filename);
  ~converter();
  void write_frame(Mat &frame, bool convert);
  bool display;
protected:
  void view(Mat &frame);

private:
  int rows, cols;
  string file;
  VideoWriter *video;
  ros::NodeHandle nh;
};

class can_converter{
private:
  long imgCount;
  int rows,cols;
  bool update_param;
  mutex mu;
  ros::NodeHandle nh;
  vector<deque<float>> img;
  vector<float>max_val,min_val;
  converter *vclass;


protected:
  void update(float *data);

public:
  can_converter(int row, int col);
  ~can_converter();
  void print();
  void write_video();
  void can_callback(const kvaser::CANPacket::ConstPtr& msg );
};
class img_converter{
private:
  cv::Mat img;
  int imgCount,rows,cols;
  converter *vclass;
  ros::NodeHandle nh;
public:
  img_converter(int row,int col);
  ~img_converter();
  void print();
  void img_callback(const sensor_msgs::Image::ConstPtr &msg);
  void write_video();

};
class cloud_converter{
private:
    float v_fov[2],v_res,v_res_rad,h_res,h_res_rad;
    int imgCount,rows,cols;
    cv::Mat img;
    converter *vclass;
    ros::NodeHandle nh;
public:
  cloud_converter(int row, int col);
  ~cloud_converter();
  void pcl_callback(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void print();
  void write_video();

};

#endif // CONVERTER_H
