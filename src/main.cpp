#include <ros/ros.h>
#include <glob.h>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include<iostream>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include "sensor_msgs/Image.h"
#include "kvaser/CANPacket.h"
#include "sensor_msgs/PointCloud2.h"
#include <std_msgs/String.h>
#include "converter.h"
#include <unistd.h>// for usleep
#include <chrono>
#include "helper.h"
using namespace std;
typedef sensor_msgs::Image type_img;
typedef kvaser::CANPacket type_can;
typedef sensor_msgs::PointCloud2 type_cloud;

//read all topics using functor
class decoder{
private:
  string node;
  can_converter *can_decoder;
  img_converter *img_decoder;
  cloud_converter *cloud_decoder;
public:
  decoder(string &nod):node(nod){
    can_decoder=new can_converter(16,100,node);
    img_decoder=new img_converter(480,640,node);
    cloud_decoder=new cloud_converter(40,1030,node);
  }
  ~decoder(){
    delete  can_decoder;
    delete  img_decoder;
    delete  cloud_decoder;
    cout<<"decoder class deleted"<<endl;
  }

  void operator()(const sensor_msgs::Image::ConstPtr& msg){
    if (msg == NULL) return;
    img_decoder->img_callback(msg);
  }
  void operator()(const kvaser::CANPacket::ConstPtr& msg){
    if (msg == NULL) return;
    can_decoder->can_callback(msg);

  }
  void operator()(const sensor_msgs::PointCloud2::ConstPtr& msg){
    if (msg == NULL) return;
    cloud_decoder->pcl_callback(msg);
    img_decoder->write_video();
    can_decoder->write_video();
    cloud_decoder->write_video();

  }

};

int main(int argc, char* argv[]){

  string prefix(argv[1]);
  ros::init(argc, argv, "rosbag_converter");
  ros::NodeHandle nh;
  decoder dec(prefix);
  prefix="/"+prefix;
  std::string camera,can_bus,point_cloud,bag_dir;
  nh.param<std::string>(prefix+"/topics/camera_in", camera, "/camera1/image_raw");
  nh.param<std::string>(prefix+"/topics/can_bus", can_bus, "/can_raw");
  nh.param<std::string>(prefix+"/topics/cloud_in", point_cloud, "/points_raw");
  nh.param<std::string>(prefix+"/topics/bag_in", bag_dir, "");

  std::vector<std::string> topics;
  topics.push_back(camera);
  topics.push_back(can_bus);
  topics.push_back(point_cloud);

  if(bag_dir.empty())return 0;
  std::vector<std::string> bags=glob(bag_dir);

  for(string &bagname:bags)
  {
    ROS_INFO("Reading rosbag: \n [%s] \n it may take a while, please wait....",bagname.c_str());

    auto t1 = std::chrono::high_resolution_clock::now();
    rosbag::Bag bag;
    bag.open(bagname, rosbag::bagmode::Read);


    rosbag::View view(bag, rosbag::TopicQuery(topics)) ;
    auto t2 = std::chrono::high_resolution_clock::now();
    auto int_s = std::chrono::duration_cast<std::chrono::seconds>(t2 - t1);

    ROS_INFO_STREAM("Decoding rosbag takes :[ "<<int_s.count()<<" ] sec");



    int count(0);
    for(auto m:view){
      dec(m.instantiate<type_can>());
      dec(m.instantiate<type_img>());
      dec(m.instantiate<type_cloud>());
      if(ros::isShuttingDown())break;

      if(++count%500){
      cout<<"rosbag: " <<view.size()<<"/"<<count<<"\xd";
      cout.flush();
      }

    }
    auto t3 = std::chrono::high_resolution_clock::now();
    auto int_s2 = std::chrono::duration_cast<std::chrono::minutes>(t3 - t2);

  ROS_INFO_STREAM("Decoding completed ["<<int_s2.count()<<"] min");
  }

usleep(5e6);

return 0;


}
