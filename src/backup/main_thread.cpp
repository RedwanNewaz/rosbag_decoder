#include <ros/ros.h>
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
#include <pthread.h>
#include "helper.h"
using namespace std;
typedef sensor_msgs::Image type_img;
typedef kvaser::CANPacket type_can;
typedef sensor_msgs::PointCloud2 type_cloud;
//read all topics using functor
class decoder{
private:
  can_converter *can_decoder;
  img_converter *img_decoder;
  cloud_converter *cloud_decoder;
public:
  decoder(){
    can_decoder=new can_converter(16,100);
    img_decoder=new img_converter(480,640);
    cloud_decoder=new cloud_converter(40,1030);
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
// threading
  std::vector<std::string> topics;
void *rosbag_thread(void *x_void_ptr)
{
//pthread_mutex_lock( &pmutex );
/* increment x to 100 */
string bagname = *(string *)x_void_ptr;

string base_file=get_basename(bagname);
ROS_INFO_STREAM(base_file.c_str());
decoder dec;

//pthread_mutex_unlock( &pmutex );

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

return NULL;

}
int main(int argc, char* argv[]){

  ros::init(argc, argv, "rosbag_converter_threaded");
  ros::NodeHandle nh;

  std::string camera,can_bus,point_cloud,bag_dir;
  nh.param<std::string>("/rosbag_decoder/topics/camera_in", camera, "/camera1/image_raw");
  nh.param<std::string>("/rosbag_decoder/topics/can_bus", can_bus, "/can_raw");
  nh.param<std::string>("/rosbag_decoder/topics/cloud_in", point_cloud, "/points_raw");
  nh.param<std::string>("/rosbag_decoder/topics/bag_in", bag_dir, "");


  topics.push_back(camera);
  topics.push_back(can_bus);
  topics.push_back(point_cloud);

  if(bag_dir.empty())return 0;
  std::vector<std::string> bags=glob(bag_dir);
  int max_thread,thread_count(0);
  nh.getParam("/rosbag_decoder/topics/num_thread",max_thread);
  pthread_t threads[max_thread];
  string can_name,delete_name("can_video.avi");
  nh.getParam("/rosbag_decoder/output/can_file",can_name);
  int b(can_name.length()),e(delete_name.length());
  can_name.erase(b-e,b);
  ROS_INFO_STREAM(can_name);


  for(string &bagname:bags)
  {


    string can_base_file=can_name+get_basename(bagname)+"_can_video.avi";
    nh.setParam("/rosbag_decoder/output/can_file",can_base_file);
    string camera_base_file=can_name+get_basename(bagname)+"_camera_video.avi";
    nh.setParam("/rosbag_decoder/output/camera_file",camera_base_file);
    string depth_base_file=can_name+get_basename(bagname)+"_depth_video.avi";
    nh.setParam("/rosbag_decoder/output/depth_file",depth_base_file);

    pthread_create(&threads[thread_count], NULL, rosbag_thread, &bagname);

    if(++thread_count>=max_thread)break;
    usleep(1e6);
  }

  for(int t(0);t<max_thread;t++)
    pthread_join(threads[t], NULL);






return 0;


}

