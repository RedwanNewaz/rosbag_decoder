#include "converter.h"

converter::converter(int row, int col, string &filename):rows(row),cols(col)
{
   video=new VideoWriter(filename,CV_FOURCC('M','J','P','G'),10, Size(cols,rows));
   file=filename;
   nh.getParam("/rosbag_decoder/output/visualize",display);
}

converter::~converter()
{
  ROS_WARN(" releasing video");
  video->release();
  if(display)
  destroyAllWindows();
  delete video;
}

void converter::write_frame(Mat &frame,bool convert)
{
  if (frame.empty())
       return;
  if(convert){
  cv::Mat img;
  cv::cvtColor(frame,img,COLOR_GRAY2BGR);
  video->write(img);
  if(display)
  view(img);
  }
  else{
    cv::Mat img(frame);
    video->write(img);
    if(display)
    view(img);
  }
}

void converter::view(Mat &frame)
{
  cv::imshow( file, frame );
  cv::waitKey(1);
}


// can converter
void can_converter::update(float *data)
{
  for(int j(0);j<rows;j++){
    deque<float>arr(img[j]);
    float X = (data[j] - min_val[j]) / (max_val[j] - min_val[j]);
    arr.pop_front();
    arr.push_back(X);
    img[j]=arr;
    if(update_param){
      if(data[j]>max_val[j])
        max_val[j]=data[j];
      else if (data[j]<min_val[j])
        min_val[j]=data[j];
    }
  }


  ++imgCount;

}

can_converter::can_converter(int row, int col):rows(row),cols(col)
{

  img.resize(rows);
  deque<float>zero(0,cols);
  for(int i(0);i<rows;i++){
    img[i].resize(cols);
    img[i]=zero;
  }
  nh.getParam("/rosbag_decoder/can_bus/max_value",max_val);
  nh.getParam("/rosbag_decoder/can_bus/min_value",min_val);
  nh.getParam("/rosbag_decoder/can_bus/update",update_param);

  string can_file;
  nh.getParam("/rosbag_decoder/output/can_file",can_file);
  ROS_INFO_STREAM("can_node: "<<can_file);

  vclass=new converter(rows,cols,can_file);
  imgCount=0;
}

can_converter::~can_converter()
{
  if(update_param)
    print();
  ROS_WARN("PUBLISHING CAN VIDEO");
  delete vclass;
}

void can_converter::print()
{
  //std::cout<<"total opencv image "<<imgCount<<std::endl;
  ROS_INFO("MAX VALUES");
  for (auto &m:max_val){
    cout<<m<<",\t";
  }
  cout<<endl;

  ROS_INFO("MIN VALUES");
  for (auto &m:min_val){
    cout<<m<<",\t";
  }
  cout<<endl;
  ROS_INFO_STREAM("total can image "<<imgCount);

}

void can_converter::write_video()
{
  // write to video recorder
  cv::Mat grayImg(rows,cols,CV_8U);
  for (int m(0);m<rows;m++){
    auto deq=img[m];
    for (int n(0);n<cols;n++)
      grayImg.at<uchar>(m,n)=uchar(255-255*deq[n]);
  }
  vclass->write_frame(grayImg,true);
}

void can_converter::can_callback(const kvaser::CANPacket::ConstPtr&msg)
{
  unsigned short w;
  static int enc_sum;
  short diff;
  static short steer,shift,speed2,speed3,brake;
  static char  speed;
  static short enc,enc_p,enc_diff;
  static short wheel1,wheel2,wheel3,wheel4;
  static short gyro,accy,accz;
  static char accx;

 // ROS_INFO_STREAM("can_raw "<<msg->header.seq);

  int changed=0;

  if(msg->id==0x24){
    w=msg->dat[0]*256+msg->dat[1];
    gyro=w;
    w=msg->dat[2]*256+msg->dat[3];
    accx=w;
    w=msg->dat[4]*256+msg->dat[5];
    accy=w;
    w=msg->dat[7];
    accz=w;
    changed=1;
  }
  if(msg->id==0x25){
    w=msg->dat[0]*4096+msg->dat[1]*16;
    steer=w;
    steer=steer/16;
    changed=1;
  }

  if(msg->id==0xaa){
    w=msg->dat[0]*256+msg->dat[1];
    wheel1=w;
    w=msg->dat[2]*256+msg->dat[3];
    wheel2=w;
    w=msg->dat[4]*256+msg->dat[5];
    wheel3=w;
    w=msg->dat[6]*256+msg->dat[7];
    wheel4=w;
    changed=1;
  }
  if(msg->id==0xb4){
    w=msg->dat[5]*256+msg->dat[6];
    speed3=w;
    changed=1;
  }
  if(msg->id==0x224){
    w=msg->dat[4]*256+msg->dat[5];
    brake=w;
    changed=1;
  }
  if(msg->id==0x127){
    shift=msg->dat[3];
    speed=msg->dat[4];
    changed=1;
  }
  if(msg->id==0x230){
    w=msg->dat[0]*256+msg->dat[1];
    enc_p=enc;
    enc=w;
    diff=enc-enc_p;
    enc_diff=diff;
    enc_sum+=diff;
    changed=1;
  }
  if(changed){

    float dat[]={steer,shift,speed,speed2,speed3,enc_sum,enc_diff,brake,
            wheel1,wheel2,wheel3,wheel4,
            accx,accy,accz,gyro};
    update(dat);
  }


}

// image converter
img_converter::img_converter(int row, int col):rows(row),cols(col)
{
  imgCount=0;
  string camera_file;
  nh.getParam("/rosbag_decoder/output/camera_file",camera_file);

  vclass=new converter(rows,cols,camera_file);
}

img_converter::~img_converter()
{
  ROS_INFO("deleteing image converter class");
  delete vclass;
}

void img_converter::print()
{
  ROS_INFO_STREAM("total can image "<<imgCount);
}

void img_converter::img_callback(const sensor_msgs::Image::ConstPtr &msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Size newsize(cols,rows);
    cv::resize(cv_ptr->image,img,newsize);//resize image
    ++imgCount;

  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

}

void img_converter::write_video()
{
  vclass->write_frame(img,false);
}


//point cloud converter
cloud_converter::cloud_converter(int row, int col):rows(row),cols(col)
{
      v_fov[0] =-30.67;
      v_fov[1] =10.67;
      v_res    =1.33;
      v_res_rad=v_res*M_PI/180;
      h_res    =0.35;
      h_res_rad=h_res*M_PI/180;
      img=cv::Mat::zeros(rows,cols,CV_8U);
      string cloud_file;
      nh.getParam("/rosbag_decoder/output/cloud_file",cloud_file);

      vclass=new converter(rows,cols,cloud_file);
}

cloud_converter::~cloud_converter()
{
  ROS_WARN("PUBLISHING DEPTH VIDEO");
  delete vclass;
}

void cloud_converter::pcl_callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*msg,pcl_pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

  int n=temp_cloud->size();

  auto clip=[](float d){return (d>100)?100:d;};
  auto minmax=[](float X){return uchar(255-255*X /100.0);};

  float x,y,z,d;

  for(int i(0);i<n;i++){
    x=temp_cloud->points[i].x;
    y=temp_cloud->points[i].y;
    z=temp_cloud->points[i].z;
    d=sqrt(x*x + y*y+ z*z);
    float x_min = -360.0 / h_res / 2 ;
    int x_img=atan2(-y,x)/h_res_rad-x_min;
    float y_min = v_fov[0] / v_res;
    int y_img=atan2(z,d)/v_res_rad-y_min;
    d=(d<0)?0:d;
    float pixel_value=clip(d);
    img.at<uchar>(y_img, x_img)=minmax(pixel_value);


  }


  imgCount++;


}

void cloud_converter::print(){
  cv::imshow( "Frame", img );
  cv::waitKey(1);


}

void cloud_converter::write_video()
{
  cv::Mat dst;               // dst must be a different Mat
  cv::flip(img, dst, 0);
  vclass->write_frame(dst,true);
}
