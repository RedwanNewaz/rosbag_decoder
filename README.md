# rosbag_decoder
For autonomous vehicles it is essential to have a clear view and understanding of their surroundings to be able to navigate safely. To generate this view, out of many different sensors we are interested to use Lidar sensor since it's performance is mostly invariant to all weather and lighting conditions. On the other hand, we rely on control area network (CAN) bus that capture the driving behavior of an agent using multi-modal sensor outputs. To be able to learn the driving behavior of an agent it is necessary to synchronize the data delivered by these sensors so that we can process them for the neural networks.
## variable frequency
Each sensor has its own frequency for detection and extraction of scene information. CAN sensors are faster in response but noisy while Lidar is accurate but slower, both of which have their pros and cons. The synchronization of data should ensure a higher certainty about the existence of perception and actions. 
![](https://github.com/RedwanNewaz/rosbag_decoder/blob/master/include/canvas.png)
## Requirement
* rosbag  
* cv_bridge
* pcl_ros
* opencv
* kvaser (Autoware)

## Launch file
In decoder.launch file the argument "nodename" should be unique. All the parameters depends on the nodename. 
It is possible to simultaneously run the program in multiple terminals by varing the nodename. To identify the output from multiple decoder nodes, the output file names are automatically modified according to nodename. To the best practice, nodename should be the base direcory name of bag files. Thus, the output would be the 3 files staring with nodename.
``` xml
<?xml version="1.0"?>
<launch>
  <arg name="nodename" default="ELECOM_20160319_163019_172519"/> 
  <node name="$(arg nodename)" pkg="rosbag_decoder" type="rosbag_decoder_node"
        output="screen" args="$(arg nodename)">
  	<rosparam command="load" file="param.yaml" />
  </node>
</launch>
```

## Parameter Tuning 
For each topic, the rosnode can decode all the bags in a given direcory and output a single file.
First you need to define appropriate rostopic names and the direcory of bag files. Therefore, you may make 
separate folder to decode single rosbag file. Here, min_max normalization is used for normalizing can_bus data. However,
getting the parameter max_value and min_value are tricky. If you don't know these parameters, you can get to know 
these parameters by setting the update true with random max_value, min_value. The program is then display the gloabl max and global min
in the terminal by decoding all the files of a given directory. Finally, you need to redo your experiment with updated parameters.
``` ruby
topics:
  can_raw: "/can_raw"
  cloud_in: "/points_raw"
  camera_in: "/camera0/image_raw"
  bag_in: "/home/redwan/ELECOM/20160319_163019_172519/*.bag"

can_bus:
  max_value: [375,  79, 127,  0,  7774, 0,  0,  239,  14542,  14557,  14532,  14550,  65, 25200,  255,  619]
  min_value: [-385, 0,  -128, 0,  0,  0,  0,  0,  6767, 6767, 6767, 6767, -99,  0,  0,  0]
  update: false

output:
  can_file: "/home/redwan/Videos/dataset/can_video.avi"
  cloud_file: "/home/redwan/Videos/dataset/depth_video.avi"
  camera_file: "/home/redwan/Videos/dataset/camera_video.avi"
  visualize: true

```
As I explain earlier the output would be 3 files, e.g. "nodename"_can_video.avi, "nodename"_depth_video.avi, "camera"_video.avi. Furthermore, you can also see realtime operations if the parameter visualize is true. However, for the fast processing it is recommended to set this parameter false.
