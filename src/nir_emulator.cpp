#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h> //rsd - was giving error on swami


#include <sensor_msgs/PointCloud2.h>




int main( int argc, char** argv ){

  ros::init( argc, argv, "NIROverlay" );

  ros::NodeHandle nh, nhp("~");
  ros::Rate loop_rate(10);
  ros::Publisher    pub_cog=nh.advertise< pcl::PointCloud<pcl::PointXYZI> >("/NIROverlay/cog", 1);
  pcl::PointCloud<pcl::PointXYZI> markers;
  pcl::PointXYZI xyzi;
  xyzi.x = 0.0;
  xyzi.y = 0.0;
  xyzi.z = 0.0;
  xyzi.intensity = 1.0;
  markers.push_back( xyzi );
  xyzi.x = 0.03;
  xyzi.y = 0.03;
  xyzi.z = 0.03;
  xyzi.intensity = 1.0;
  markers.push_back( xyzi );
  int seq = 0;
  while(ros::ok()){
 	  std_msgs::Header header;
      header.stamp = ros::Time::now();
      header.seq = seq++;
      header.frame_id = std::string( "/camera_color_optical_frame" );
		
      markers.header = pcl_conversions::toPCL( header );
	  pub_cog.publish(markers);
	  loop_rate.sleep();
	  ros::spinOnce();
  }
  return 0;

}
