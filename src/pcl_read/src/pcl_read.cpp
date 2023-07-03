#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("PointCloud Viewer"));

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*cloud_msg, *cloud);

  viewer->removeAllPointClouds();
  viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud");
  viewer->spinOnce();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcl_visualization");
  ros::NodeHandle nh;

  // Read the bag file
  rosbag::Bag bag;
  bag.open("/home/desecy/all.bag", rosbag::bagmode::Read);

  std::vector<std::string> topics;
  topics.push_back("/rslidar_points");

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  for (rosbag::View::iterator it = view.begin(); it != view.end(); ++it)
  {
    rosbag::MessageInstance msg = *it;
    sensor_msgs::PointCloud2ConstPtr cloud_msg = msg.instantiate<sensor_msgs::PointCloud2>();

    if (cloud_msg != NULL)
    {
      cloudCallback(cloud_msg);
      ros::spinOnce();
      ros::Duration(0.05).sleep();  // Adjust the delay between displaying consecutive point clouds
    }
  }

  bag.close();

  return 0;
}
