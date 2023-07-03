#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>

// 回调函数：接收颜色相机图像
void colorImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        // 将ROS图像消息转换为OpenCV图像
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        // 显示颜色相机图像
        cv::imshow("Color Image", cv_ptr->image);
        cv::waitKey(1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

// 回调函数：接收深度相机图像
void depthImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        // 将ROS图像消息转换为OpenCV图像
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);

        // 显示深度相机图像
        cv::imshow("Depth Image", cv_ptr->image);
        cv::waitKey(1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "my_node");
    ros::NodeHandle nh;

    // 创建订阅颜色相机图像的订阅者
    ros::Subscriber color_image_sub = nh.subscribe("/camera/color/image_raw", 1, colorImageCallback);

    // 创建订阅深度相机图像的订阅者
    ros::Subscriber depth_image_sub = nh.subscribe("/camera/depth/image_raw", 1, depthImageCallback);

    // 初始化OpenCV图形界面
    cv::namedWindow("Color Image");
    cv::namedWindow("Depth Image");

    // 打开ROS包
    rosbag::Bag bag;
    try
    {
        bag.open("/home/desecy/all.bag", rosbag::bagmode::Read);
    }
    catch (rosbag::BagException& e)
    {
        ROS_ERROR("Failed to open bag file: %s", e.what());
        return 1;
    }

    // 创建一个包含所有消息的视图
    rosbag::View view(bag);

    // 播放视图中的消息
    for (const rosbag::MessageInstance& msg : view)
    {
        // 检查消息类型并处理相应的消息

        // 处理颜色相机图像消息
        if (msg.isType<sensor_msgs::Image>() && msg.getTopic() == "/camera/color/image_raw")
        {
            sensor_msgs::Image::ConstPtr color_image_msg = msg.instantiate<sensor_msgs::Image>();
            if (color_image_msg != nullptr)
            {
                colorImageCallback(color_image_msg);
            }
        }

        // 处理深度相机图像消息
        if (msg.isType<sensor_msgs::Image>() && msg.getTopic() == "/camera/depth/image_rect_raw")
        {
            sensor_msgs::Image::ConstPtr depth_image_msg = msg.instantiate<sensor_msgs::Image>();
            if (depth_image_msg != nullptr)
            {
                depthImageCallback(depth_image_msg);
            }
        }
    }

    // 关闭ROS包
    bag.close();

    // 关闭OpenCV图形界面
    cv::destroyAllWindows();

    return 0;
}
