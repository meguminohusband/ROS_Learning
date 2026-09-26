#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    ROS_INFO("收到一帧点云，共 %u 个点", msg->width);
}
int main(int argc, char** argv) {
    ros::init(argc, argv, "hello_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/only_lidar_points_pub", 1, cloudCallback);
    ROS_INFO("hello_node 已启动");
    ros::spin();
    return 0;
}
