// 每个 ROS C++ 程序都以这几行开头
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// ============ 回调函数 ============
// "回调"就是"每当我订阅的话题来了新消息，就自动调用这个函数"
void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    // msg->width 就是这帧点云里有多少个点
    ROS_INFO("收到一帧点云，共 %u 个点", msg->width);
}

int main(int argc, char** argv) {
    // 1. 初始化 ROS，给节点起个名字
    ros::init(argc, argv, "hello_node");

    // 2. 创建一个"句柄"，后面所有操作都通过它
    ros::NodeHandle nh;

    // 3. 订阅话题
    //    参数：话题名， 队列长度， 回调函数
    ros::Subscriber sub = nh.subscribe("/only_lidar_points_pub", 1, cloudCallback);

    ROS_INFO("hello_node 已启动，正在等待点云...");

    // 4. 进入循环，不断处理回调，直到按 Ctrl+C
    ros::spin();

    return 0;
}