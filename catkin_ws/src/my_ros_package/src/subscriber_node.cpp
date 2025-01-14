#include <ros/ros.h>
#include <std_msgs/String.h>

// 回调函数，处理接收到的消息
void chatterCallback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("I heard: %s", msg->data.c_str());
}

int main(int argc, char **argv) {
    // 初始化 ROS 节点
    ros::init(argc, argv, "subscriber_node");
    ros::NodeHandle nh;

    // 创建一个订阅者，订阅 "chatter" 话题
    ros::Subscriber sub = nh.subscribe("chatter", 10, chatterCallback);

    // 进入事件循环
    ros::spin();

    return 0;
}
