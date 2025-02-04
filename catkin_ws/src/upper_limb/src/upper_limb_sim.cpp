#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <vector>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "upper_limb_sim");
    ros::NodeHandle nh;

    // 创建一个发布器，向 /motor_command_r 发布数据
    ros::Publisher motor_pub_r = nh.advertise<std_msgs::Float32MultiArray>("/motor_command_r", 1);
    ros::Publisher motor_pub_l = nh.advertise<std_msgs::Float32MultiArray>("/motor_command_l", 1);

    // 设置发布频率 250Hz
    ros::Rate loop_rate(250);

    // 初始化数据
    std_msgs::Float32MultiArray msg;
    msg.data.resize(9, 0.0); // 初始化数据为长度9的全0数组

    // 递增步长和循环次数
    float increment = 250.0;
    int max_iterations = 2000;

    // 发布2000次数据
    for (int i = 0; i < max_iterations; i++) {
        // 更新数据
        for (int j = 0; j < 9; j++) {
            msg.data[j] += increment;
        }

        // 发布数据
        motor_pub_r.publish(msg);
        motor_pub_l.publish(msg);

        // 打印输出当前数据
        ROS_INFO("Publishing data: %f, %f, %f, %f, %f, %f, %f, %f, %f",
                 msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4],
                 msg.data[5], msg.data[6], msg.data[7], msg.data[8]);

        // 每4ms递增一次
        ros::Duration(0.004).sleep(); // 4ms 延迟

        // 检查ROS是否正在运行
        ros::spinOnce();
    }

    ROS_INFO("Publishing complete, program exiting.");
    return 0;
}
