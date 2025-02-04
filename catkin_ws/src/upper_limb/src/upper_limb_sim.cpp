#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <vector>

std::vector<float> arr_r(9, 0.0);  // 右侧电机的位置数据
std::vector<float> arr_l(9, 0.0);  // 左侧电机的位置数据

// 回调函数，用于处理来自左侧电机的消息
void motorStatusCallbackL(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    // 提取位置数据（数组前9个元素）
    for (int i = 0; i < 9; i++) {
        arr_l[i] = msg->data[i*3];  // 将前9个位置数据存储到 arr_l 中
    }
}

// 回调函数，用于处理来自右侧电机的消息
void motorStatusCallbackR(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    // 提取位置数据（数组前9个元素）
    for (int i = 0; i < 9; i++) {
        arr_r[i] = msg->data[i*3];  // 将前9个位置数据存储到 arr_r 中
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "upper_limb_sim");
    ros::NodeHandle nh;

    // 创建一个发布器，向 /motor_command_r 发布数据
    ros::Publisher motor_pub_r = nh.advertise<std_msgs::Float32MultiArray>("/motor_command_r", 1);
    ros::Publisher motor_pub_l = nh.advertise<std_msgs::Float32MultiArray>("/motor_command_l", 1);

    // 创建两个订阅器，分别订阅 /motor_status_r 和 /motor_status_l
    ros::Subscriber motor_sub_r = nh.subscribe("/motor_status_r", 1, motorStatusCallbackR);
    ros::Subscriber motor_sub_l = nh.subscribe("/motor_status_l", 1, motorStatusCallbackL);


    ros::Rate loop_rate(125);

    // 等待 2 秒钟，确保程序有时间订阅到数据
    ROS_INFO("Waiting for 2 seconds to allow for subscribing...");
    ros::Duration(2.0).sleep();  // 等待 2 秒钟    

    // 初始化数据
    std_msgs::Float32MultiArray msg_l;
    std_msgs::Float32MultiArray msg_r;
    msg_l.data.resize(9, 0.0); // 初始化数据为长度9的全0数组
    msg_r.data.resize(9, 0.0); 

    for(int i=0;i<9;i++){
        msg_l.data[i] = arr_l[i];
        msg_r.data[i] = arr_r[i];
    }

    // 递增步长和循环次数
    float increment = 250.0;
    int max_iterations = 2000;

    // 发布2000次数据
    for (int i = 0; i < max_iterations; i++) {
        // 更新数据
        for (int j = 0; j < 9; j++) {
            msg_l.data[j] += increment;
            msg_r.data[j] += increment;
        }

        // 发布数据
        motor_pub_r.publish(msg_r);
        motor_pub_l.publish(msg_l);

        ros::Duration(0.008).sleep(); 

        // 检查ROS是否正在运行
        ros::spinOnce();
    }

    ROS_INFO("Publishing complete, program exiting.");
    return 0;
}
