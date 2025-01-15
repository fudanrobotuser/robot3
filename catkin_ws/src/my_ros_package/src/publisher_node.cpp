#include <ros/ros.h>
#include <std_msgs/String.h>
#include <pthread.h>
#include <iostream>
#include <vector>
#include "rt_ethercat.hpp"
#include <string>
#include <thread>
#include <iomanip>
#include <sys/ipc.h>
#include <sys/shm.h>
#include "my_ros_package/MotorCmd.h"
#include <sensor_msgs/JointState.h>

using namespace ETHERCAT_SPACE;

Ecat_motor motors("enp7s0", 1000, 15);

// 定义控制指令和电机数据
vector<ImpedanceModeCmd> Icmomond;
vector<SpeedModeCmd> Scmomond;
vector<PositionModeCmd> Pcmomond;
vector<motor_data> Data;

pthread_t thread1;
//EtherCat线程读写实例
void Ethercat_syncThread(){
  while(1){
      motors.rt_ethercat_run();  //程序内部包含与从站线程周期同步，此函数处理时间约300Us左右
  }
}

int main(int argc, char **argv) {

    Icmomond.resize(motors._motor_num);
    Scmomond.resize(motors._motor_num);
    Pcmomond.resize(motors._motor_num);
    Data.resize(motors._motor_num);

    motors.Id_data(15, &Data[15-1]);
    motors.Id_data(14, &Data[14-1]);
    motors.Id_data(13, &Data[13-1]);

    // motors.Id_control_order(15,ENABLE_MODE);
    // motors.Id_control_order(14,ENABLE_MODE);
    // motors.Id_control_order(13,ENABLE_MODE);

    Pcmomond[15 - 1].angle = Data[15-1].angle;     // 目标位置
    Pcmomond[15 - 1].Vkp = 3;                      // 位置环比例增益
    Pcmomond[15 - 1].Vki = 700;                    // 位置环积分增益
    Pcmomond[15 - 1].KP = 1;                       // 刚度
    Pcmomond[15 - 1].KD = 0;                       // 阻尼
    // motors.Id_command(15, &Pcmomond[15 - 1]); // 发送控制指令    

    Pcmomond[14 - 1].angle = Data[14-1].angle;     // 目标位置
    Pcmomond[14 - 1].Vkp = 3;                      // 位置环比例增益
    Pcmomond[14 - 1].Vki = 700;                    // 位置环积分增益
    Pcmomond[14 - 1].KP = 1;                       // 刚度
    Pcmomond[14 - 1].KD = 0;                       // 阻尼
    // motors.Id_command(14, &Pcmomond[14 - 1]); // 发送控制指令  

    Pcmomond[13 - 1].angle = Data[13-1].angle;     // 目标位置
    Pcmomond[13 - 1].Vkp = 3;                      // 位置环比例增益
    Pcmomond[13 - 1].Vki = 700;                    // 位置环积分增益
    Pcmomond[13 - 1].KP = 1;                       // 刚度
    Pcmomond[13 - 1].KD = 0;                       // 阻尼
    // motors.Id_command(13, &Pcmomond[13 - 1]); // 发送控制指令  

    motors.Id_control_order(1,ENABLE_MODE);
    motors.Id_data(1, &Data[1-1]);
    Pcmomond[1 - 1].angle = Data[1-1].angle;     // 目标位置
    Pcmomond[1 - 1].Vkp = 3;                      // 位置环比例增益
    Pcmomond[1 - 1].Vki = 700;                    // 位置环积分增益
    Pcmomond[1 - 1].KP = 1;                       // 刚度
    Pcmomond[1 - 1].KD = 0;                       // 阻尼
    motors.Id_command(1, &Pcmomond[1 - 1]); // 发送控制指令  

    // 开启电机通信线程
    std::thread rcv_thread1 = std::thread(&Ethercat_syncThread);
    usleep(100000); // 延时 100ms    

    // 初始化 ROS 节点
    ros::init(argc, argv, "publisher_node");
    ros::NodeHandle nh;

    // 创建一个发布者，发布到 "chatter" 话题
    ros::Publisher pub = nh.advertise<std_msgs::String>("chatter", 10);
    ros::Publisher pub2 = nh.advertise<my_ros_package::MotorCmd>("pub2", 10);
    ros::Publisher pub3 = nh.advertise<sensor_msgs::JointState>("joint_states", 10);
    sensor_msgs::JointState joint_state;

    // 设置发布频率
    ros::Rate rate(1); // 1 Hz

    while (ros::ok()) {
        joint_state.name.clear();    // 清空关节名称数组
        joint_state.position.clear(); // 清空关节位置数组
        joint_state.velocity.clear(); // 清空关节速度数组
        joint_state.effort.clear();   // 清空关节力矩数组

        // 设置 header
        joint_state.header.stamp = ros::Time::now();
        joint_state.header.frame_id = "base_link";
        for (int i = 0; i < 12; i++) {
            motors.Id_data(i + 1, &Data[i]); // 读取电机数据到 Data_left
            joint_state.name.push_back("joint"+ std::to_string(i));
            joint_state.position.push_back(Data[i].angle);
            joint_state.velocity .push_back(Data[i].velocity);
            joint_state.effort.push_back(Data[i].torque);
        }

        pub3.publish(joint_state);

        std_msgs::String msg;
        msg.data = "Hello, ROS!";
        pub.publish(msg); // 发布消息
        ROS_INFO("Published: %s", msg.data.c_str());
        rate.sleep(); // 等待
    }

    return 0;
}
