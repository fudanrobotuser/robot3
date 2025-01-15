#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>  // 用于接收 /action 的整型消息
#include <pthread.h>
#include <iostream>
#include <vector>
#include "rt_ethercat.hpp"
#include <string>
#include <thread>
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
sensor_msgs::JointState joint_state;
pthread_t thread1;

ros::Publisher high_freq_pub; // 250Hz 的 Publisher
// 创建一个订阅者，订阅 /action 话题
ros::Subscriber action_sub;
ros::Timer high_freq_timer;

// EtherCat线程读写实例
void Ethercat_syncThread() {
  while (1) {
    motors.rt_ethercat_run();  // 程序内部包含与从站线程周期同步，此函数处理时间约300Us左右
  }
}

// 全局变量，用于存储 /action 的值
int action_value = 0;

// /action 话题的回调函数
void actionCallback(const std_msgs::Int32::ConstPtr& msg) {
  action_value = msg->data;  // 更新 action_value
  ROS_INFO("Received action value: %d", action_value);

  if (action_value == 1) {
    // 使能电机 1
    motors.Id_control_order(15, ENABLE_MODE);
    motors.Id_control_order(14, ENABLE_MODE);
    motors.Id_control_order(13, ENABLE_MODE);
    ROS_INFO("Motor 1 enabled.");
  } else if (action_value == 2) {
    // 设置电机 1 进入位置模式
    motors.Id_data(15, &Data[15 - 1]);
    Pcmomond[15 - 1].angle = Data[15 - 1].angle;  // 目标位置
    Pcmomond[15 - 1].Vkp = 3;                     // 位置环比例增益
    Pcmomond[15 - 1].Vki = 700;                   // 位置环积分增益
    Pcmomond[15 - 1].KP = 1;                      // 刚度
    Pcmomond[15 - 1].KD = 0;                      // 阻尼
    motors.Id_command(15, &Pcmomond[15 - 1]);     // 发送控制指令

    motors.Id_data(14, &Data[14 - 1]);
    Pcmomond[14 - 1].angle = Data[14 - 1].angle;  // 目标位置
    Pcmomond[14 - 1].Vkp = 3;                     // 位置环比例增益
    Pcmomond[14 - 1].Vki = 700;                   // 位置环积分增益
    Pcmomond[14 - 1].KP = 1;                      // 刚度
    Pcmomond[14 - 1].KD = 0;                      // 阻尼
    motors.Id_command(14, &Pcmomond[14 - 1]);     // 发送控制指令

    motors.Id_data(13, &Data[13 - 1]);
    Pcmomond[13 - 1].angle = Data[13 - 1].angle;  // 目标位置
    Pcmomond[13 - 1].Vkp = 3;                     // 位置环比例增益
    Pcmomond[13 - 1].Vki = 700;                   // 位置环积分增益
    Pcmomond[13 - 1].KP = 1;                      // 刚度
    Pcmomond[13 - 1].KD = 0;                      // 阻尼
    motors.Id_command(13, &Pcmomond[13 - 1]);     // 发送控制指令
    ROS_INFO("Motor 1 set to position mode.");
  }
}

void highFreqCallback(const ros::TimerEvent& event) {
  joint_state.name.clear();
  joint_state.position.clear();
  joint_state.velocity.clear();
  joint_state.effort.clear();

  // 设置 header
  joint_state.header.stamp = ros::Time::now();
  joint_state.header.frame_id = "base_link";

  //读取电机数据并发布 joint_state
  for (int i = 0; i < 12; i++) {
      motors.Id_data(i + 1, &Data[i]);  // 读取电机数据到 Data
      joint_state.name.push_back("joint" + std::to_string(i));
      joint_state.position.push_back(Data[i].angle);
      joint_state.velocity.push_back(Data[i].velocity);
      joint_state.effort.push_back(Data[i].torque);
  }

  high_freq_pub.publish(joint_state);
}


int main(int argc, char** argv) {
  // 初始化电机控制指令和电机数据
  Icmomond.resize(motors._motor_num);
  Scmomond.resize(motors._motor_num);
  Pcmomond.resize(motors._motor_num);
  Data.resize(motors._motor_num);

  // 读取电机数据
  // motors.Id_data(15, &Data[15 - 1]);
  // motors.Id_data(14, &Data[14 - 1]);
  // motors.Id_data(13, &Data[13 - 1]);
  motors.print_Ecat_id_info();
  // 开启电机通信线程
  std::thread rcv_thread1 = std::thread(&Ethercat_syncThread);
  usleep(100000);  // 延时 100ms
  // 初始化 ROS 节点
  ros::init(argc, argv, "publisher_node");
  ros::NodeHandle nh;
  high_freq_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 10);
  // 创建一个订阅者，订阅 /action 话题
  action_sub = nh.subscribe("/action_motor1", 10, actionCallback);
  high_freq_timer = nh.createTimer(ros::Duration(1.0 / 250.0), highFreqCallback);

  ros::spin();
  // 等待线程结束
  rcv_thread1.join();

  return 0;
}