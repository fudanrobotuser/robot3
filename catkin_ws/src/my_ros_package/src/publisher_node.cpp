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


using namespace ETHERCAT_SPACE;

Ecat_motor motors("enp3s0", 1000, 15);

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

    // 开启电机通信线程
    std::thread rcv_thread1 = std::thread(&Ethercat_syncThread);
    usleep(100000); // 延时 100ms    

    // 初始化 ROS 节点
    ros::init(argc, argv, "publisher_node");
    ros::NodeHandle nh;

    // 创建一个发布者，发布到 "chatter" 话题
    ros::Publisher pub = nh.advertise<std_msgs::String>("chatter", 10);

    // 设置发布频率
    ros::Rate rate(1); // 1 Hz

    while (ros::ok()) {
        std_msgs::String msg;
        msg.data = "Hello, ROS!";
        pub.publish(msg); // 发布消息
        ROS_INFO("Published: %s", msg.data.c_str());
        rate.sleep(); // 等待
    }

    return 0;
}
