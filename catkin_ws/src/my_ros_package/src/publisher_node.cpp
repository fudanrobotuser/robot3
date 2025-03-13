#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
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

int idxChanged[15] = {6, 5, 4, 3, 2, 1, 12, 11, 10, 9, 8, 7, 13, 14, 15};

bool isAllMotorEnabled = false;
using namespace ETHERCAT_SPACE;
std::atomic<bool> stop_thread(false);  // 退出标志

Ecat_motor motors("enp7s0", 1000, 15);

// 定义控制指令和电机数据
vector<ImpedanceModeCmd> Icmomond;
vector<SpeedModeCmd> Scmomond;
vector<PositionModeCmd> Pcmomond;
vector<motor_data> Data;
sensor_msgs::JointState joint_state;
pthread_t thread1;

ros::Publisher motor_feedback_pub;  // 250Hz 的 Publisher
// 创建一个订阅者，订阅 /action 话题
ros::Subscriber motor_action_sub;
ros::Timer motor_feedback_timer;
ros::Subscriber motor_command_sub;

// EtherCat线程读写实例
void Ethercat_syncThread() {
  while (!stop_thread) {       // 检查退出标志
    motors.rt_ethercat_run();  // 程序内部包含与从站线程周期同步，此函数处理时间约300Us左右
  }
}

// 全局变量，用于存储 /action 的值
int action_value = 0;

int action_count = 0;

// /action 话题的回调函数
void motor_action_callback(const std_msgs::Int32::ConstPtr& msg) {


  action_value = msg->data;  // 更新 action_value
  ROS_INFO("Received action value: %d", action_value);

  if (action_value == 1) {
    // 使能电机 1
    for (int i = 1; i <= 15; i++) {
      motors.Id_control_order(i, ENABLE_MODE);
    }
    // motors.Id_control_order(15, ENABLE_MODE);
    // motors.Id_control_order(14, ENABLE_MODE);
    // motors.Id_control_order(13, ENABLE_MODE);

  } else if (action_value == 2) {
    // 设置腰部电机  进入位置模式
    motors.Id_data(15, &Data[15 - 1]);
    Pcmomond[15 - 1].angle = Data[15 - 1].angle;  // 目标位置
    Pcmomond[15 - 1].Vkp = 5;                     // 位置环比例增益
    Pcmomond[15 - 1].Vki = 500;                   // 位置环积分增益
    Pcmomond[15 - 1].KP = 10;                      // 刚度
    Pcmomond[15 - 1].KD = 1.0;                      // 阻尼
    motors.Id_command(15, &Pcmomond[15 - 1]);     // 发送控制指令

    motors.Id_data(14, &Data[14 - 1]);
    Pcmomond[14 - 1].angle = Data[14 - 1].angle;  // 目标位置
    Pcmomond[14 - 1].Vkp = 5;                     // 位置环比例增益
    Pcmomond[14 - 1].Vki = 500;                   // 位置环积分增益
    Pcmomond[14 - 1].KP = 10;                      // 刚度
    Pcmomond[14 - 1].KD = 1.0;                      // 阻尼
    motors.Id_command(14, &Pcmomond[14 - 1]);     // 发送控制指令

    motors.Id_data(13, &Data[13 - 1]);
    Pcmomond[13 - 1].angle = Data[13 - 1].angle;  // 目标位置
    Pcmomond[13 - 1].Vkp = 5;                     // 位置环比例增益
    Pcmomond[13 - 1].Vki = 500;                   // 位置环积分增益
    Pcmomond[13 - 1].KP = 10;                      // 刚度
    Pcmomond[13 - 1].KD = 1.0;                      // 阻尼
    motors.Id_command(13, &Pcmomond[13 - 1]);     // 发送控制指令
    ROS_INFO("3 Motor  set to position mode.");
  } else if (action_value == 3) {
    isAllMotorEnabled = true;
  } else if (action_value == 4) {
    // 使能电机 1
    for (int i = 1; i <= 15; i++) {
      motors.Id_control_order(i, DISABLE_MODE);
    }
    // motors.Id_control_order(15, ENABLE_MODE);
    // motors.Id_control_order(14, ENABLE_MODE);
    // motors.Id_control_order(13, ENABLE_MODE);

  }
  // 标零位
  else if (action_value == 101) {
    motors.Id_control_order(action_value - 100, ZERO_MODE);
  } else if (action_value == 102) {
    motors.Id_control_order(action_value - 100, ZERO_MODE);
  } else if (action_value == 103) {
    motors.Id_control_order(action_value - 100, ZERO_MODE);
  } else if (action_value == 104) {
    motors.Id_control_order(action_value - 100, ZERO_MODE);
  } else if (action_value == 105) {
    motors.Id_control_order(action_value - 100, ZERO_MODE);
  } else if (action_value == 106) {
    motors.Id_control_order(action_value - 100, ZERO_MODE);
  } else if (action_value == 107) {
    motors.Id_control_order(action_value - 100, ZERO_MODE);
  } else if (action_value == 108) {
    motors.Id_control_order(action_value - 100, ZERO_MODE);
  } else if (action_value == 109) {
    motors.Id_control_order(action_value - 100, ZERO_MODE);
  } else if (action_value == 110) {
    motors.Id_control_order(action_value - 100, ZERO_MODE);
  } else if (action_value == 111) {
    motors.Id_control_order(action_value - 100, ZERO_MODE);
  } else if (action_value == 112) {
    motors.Id_control_order(action_value - 100, ZERO_MODE);
  } else if (action_value == 113) {
    motors.Id_control_order(action_value - 100, ZERO_MODE);
  } else if (action_value == 114) {
    motors.Id_control_order(action_value - 100, ZERO_MODE);
  } else if (action_value == 115) {
    motors.Id_control_order(action_value - 100, ZERO_MODE);
  } else if (action_value == 200) {
    for (int i = 0; i < 15; i++) {
      motors.Id_data(i + 1, &Data[i]);  // 读取电机数据到 Data
      ROS_INFO("angle %d, %f", i, Data[i].angle);
    }
  }
}

void motor_command_callback(const std_msgs::Float32MultiArray::ConstPtr& cmd_msg) {

  action_count++;

  // 如果 action_count 可以整除 500，输出当前时间和 action_count
  if (action_count % 500 == 0) {
      time_t now = time(NULL);
      char buffer[80];
      strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", localtime(&now));
      printf("Time: %s, action_count: %d\n", buffer, action_count);
  }

  // 如果 action_count 超过 500 * 60 * 60，重置为 1
  if (action_count > 500 * 60 * 60) {
      action_count = 1;
  }


  // for (int i = 0; i < 12; i++) {
  // cmd_msg.data[i * 5] = reference.motor_ref[i].target_postion;
  // cmd_msg.data[i * 5 + 1] = reference.motor_ref[i].target_speed;
  // cmd_msg.data[i * 5 + 2] = reference.motor_ref[i].target_torque;
  // cmd_msg.data[i * 5 + 3] = reference.motor_ref[i].kp;
  // cmd_msg.data[i * 5 + 4] = reference.motor_ref[i].kd;
  // ROS_INFO("%f", cmd_msg->data[i * 5]);
  // ROS_INFO("%f", cmd_msg->data[i * 5 + 1]);
  // ROS_INFO("%f", cmd_msg->data[i * 5 + 2]);
  // ROS_INFO("%f", cmd_msg->data[i * 5 + 3]);
  // ROS_INFO("%f", cmd_msg->data[i * 5 + 4]);
  // }
  // 阻抗模式
  if (isAllMotorEnabled) {
    if (2 > 1) {
      int i = 6;
      int ii = idxChanged[i - 1];

      Icmomond[ii - 1].angle = cmd_msg->data[(i - 1) * 6];
      Icmomond[ii - 1].velocity = cmd_msg->data[(i - 1) * 6 + 1];
      Icmomond[ii - 1].torque = cmd_msg->data[(i - 1) * 6 + 2];
      Icmomond[ii - 1].KP = cmd_msg->data[(i - 1) * 6 + 3];
      Icmomond[ii - 1].KD = cmd_msg->data[(i - 1) * 6 + 4];
      motors.Id_command(ii, &Icmomond[ii - 1]);
      //cout << ii-1 << "   " << i-1 <<"    " <<  Icmomond[ii - 1].angle << endl;
    }
    if (2 > 1) {
      int i = 5;
      int ii = idxChanged[i - 1];

      Icmomond[ii - 1].angle = cmd_msg->data[(i - 1) * 6];
      Icmomond[ii - 1].velocity = cmd_msg->data[(i - 1) * 6 + 1];
      Icmomond[ii - 1].torque = cmd_msg->data[(i - 1) * 6 + 2];
      Icmomond[ii - 1].KP = cmd_msg->data[(i - 1) * 6 + 3];
      Icmomond[ii - 1].KD = cmd_msg->data[(i - 1) * 6 + 4];
      motors.Id_command(ii, &Icmomond[ii - 1]);
    }
    if (2 > 1) {
      int i = 4;
      int ii = idxChanged[i - 1];

      Icmomond[ii - 1].angle = cmd_msg->data[(i - 1) * 6];
      Icmomond[ii - 1].velocity = cmd_msg->data[(i - 1) * 6 + 1];
      Icmomond[ii - 1].torque = cmd_msg->data[(i - 1) * 6 + 2];
      Icmomond[ii - 1].KP = cmd_msg->data[(i - 1) * 6 + 3];
      Icmomond[ii - 1].KD = cmd_msg->data[(i - 1) * 6 + 4];
      motors.Id_command(ii, &Icmomond[ii - 1]);
    }
    if (2 > 1) {
      int i = 3;
      int ii = idxChanged[i - 1];

      Icmomond[ii - 1].angle = cmd_msg->data[(i - 1) * 6];
      Icmomond[ii - 1].velocity = cmd_msg->data[(i - 1) * 6 + 1];
      Icmomond[ii - 1].torque = cmd_msg->data[(i - 1) * 6 + 2];
      Icmomond[ii - 1].KP = cmd_msg->data[(i - 1) * 6 + 3];
      Icmomond[ii - 1].KD = cmd_msg->data[(i - 1) * 6 + 4];
      motors.Id_command(ii, &Icmomond[ii - 1]);
    }
    if (2 > 1) {
      int i = 2;
      int ii = idxChanged[i - 1];

      Icmomond[ii - 1].angle = cmd_msg->data[(i - 1) * 6];
      Icmomond[ii - 1].velocity = cmd_msg->data[(i - 1) * 6 + 1];
      Icmomond[ii - 1].torque = cmd_msg->data[(i - 1) * 6 + 2];
      Icmomond[ii - 1].KP = cmd_msg->data[(i - 1) * 6 + 3];
      Icmomond[ii - 1].KD = cmd_msg->data[(i - 1) * 6 + 4];
      motors.Id_command(ii, &Icmomond[ii - 1]);
    }
    if (2 > 1) {
      int i = 1;
      int ii = idxChanged[i - 1];

      Icmomond[ii - 1].angle = cmd_msg->data[(i - 1) * 6];
      Icmomond[ii - 1].velocity = cmd_msg->data[(i - 1) * 6 + 1];
      Icmomond[ii - 1].torque = cmd_msg->data[(i - 1) * 6 + 2];
      Icmomond[ii - 1].KP = cmd_msg->data[(i - 1) * 6 + 3];
      Icmomond[ii - 1].KD = cmd_msg->data[(i - 1) * 6 + 4];
      motors.Id_command(ii, &Icmomond[ii - 1]);
    }
    if (2 > 1) {
      int i = 12;
      int ii = idxChanged[i - 1];

      Icmomond[ii - 1].angle = cmd_msg->data[(i - 1) * 6];
      Icmomond[ii - 1].velocity = cmd_msg->data[(i - 1) * 6 + 1];
      Icmomond[ii - 1].torque = cmd_msg->data[(i - 1) * 6 + 2];
      Icmomond[ii - 1].KP = cmd_msg->data[(i - 1) * 6 + 3];
      Icmomond[ii - 1].KD = cmd_msg->data[(i - 1) * 6 + 4];
      motors.Id_command(ii, &Icmomond[ii - 1]);
      //cout << ii-1 << "   " << i-1 <<"    " <<  Icmomond[ii - 1].angle << endl;
    }
    if (2 > 1) {
      int i = 11;
      int ii = idxChanged[i - 1];

      Icmomond[ii - 1].angle = cmd_msg->data[(i - 1) * 6];
      Icmomond[ii - 1].velocity = cmd_msg->data[(i - 1) * 6 + 1];
      Icmomond[ii - 1].torque = cmd_msg->data[(i - 1) * 6 + 2];
      Icmomond[ii - 1].KP = cmd_msg->data[(i - 1) * 6 + 3];
      Icmomond[ii - 1].KD = cmd_msg->data[(i - 1) * 6 + 4];
      motors.Id_command(ii, &Icmomond[ii - 1]);
    }
    if (2 > 1) {
      int i = 10;
      int ii = idxChanged[i - 1];

      Icmomond[ii - 1].angle = cmd_msg->data[(i - 1) * 6];
      Icmomond[ii - 1].velocity = cmd_msg->data[(i - 1) * 6 + 1];
      Icmomond[ii - 1].torque = cmd_msg->data[(i - 1) * 6 + 2];
      Icmomond[ii - 1].KP = cmd_msg->data[(i - 1) * 6 + 3];
      Icmomond[ii - 1].KD = cmd_msg->data[(i - 1) * 6 + 4];
      motors.Id_command(ii, &Icmomond[ii - 1]);
    }
    if (2 > 1) {
      int i = 9;
      int ii = idxChanged[i - 1];

      Icmomond[ii - 1].angle = cmd_msg->data[(i - 1) * 6];
      Icmomond[ii - 1].velocity = cmd_msg->data[(i - 1) * 6 + 1];
      Icmomond[ii - 1].torque = cmd_msg->data[(i - 1) * 6 + 2];
      Icmomond[ii - 1].KP = cmd_msg->data[(i - 1) * 6 + 3];
      Icmomond[ii - 1].KD = cmd_msg->data[(i - 1) * 6 + 4];
      motors.Id_command(ii, &Icmomond[ii - 1]);
    }
    if (2 > 1) {
      int i = 8;
      int ii = idxChanged[i - 1];

      Icmomond[ii - 1].angle = cmd_msg->data[(i - 1) * 6];
      Icmomond[ii - 1].velocity = cmd_msg->data[(i - 1) * 6 + 1];
      Icmomond[ii - 1].torque = cmd_msg->data[(i - 1) * 6 + 2];
      Icmomond[ii - 1].KP = cmd_msg->data[(i - 1) * 6 + 3];
      Icmomond[ii - 1].KD = cmd_msg->data[(i - 1) * 6 + 4];
      motors.Id_command(ii, &Icmomond[ii - 1]);
    }
    if (2 > 1) {
      int i = 7;
      int ii = idxChanged[i - 1];

      Icmomond[ii - 1].angle = cmd_msg->data[(i - 1) * 6];
      Icmomond[ii - 1].velocity = cmd_msg->data[(i - 1) * 6 + 1];
      Icmomond[ii - 1].torque = cmd_msg->data[(i - 1) * 6 + 2];
      Icmomond[ii - 1].KP = cmd_msg->data[(i - 1) * 6 + 3];
      Icmomond[ii - 1].KD = cmd_msg->data[(i - 1) * 6 + 4];
      motors.Id_command(ii, &Icmomond[ii - 1]);
    }
  }
}

void motor_feedback_timer_callback(const ros::TimerEvent& event) {
  std_msgs::Float32MultiArray array_msg;
  array_msg.data.resize(36);

  for (int i = 0; i < 12; i++) {
    int ii = idxChanged[i] - 1;
    motors.Id_data(ii + 1, &Data[ii]);  // 读取电机数据到 Data
    array_msg.data[i * 3] = Data[ii].angle;
    array_msg.data[i * 3 + 1] = Data[ii].velocity;
    array_msg.data[i * 3 + 2] = Data[ii].torque;
  }
  motor_feedback_pub.publish(array_msg);
}

int main(int argc, char** argv) {
  // 初始化电机控制指令和电机数据
  Icmomond.resize(motors._motor_num);
  Scmomond.resize(motors._motor_num);
  Pcmomond.resize(motors._motor_num);
  Data.resize(motors._motor_num);

  motors.print_Ecat_id_info();
  // 开启电机通信线程
  std::thread rcv_thread1 = std::thread(&Ethercat_syncThread);
  usleep(100000);  // 延时 100ms
  // 初始化 ROS 节点
  ros::init(argc, argv, "publisher_node");
  ros::NodeHandle nh;
  motor_feedback_pub = nh.advertise<std_msgs::Float32MultiArray>("motor_feedback", 1);
  // 创建一个订阅者，订阅 /action 话题
  motor_action_sub = nh.subscribe("/motor_action", 1, motor_action_callback);
  motor_command_sub = nh.subscribe("/motor_command", 1, motor_command_callback);
  motor_feedback_timer = nh.createTimer(ros::Duration(1.0 / 500.0), motor_feedback_timer_callback);

  ros::spin();
  ros::shutdown();

  // 设置退出标志并等待线程结束
  stop_thread = true;
  // 等待线程结束
  rcv_thread1.join();

  return 0;
}