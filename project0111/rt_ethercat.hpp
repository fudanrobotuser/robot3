#ifndef PROJECT_RT_ETHERCAT_H
#define PROJECT_RT_ETHERCAT_H
#include <unistd.h>
#include <pthread.h>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <vector>
#include <mutex>
#include <thread>
// #include "Timer.h"
#include <map>
using namespace std;
namespace ETHERCAT_SPACE{
  #define CUST_BYTE_NUM_OUT	28
  #define CUST_BYTE_NUM_IN	20
  #define TOT_BYTE_NUM_ROUND_OUT	28
  #define TOT_BYTE_NUM_ROUND_IN	20
  //电机指令类型
  #define DISABLE_MODE            0x01      //失能
  #define ENABLE_MODE             0x03      //使能
  #define ZERO_MODE               0x06      //角度置零
  //下面指令为出厂时配置，调用需联系我司技术人员
  #define CALIBRATION_MODE        0x02      //编码器矫正
  #define ABS_CALIBRATION_MODE    0x07      //绝对位置矫正，电机将自转一周，确保电机无机械限位再使用
  #define ZERO_MODE_DELAY         0x09      //角度置零
  #define PARA_INDENTIFY          0x0D      //参数辨识

    //电机参数类型
  #define MOTOR_OR_Error                0x01      //错误信息
  /*电机可观测信号*/
  #define MOTOR_OR_Ia                   0x02      //A相电流
  #define MOTOR_OR_Ib                   0x03      //B相电流
  #define MOTOR_OR_Ic                   0x04      //C相电流
  #define MOTOR_OR_Id                   0x05      //D轴电流
  #define MOTOR_OR_Iq                   0x06      //Q轴电流
  #define MOTOR_OR_Vbus                 0x07      //总线电压
  #define MOTOR_OR_Vd                   0x08      //D轴电压
  #define MOTOR_OR_Vq                   0x09      //Q轴电压
  #define MOTOR_OR_Te                   0x0A      //电磁转矩
  #define MOTOR_OR_Angel                0x0B      //电角度
  #define MOTOR_OR_We                   0x0C      //电角速度
  #define MOTOR_OR_temperature          0x0D      //电机温度
  #define DRIVE_OR_temperature          0x0E      //驱动温度
  #define MOTOR_OR_angle                0x0F      //角度
  #define MOTOR_OR_velocity             0x10      //角速度
  #define MOTOR_OR_torque               0x11      //扭矩

  #define MOTOR_OW_Save_Patemeter       0x2A        //将参数存入FLASH
  #define MOTOR_OW_Clear_Fault          0x2B        //清除错误

  /*电机电气参数*/
  #define MOTOR_WR_LD                   0x40        //电机d轴电感
  #define MOTOR_WR_LQ                   0x41        //电机q轴电感
  #define MOTOR_WR_FLUX                 0x42        //电机转子磁链
  #define MOTOR_WR_RESISTANCE           0x43        //电机相电阻
  /*电机机械参数*/
  #define MOTOR_WR_GR                   0x44        //电机减速比 1.0-32.0
  #define MOTOR_WR_J                    0x45        //电机转动惯量
  #define MOTOR_WR_B                    0x46        //粘滞系数
  #define MOTOR_WR_P                    0x47        //电机极对数
  #define MOTOR_WR_Tf                   0x48        //静摩擦力矩
  #define MOTOR_WR_KT_OUT               0x49        //电机电磁转矩系数 0.1 - 10.0 A/NM
  /*可读写其他参数*/
  #define MOTOR_WR_Major                0x50      //电机型号
  #define MOTOR_WR_CAN_ID               0x51      //设置CANID，其中CANID范围1-15
  #define MOTOR_WR_Current_Risetime     0x52      //电机电流环响应时间 单位us 最小值为：0.345*MOTOR_LD/MOTOR_RESISTANCE * 1e6
  #define MOTOR_WR_Max_Angle            0x53      //限位最大角度 单位弧度
  #define MOTOR_WR_Min_Angle            0x54      //限位最小角度 单位弧度
  #define MOTOR_WR_Angle_Limit_Switch   0x55      //角度限位开关 0或1
  #define MOTOR_WR_Current_Limit        0x56      //最大电流限制 0-60A
  #define MOTOR_WR_CAN_Timeout          0x57      //CAN通信中,Timeout个周期未收到CAN信号，电机进入失能模式
  #define MOTOR_WR_ECAT_ID              0x58      //设置ECAT_ID，其中ECAT_ID范围0-255
  #define MOTOR_WR_temp_Protection      0x59      //电机温度保护阈值,为0时表示关闭温度保护
  #define DRIVER_WR_temp_Protection     0x5A      //驱动温度保护阈值,为0时表示关闭温度保护
  #define MOTOR_WR_CONTROL_MODE         0x5B      //控制模式，0阻抗控制 1速度控制 2位置控制
  #define MOTOR_WR_Velfilter_constant   0x5C      //角速度低通滤波常数
  
  //电机控制模式
  #define MOTOR_IMPEDANCE_MODE    0
  #define MOTOR_SPEED_MODE        1
  #define MOTOR_POSITION_MODE     2

  #define DEFAULT_MOTOR_NUM 1
  #define ECAT_PI              3.1415926535898
  #define NSEC_PER_SEC 1000000000
  #define EC_TIMEOUTMON 500
  #define stack64k (64 * 1024)

  typedef union												//---- output buffer ----
  {
    uint8_t  Byte [TOT_BYTE_NUM_ROUND_OUT];
    struct
    {
      float       cmd1;   
      float       cmd2;     
      float       cmd3;     
      float       cmd4;             
      float       cmd5;             
      float       data_set_value; 
      uint16_t    control;
      uint8_t     data_type;
      uint8_t     watchdog;
    }value;
  } PROCBUFFER_OUT;

  typedef union												//---- input buffer ----
  {
    uint8_t  Byte [TOT_BYTE_NUM_ROUND_IN];
    struct
    {
      float       angle;
      float       velocity;
      float       torque;
      float       data;
      uint16_t    state;
      uint16_t    ecat_id;
    }value;
  } PROCBUFFER_IN;
  //电机控制命令结构体 阻抗控制，rad为弧度
  struct ImpedanceModeCmd
  {
    float       angle;    //期望角度    单位rad
    float       velocity; //期望角速度  单位rad/s   
    float       torque;   //期望扭矩    单位Nm   
    float       KP;       //刚度系数       
    float       KD;       //阻尼系数 
    ImpedanceModeCmd(){
      angle    = 0.0f;
      velocity = 0.0f;
      torque   = 0.0f;
      KP       = 0.0f;
      KD       = 0.0f;
    }           
  };

  struct Bootloard_data{
      uint8_t intput_byte [TOT_BYTE_NUM_ROUND_IN-4];
      uint8_t output_byte [TOT_BYTE_NUM_ROUND_OUT-4];
  };

  //电机控制命令结构体 速度控制，rad为弧度
  struct SpeedModeCmd
  {
    float       velocity;   //期望角速度  单位rad/s   
    float       Vkp;        //速度环Kp增益    
    float       Vki;        //速度环Ki增益    
    float       Temp1;      //空      
    float       Temp2;      //空
    SpeedModeCmd(){
      velocity = 0.0f;
      Vkp = 0.0f;
      Vki = 0.0f;
      Temp1 = 0.0f;
      Temp2 = 0.0f;
    }           
  };
  //电机控制命令结构体 位置控制，rad为弧度
  struct PositionModeCmd
  {
    float       angle;    //期望角度    单位rad
    float       Vkp;      //速度环Kp增益  
    float       Vki;      //速度环Ki增益   
    float       KP;       //位置环Kp增益        
    float       KD;       //位置环Kd增益  
    PositionModeCmd(){
      angle = 0.0f;
      Vkp = 0.0f;
      KD = 0.0f;
      KP = 0.0f;
      KD = 0.0f;
    }           
  };
  struct motor_data
  {
    float       angle;   //电机角度  单位rad
    float       velocity;//电机角速度  单位rad/s   
    float       torque;  //电机扭矩  单位Nm  
    uint16_t    ecat_id; //电机id编号,保存在电机内部。与Ethercat的CNT无关
    uint16_t    state;
    motor_data(){
      angle = 0.0f;
      velocity = 0.0f;
      torque = 0.0f;
      ecat_id = 0;
      // error = 0;
      state = 0;
    }
  };
  /*!
  * Timer for measuring time elapsed with clock_monotonic
  */
  class Timer {
  public:

    /*!
    * Construct and start timer
    */
    explicit Timer() { start(); }

    /*!
    * Start the timer
    */
    void start() { clock_gettime(CLOCK_MONOTONIC, &_startTime); }

    /*!
    * Get milliseconds elapsed
    */
    double getMs() { return (double)getNs() / 1.e6; }

    /*!
    * Get nanoseconds elapsed
    */
    int64_t getNs() {
      struct timespec now;
      clock_gettime(CLOCK_MONOTONIC, &now);
      return (int64_t)(now.tv_nsec - _startTime.tv_nsec) +
            1000000000 * (now.tv_sec - _startTime.tv_sec);
    }

    /*!
    * Get seconds elapsed
    */
    double getSeconds() { return (double)getNs() / 1.e9; }

    struct timespec _startTime;
  };

  /** ETHERCAT 电机驱动共用初始化类
   */
  class Ecat_motor {
  public:
      /** 内部变量赋值默认参数
       *  @param port      网卡名
       *  @param period    线程运行周期 单位us
       *  @param motor_num 电机数量
       *  @param run_self （可选）是否自启动线程：为true时自启动实时线程运行rt_ethercat_run()，用户无需再自定义线程运行rt_ethercat_run()函数
       *  @note
       *  设定周期最小500us
       */
      Ecat_motor(std::string port,float period,int16_t motor_num);
      Ecat_motor(std::string port,float period,int16_t motor_num,bool run_self);
      Ecat_motor(std::string port1,std::string port2,float period,int16_t motor_num);
      ~Ecat_motor();
      
      //注：控制电机有两种形式：
      //1.通过cnt控制，电机编号为线缆连接顺序——与主控连接的第一个电机cnt=1，连接1号电机后的cnt=2,一次类推 (例如：control_order(int32_t cnt,uint8_t type);)
      //2.通过Ecat_id控制，Ecat_id通过上位机提前设置保存在电机内部，设置id号1-255，确保所有电机id号均不相同，
      //  在线程中运行rt_ethercat_run（）函数100ms后,系统会自动获取完电机Ecat_id,此时可通过此方式进行控制（(例如：Id_control_order(int32_t id,uint8_t type);)）

      /** 设置电机指令如：电机使能，电机失能，角度调零，角度矫正等
       *  @param cnt       从站编号,最小值为1
       *  @param type      指令类型
       */   
      void control_order(int32_t cnt,uint8_t type);

       /** 设置电机指令如：电机使能，电机失能，角度调零，角度矫正等
       *  @param id        电机flash中存储的ECat_id
       *  @param type      指令类型
       */        
      bool Id_control_order(int32_t id,uint8_t type);

      /** 读写电机参数 
       *  @param cnt           从站编号,最小值为1
       *  @param type          参数类型
       *  @param value         读写变量指针
       *  @param write         写入时为ture,读取时为false
       *  @return 设定成功返回1 设定失败返回0
       *  @note
       *  处理时间较慢切勿放在主线程中
       *  写入是将指针value中存放的值写入对于类型，读取是通过指针value返回读取值
       */  
      bool para_exchange(int32_t cnt,uint8_t type ,float* const value,bool write);

      /** 读写电机参数 
       *  @param id            电机flash中存储的ECat_id
       *  @param type          参数类型
       *  @param value         读写变量指针
       *  @param write         写入时为ture,读取时为false
       *  @return 设定成功返回1 设定失败返回0
       *  @note
       *  处理时间较慢切勿放在主线程中
       *  写入是将指针value中存放的值写入对于类型，读取是通过指针value返回读取值
       */  
      bool Id_para_exchange(int32_t id,uint8_t type ,float* const value,bool write);

      /** 阻抗控制模式 
       *  @param cnt          从站编号,最小值为1
       *  @param cmd          阻抗控制命令的结构体指针
       *  @note 不同的结构体指针对于不同的控制模式
       */     
      void command(int32_t cnt,ImpedanceModeCmd *cmd);

      /** 速度控制模式
       *  @param cnt          从站编号,最小值为1
       *  @param cmd          速度控制命令的结构体指针
       *  @note 不同的结构体指针对于不同的控制模式
       */   
      void command(int32_t cnt,SpeedModeCmd *cmd);

      /** 位置控制模式
       *  @param cnt          从站编号,最小值为1
       *  @param cmd          位置控制命令的结构体指针
       *  @note 不同的结构体指针对于不同的控制模式
       */         
      void command(int32_t cnt,PositionModeCmd *cmd);

      /** 阻抗控制模式 
       *  @param id           电机flash中存储的ECat_id
       *  @param cmd          阻抗控制命令的结构体指针
       *  @note 不同的结构体指针对于不同的控制模式
       */   
      bool Id_command(int32_t id,ImpedanceModeCmd *cmd);

      /** 速度控制模式
       *  @param id           电机flash中存储的ECat_id
       *  @param cmd          速度控制命令的结构体指针
       *  @note 不同的结构体指针对于不同的控制模式
       */ 
      bool Id_command(int32_t id,SpeedModeCmd *cmd);

      /** 位置控制模式
       *  @param id           电机flash中存储的ECat_id
       *  @param cmd          位置控制命令的结构体指针
       *  @note 不同的结构体指针对于不同的控制模式
       */  
      bool Id_command(int32_t id,PositionModeCmd *cmd);

      /** 电机返回参数
       *  @param cnt          从站编号,最小值为1
       *  @param ptr          读取数据类型的结构体指针
       *  @note 不同的结构体指针对于不同的控制模式
       */  
      void data(int32_t cnt,motor_data *ptr);

      /** 电机返回参数
       *  @param id           电机flash中存储的ECat_id
       *  @param ptr          读取数据类型的结构体指针
       *  @note 不同的结构体指针对于不同的控制模式
       */  
      bool Id_data(int32_t id,motor_data *ptr);

      /** EtherCat数据收发
       *  @note 建立线程后的程序中延时100ms，确保Ethercat已经获取到全部电机的ECat_id
       */      
      void rt_ethercat_run();   

       /** 打印Ecat_id与cnt的对应关系
       */       
      void print_Ecat_id_info();

      bool bootloader(int32_t cnt,Bootloard_data * data);

      bool Id_bootloader(int32_t cnt,Bootloard_data * data);

      int16_t _motor_num = DEFAULT_MOTOR_NUM;         //电机数量
      int16_t _slave_num = DEFAULT_MOTOR_NUM;
      float _lastPeriodTime,_maxPeriod; // 最新的电机运行周期 运行中电机的最大周期
      Timer _timer;
      vector<uint16_t> _motor_cnt;
      vector <bool> _motor_lost;                      //检测电机通讯是否断开
      

  protected:
      bool init();
      static void ecatcheck( void *ptr );
      static void degraded_handler();
      static void add_timespec(struct timespec *ts, int64_t addtime);
      static void ec_sync(int64_t reftime, int64_t cycletime , int64_t *offsettime);
      void data_resize();
      void read_pdo_data(uint16_t cnt);
      void send_pdo_cmd(uint16_t cnt);
      void ecat_sync_control();
      void ecat_send_receive();
      void map_CntToEcatid(uint16_t id,uint16_t cnt);
      void self_ethercat_run();  
      int  getEcatid_map(uint16_t id);
      float _period = 1000000; 
      bool _init_flag = false;
      bool _run_self = false;
      std::thread _thread_run; 
      vector<PROCBUFFER_OUT> _out_data; 
      vector<PROCBUFFER_IN>  _in_data;  

      map<uint16_t,uint16_t> _ecat_id_cnt;
      int64_t toff = 0;
      bool _ecatIdGetDone = false;
      std::mutex _data_mutex;
      std::mutex _para_mutex;
      // char _motor_slave_ecatName[100]{};
  };
}


#endif //PROJECT_RT_ETHERCAT_H
