#include "ethercat_interface_r.h"

#define CLOCK_TO_USE CLOCK_REALTIME
#define TIMEOUT_CLEAR_ERROR (1 * TASK_FREQUENCY) /* clearing error timeout */
// 时间控制相关函数
#define NSEC_PER_SEC (1000000000L)
#define PERIOD_NS (NSEC_PER_SEC / TASK_FREQUENCY)
#define DIFF_NS(A, B) (((B).tv_sec - (A).tv_sec) * NSEC_PER_SEC + \
                       (B).tv_nsec - (A).tv_nsec)
#define TIMESPEC2NS(T) ((uint64_t)(T).tv_sec * NSEC_PER_SEC + (T).tv_nsec)
const struct timespec cycletime = {0, PERIOD_NS};
static struct timespec apptime;

static unsigned int sync_ref_counter = 0;

static ec_master_t* master = NULL;
static ec_master_state_t master_state = {};
static ec_domain_t* domain1 = NULL;
static ec_domain_state_t domain1_state = {};
uint8_t* domain1_pd = NULL;

bool running_thread = false;
bool isAllEnabled = false;
bool isAllReachedDefault = false;
int action_value_ = 0;
JointData rightArm;

// ethercat 电机内部功能所对应的地址偏移量
static struct
{
  unsigned int ctrl_word;
  unsigned int target_position;
  unsigned int target_torque;
  unsigned int target_velocity;
  unsigned int max_torque;
  unsigned int DO;
  unsigned int DI;
  unsigned int act_DI;

  unsigned int offset_velocity;
  unsigned int offset_torque;
  unsigned int offset_position;

  unsigned int error_code;
  unsigned int status_word;
  unsigned int act_position;
  unsigned int act_torque;

  unsigned int disp_mode;
  unsigned int actual_ferr;
  unsigned int probe_status;
  unsigned int probe1_pos;

  unsigned int probe2_pos;
  unsigned int act_velocity;
  unsigned int mode_Of_Operation;
  unsigned int mode_Of_Operation_dsiplay;
} offset[E_STOP + 1];

// 电机属性
struct {
  int defaultPosition = 0;
  int target_postion = 0;
  bool isInitedToDefault = false;
  bool isEnabled = false;
  uint16_t statusOld = 0;
  int statusDeCount = 5;
  int last_position = 0;
  int act_position = 0;
  int act_torque = 0;
  int act_velocity = 0;
  int act_wrong = 0;
} motorData[E_STOP + 1];

// ethercat 总线的PDO映射表,注册到主栈中
const static ec_pdo_entry_reg_t domain1_regs[] = {
#if E_START <= 0 && E_STOP >= 0
    {0, 0, VidPid, 0x6040, 0, &offset[0].ctrl_word},
    {0, 0, VidPid, 0x6071, 0, &offset[0].target_torque},
    {0, 0, VidPid, 0x607a, 0, &offset[0].target_position},
    {0, 0, VidPid, 0x60ff, 0, &offset[0].target_velocity},

    {0, 0, VidPid, 0x6041, 0, &offset[0].status_word},
    {0, 0, VidPid, 0x6064, 0, &offset[0].act_position},
    {0, 0, VidPid, 0x606c, 0, &offset[0].act_velocity},
    {0, 0, VidPid, 0x6077, 0, &offset[0].act_torque},
    {0, 0, VidPid, 0x6061, 0, &offset[0].mode_Of_Operation_dsiplay},
#endif
#if E_START <= 1 && E_STOP >= 1
    {0, 1, VidPid, 0x6040, 0, &offset[1].ctrl_word},
    {0, 1, VidPid, 0x6071, 0, &offset[1].target_torque},
    {0, 1, VidPid, 0x607a, 0, &offset[1].target_position},
    {0, 1, VidPid, 0x60ff, 0, &offset[1].target_velocity},

    {0, 1, VidPid, 0x6041, 0, &offset[1].status_word},
    {0, 1, VidPid, 0x6064, 0, &offset[1].act_position},
    {0, 1, VidPid, 0x606c, 0, &offset[1].act_velocity},
    {0, 1, VidPid, 0x6077, 0, &offset[1].act_torque},
    {0, 1, VidPid, 0x603f, 0, &offset[1].error_code},
    {0, 1, VidPid, 0x6061, 0, &offset[1].mode_Of_Operation_dsiplay},
#endif
#if E_START <= 2 && E_STOP >= 2
    {0, 2, VidPid, 0x6040, 0, &offset[2].ctrl_word},
    {0, 2, VidPid, 0x6071, 0, &offset[2].target_torque},
    {0, 2, VidPid, 0x607a, 0, &offset[2].target_position},
    {0, 2, VidPid, 0x60ff, 0, &offset[2].target_velocity},

    {0, 2, VidPid, 0x6041, 0, &offset[2].status_word},
    {0, 2, VidPid, 0x6064, 0, &offset[2].act_position},
    {0, 2, VidPid, 0x606c, 0, &offset[2].act_velocity},
    {0, 2, VidPid, 0x6077, 0, &offset[2].act_torque},
    {0, 2, VidPid, 0x603f, 0, &offset[2].error_code},
    {0, 2, VidPid, 0x6061, 0, &offset[2].mode_Of_Operation_dsiplay},
#endif
#if E_START <= 3 && E_STOP >= 3
    {0, 3, VidPid, 0x6040, 0, &offset[3].ctrl_word},
    {0, 3, VidPid, 0x6071, 0, &offset[3].target_torque},
    {0, 3, VidPid, 0x607a, 0, &offset[3].target_position},
    {0, 3, VidPid, 0x60ff, 0, &offset[3].target_velocity},

    {0, 3, VidPid, 0x6041, 0, &offset[3].status_word},
    {0, 3, VidPid, 0x6064, 0, &offset[3].act_position},
    {0, 3, VidPid, 0x606c, 0, &offset[3].act_velocity},
    {0, 3, VidPid, 0x6077, 0, &offset[3].act_torque},
    {0, 3, VidPid, 0x603f, 0, &offset[3].error_code},
    {0, 3, VidPid, 0x6061, 0, &offset[3].mode_Of_Operation_dsiplay},
#endif
#if E_START <= 4 && E_STOP >= 4
    {0, 4, VidPid, 0x6040, 0, &offset[4].ctrl_word},
    {0, 4, VidPid, 0x6071, 0, &offset[4].target_torque},
    {0, 4, VidPid, 0x607a, 0, &offset[4].target_position},
    {0, 4, VidPid, 0x60ff, 0, &offset[4].target_velocity},

    {0, 4, VidPid, 0x6041, 0, &offset[4].status_word},
    {0, 4, VidPid, 0x6064, 0, &offset[4].act_position},
    {0, 4, VidPid, 0x606c, 0, &offset[4].act_velocity},
    {0, 4, VidPid, 0x6077, 0, &offset[4].act_torque},
    {0, 4, VidPid, 0x603f, 0, &offset[4].error_code},
    {0, 4, VidPid, 0x6061, 0, &offset[4].mode_Of_Operation_dsiplay},
#endif
#if E_START <= 5 && E_STOP >= 5
    {0, 5, VidPid, 0x6040, 0, &offset[5].ctrl_word},
    {0, 5, VidPid, 0x6071, 0, &offset[5].target_torque},
    {0, 5, VidPid, 0x607a, 0, &offset[5].target_position},
    {0, 5, VidPid, 0x60ff, 0, &offset[5].target_velocity},

    {0, 5, VidPid, 0x6041, 0, &offset[5].status_word},
    {0, 5, VidPid, 0x6064, 0, &offset[5].act_position},
    {0, 5, VidPid, 0x606c, 0, &offset[5].act_velocity},
    {0, 5, VidPid, 0x6077, 0, &offset[5].act_torque},
    {0, 5, VidPid, 0x603f, 0, &offset[5].error_code},
    {0, 5, VidPid, 0x6061, 0, &offset[5].mode_Of_Operation_dsiplay},
#endif
#if E_START <= 6 && E_STOP >= 6
    {0, 6, VidPid, 0x6040, 0, &offset[6].ctrl_word},
    {0, 6, VidPid, 0x6071, 0, &offset[6].target_torque},
    {0, 6, VidPid, 0x607a, 0, &offset[6].target_position},
    {0, 6, VidPid, 0x60ff, 0, &offset[6].target_velocity},

    {0, 6, VidPid, 0x6041, 0, &offset[6].status_word},
    {0, 6, VidPid, 0x6064, 0, &offset[6].act_position},
    {0, 6, VidPid, 0x606c, 0, &offset[6].act_velocity},
    {0, 6, VidPid, 0x6077, 0, &offset[6].act_torque},
    {0, 6, VidPid, 0x603f, 0, &offset[6].error_code},
    {0, 6, VidPid, 0x6061, 0, &offset[6].mode_Of_Operation_dsiplay},
#endif

#if E_START <= 7 && E_STOP >= 7
    {0, 7, VidPid, 0x6040, 0, &offset[7].ctrl_word},
    {0, 7, VidPid, 0x6071, 0, &offset[7].target_torque},
    {0, 7, VidPid, 0x607a, 0, &offset[7].target_position},
    {0, 7, VidPid, 0x60ff, 0, &offset[7].target_velocity},

    {0, 7, VidPid, 0x6041, 0, &offset[7].status_word},
    {0, 7, VidPid, 0x6064, 0, &offset[7].act_position},
    {0, 7, VidPid, 0x606c, 0, &offset[7].act_velocity},
    {0, 7, VidPid, 0x6077, 0, &offset[7].act_torque},
    {0, 7, VidPid, 0x603f, 0, &offset[7].error_code},
    {0, 7, VidPid, 0x6061, 0, &offset[7].mode_Of_Operation_dsiplay},
#endif

#if E_START <= 8 && E_STOP >= 8
    {0, 8, VidPid, 0x6040, 0, &offset[8].ctrl_word},
    {0, 8, VidPid, 0x6071, 0, &offset[8].target_torque},
    {0, 8, VidPid, 0x607a, 0, &offset[8].target_position},
    {0, 8, VidPid, 0x60ff, 0, &offset[8].target_velocity},

    {0, 8, VidPid, 0x6041, 0, &offset[8].status_word},
    {0, 8, VidPid, 0x6064, 0, &offset[8].act_position},
    {0, 8, VidPid, 0x606c, 0, &offset[8].act_velocity},
    {0, 8, VidPid, 0x6077, 0, &offset[8].act_torque},
    {0, 8, VidPid, 0x603f, 0, &offset[8].error_code},
    {0, 8, VidPid, 0x6061, 0, &offset[8].mode_Of_Operation_dsiplay},
#endif

    // #if E_COUNT>X
    //     {0, XXX, VidPid, 0x6040, 0, &offset[XXX].ctrl_word},
    //     {0, XXX, VidPid, 0x6071, 0, &offset[XXX].target_torque},
    //     {0, XXX, VidPid, 0x607a, 0, &offset[XXX].target_position},
    //     {0, XXX, VidPid, 0x60ff, 0, &offset[XXX].target_velocity},

    //     {0, XXX, VidPid, 0x6041, 0, &offset[XXX].status_word},
    //     {0, XXX, VidPid, 0x6064, 0, &offset[XXX].act_position},
    //     {0, XXX, VidPid, 0x606c, 0, &offset[XXX].act_velocity},
    //     {0, XXX, VidPid, 0x6077, 0, &offset[XXX].act_torque},
    //     {0, XXX, VidPid, 0x6061, 0, &offset[XXX].mode_Of_Operation_dsiplay},
    // #endif
    ////
    {}};

// 伺服电机的PDO映射参数
ec_pdo_entry_info_t Igh_pdo_entries[] = {
    {0x6040, 0x00, 16},
    {0x6071, 0x00, 16},
    {0x607a, 0x00, 32},
    {0x60ff, 0x00, 32},

    {0x6041, 0x00, 16},
    {0x6064, 0x00, 32},
    {0x606c, 0x00, 32},
    {0x6077, 0x00, 16},
    {0x603f, 0x00, 16},
    {0x6061, 0x00, 8},
};

// 伺服电机PDO映射参数的组地址
ec_pdo_info_t Igh_pdos[] = {
    {0x1600, 4, Igh_pdo_entries + 0},
    {0x1a00, 6, Igh_pdo_entries + 4},
};

// IGH 将这个参数写入到电机中去
ec_sync_info_t Igh_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, Igh_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, Igh_pdos + 1, EC_WD_DISABLE},
    {0xFF}};

struct timespec timespec_add(struct timespec time1, struct timespec time2) {
  struct timespec result;

  if ((time1.tv_nsec + time2.tv_nsec) >= NSEC_PER_SEC) {
    result.tv_sec = time1.tv_sec + time2.tv_sec + 1;
    result.tv_nsec = time1.tv_nsec + time2.tv_nsec - NSEC_PER_SEC;
  } else {
    result.tv_sec = time1.tv_sec + time2.tv_sec;
    result.tv_nsec = time1.tv_nsec + time2.tv_nsec;
  }

  return result;
}

/*****************************************************************************/

void Igh_rechekTime() {
  clock_gettime(CLOCK_TO_USE, &apptime);

  ecrt_master_application_time(master, TIMESPEC2NS(apptime));
  if (sync_ref_counter) {
    sync_ref_counter--;
  } else {
    sync_ref_counter = 1;                      // sync every cycle
    ecrt_master_sync_reference_clock(master);  // DC reference clock drift compensation
  }
  ecrt_master_sync_slave_clocks(master);  // DC clock drift compensation
  // queue process data
  ecrt_domain_queue(domain1);
  // send EtherCAT data
  ecrt_master_send(master);
}

void toDisable(){
  for (int i2 = E_START; i2 <= E_STOP; i2++) {
    uint16_t ss = EC_READ_U16(domain1_pd + offset[i2].status_word);
    if (motorData[i2].statusOld != ss) {
      if ((ss == 0x1237 && motorData[i2].statusOld == 0x0237) || (ss == 0x0237 && motorData[i2].statusOld == 0x1237)) {
        // NO PRINT
      } else {
        printf("status %d : 0x%04x to 0x%04x  \n", i2, motorData[i2].statusOld, ss);
      }

      motorData[i2].statusOld = ss;
      motorData[i2].statusDeCount = 5;
    }    
    if ((ss & 0xFF) == 0x37) {
      EC_WRITE_U16(domain1_pd + offset[i2].ctrl_word, 0x07);
    }
    else if ((ss & 0xFF) == 0x33) {
      EC_WRITE_U16(domain1_pd + offset[i2].ctrl_word, 0x06);
    }  
    else if ((ss & 0xFF) == 0x21) {
      EC_WRITE_U16(domain1_pd + offset[i2].ctrl_word, 0x06);
    }  
  }
}

void toEnable() {
  int i2;
  isAllEnabled = true;

  for (i2 = E_START; i2 <= E_STOP; i2++) {
    uint16_t ss = EC_READ_U16(domain1_pd + offset[i2].status_word);
    if (motorData[i2].statusOld != ss) {
      if ((ss == 0x1237 && motorData[i2].statusOld == 0x0237) || (ss == 0x0237 && motorData[i2].statusOld == 0x1237)) {
        // NO PRINT
      } else {
        printf("status %d : 0x%04x to 0x%04x  \n", i2, motorData[i2].statusOld, ss);
      }

      motorData[i2].statusOld = ss;
      motorData[i2].statusDeCount = 5;
    }

    // 电机未使能,则执行状态机切换
    if ((ss & 0xFF) != 0x37) {
      motorData[i2].isEnabled = false;
      if (motorData[i2].statusDeCount == 5) {
        if ((ss & 0xFF) == 0x50) {
          EC_WRITE_U16(domain1_pd + offset[i2].ctrl_word, 0x06);
        } else if ((ss & 0xFF) == 0x31) {
          motorData[i2].target_postion = motorData[i2].act_position;
          EC_WRITE_S32(domain1_pd + offset[i2].target_position, motorData[i2].act_position);
          EC_WRITE_U16(domain1_pd + offset[i2].ctrl_word, 0x07);
        } else if ((ss & 0xFF) == 0x21) {
          motorData[i2].target_postion = motorData[i2].act_position;
          EC_WRITE_S32(domain1_pd + offset[i2].target_position, motorData[i2].act_position);
          EC_WRITE_U16(domain1_pd + offset[i2].ctrl_word, 0x07);
        } else if ((ss & 0xFF) == 0x33) {
          motorData[i2].target_postion = motorData[i2].act_position;
          EC_WRITE_S32(domain1_pd + offset[i2].target_position, motorData[i2].act_position);
          EC_WRITE_U16(domain1_pd + offset[i2].ctrl_word, 0x0F);
        }
      }
      if (motorData[i2].statusDeCount <= 0) {
        motorData[i2].statusDeCount = 5;
      } else {
        motorData[i2].statusDeCount--;
      }
    } else {
      motorData[i2].isEnabled = true;
    }
  }

  // 判断所有电机都使能
  for (i2 = E_START; i2 <= E_STOP; i2++) {
    isAllEnabled = (isAllEnabled && motorData[i2].isEnabled);
  }
}

void toDefaultPositions() {
  isAllReachedDefault = true;
  for (int i2 = E_START; i2 <= E_STOP; i2++) {
    uint16_t ss = EC_READ_U16(domain1_pd + offset[i2].status_word);
    if ((ss & 0xFF) == 0x37) {
      motorData[i2].last_position = motorData[i2].act_position;

      if (motorData[i2].isInitedToDefault == false) {
        if (motorData[i2].last_position > motorData[i2].defaultPosition + 80*3) {
          motorData[i2].target_postion = motorData[i2].target_postion - 120*3;
        } else if (motorData[i2].last_position < motorData[i2].defaultPosition - 80*3) {
          motorData[i2].target_postion = motorData[i2].target_postion + 120*3;
        } else {
          motorData[i2].isInitedToDefault = true;
        }
        printf("position + %d,  %d to %d  \n", i2, motorData[i2].last_position, motorData[i2].target_postion);
        EC_WRITE_S32(domain1_pd + offset[i2].target_position, motorData[i2].target_postion);
      } else {
        // printf("isInitedToDefault + %d  \n", i2);
      }
    } else {
      uint16_t wrong = EC_READ_U16(domain1_pd + offset[i2].error_code);
      printf("get wrong code:  %d , at joint %d , so the status:  %d   \n", wrong ,i2,ss);
    }
  }

  // 判断所有电机都使能
  for (int i2 = E_START; i2 <= E_STOP; i2++) {
    isAllReachedDefault = (isAllReachedDefault && motorData[i2].isInitedToDefault);
  }  
}

void toPositions() {
  if(rightArm.getSize()>0){
    std::array<float, 9> arr_out;
    rightArm.dequeue(arr_out);
    for (int i2 = E_START; i2 <= E_STOP; i2++) {
      uint16_t ss = EC_READ_U16(domain1_pd + offset[i2].status_word);
      if ((ss & 0xFF) == 0x37) {
        motorData[i2].target_postion = (int)arr_out[i2];
        EC_WRITE_S32(domain1_pd + offset[i2].target_position, motorData[i2].target_postion);
      } else {
        uint16_t wrong = EC_READ_U16(domain1_pd + offset[i2].error_code);
        printf("get wrong code:  %d , at joint %d , so the status:  %d , to postion %d , current postion %d \n", wrong ,i2,ss, motorData[i2].target_postion , motorData[i2].act_position);
      }
    }
  }
}

// EtherCat线程读写实例
void Ethercat_syncThread() {
  running_thread = true;
  // if(2>1)return 0;
  struct timespec wakeupTime, time;
  // get current time
  clock_gettime(CLOCK_TO_USE, &wakeupTime);

  while (running_thread) {
    wakeupTime = timespec_add(wakeupTime, cycletime);
    clock_nanosleep(CLOCK_TO_USE, TIMER_ABSTIME, &wakeupTime, NULL);
    ecrt_master_receive(master);
    ecrt_domain_process(domain1);
    for (int i2 = E_START; i2 <= E_STOP; i2++) {
      motorData[i2].act_position = EC_READ_S32(domain1_pd + offset[i2].act_position);
      motorData[i2].act_velocity = EC_READ_S32(domain1_pd + offset[i2].act_velocity);
      motorData[i2].act_torque = EC_READ_S16(domain1_pd + offset[i2].act_torque);
      rightArm.data[i2 * 3 + 0] = motorData[i2].act_position;
      rightArm.data[i2 * 3 + 1] = motorData[i2].act_velocity;
      rightArm.data[i2 * 3 + 2] = motorData[i2].act_torque;
    }

    if (action_value_ == 2) { 
      if (!isAllEnabled) {
        rightArm.status = 2;
        toEnable();
      } 
      else{
        rightArm.status = 202;
      }
    }
    else if(action_value_ == 3){
      if (!isAllReachedDefault) {
        rightArm.status = 3;
        toDefaultPositions();
      } 
      else{
        rightArm.status = 203;
      }
    }
    else if(action_value_ == 4){
      if (rightArm.getSize()>0) {
        rightArm.status = 4;
        toPositions();
      } 
      else{
        rightArm.status = 204;
      }
    }    
    else if(action_value_ == 5){
      toDisable();
      rightArm.status = 5;
    }      

    Igh_rechekTime();
  }
}

void consoleJoints() {
  for (int i2 = E_START; i2 <= E_STOP; i2++) {
    printf("%d \n ", motorData[i2].act_position);
  }
}

//TODO
void consoleQueue(){}

void Igh_master_activate() {
  printf("......Activating master......\n");
  if (ecrt_master_activate(master)) {
    exit(EXIT_FAILURE);
  }
  if (!(domain1_pd = ecrt_domain_data(domain1))) {
    exit(EXIT_FAILURE);
  }

  printf("......Master  Activated.......\n");



motorData[2].defaultPosition = 4575;
motorData[3].defaultPosition =  -38361;
motorData[4].defaultPosition = -1466537;
motorData[5].defaultPosition = 120834;
motorData[6].defaultPosition =  1472626;
motorData[7].defaultPosition = 88657;
motorData[8].defaultPosition = -61876;
}

/****************************************************************************/

void Igh_init() {
  ec_master_info_t master_info;
  ec_slave_info_t slave_info;
  int ret;
  int slavecnt;
  ec_slave_config_t* sc;

  // uint32_t  abort_code;
  // size_t rb;
  int ii = 0;
  master = ecrt_request_master(E_MASTER);
  if (!master) {
    exit(EXIT_FAILURE);
  }

  domain1 = ecrt_master_create_domain(master);
  if (!domain1) {
    exit(EXIT_FAILURE);
  }

  //---------get master / salve info----------------------
  ret = ecrt_master(master, &master_info);
  slavecnt = master_info.slave_count;
  printf("ret = %d, slavecnt = %d, apptime = %" PRIu64 "\n", ret, master_info.slave_count, master_info.app_time);
  ret = ecrt_master_get_slave(master, 0, &slave_info);
  printf("ret = %d,spos = %d, pcode = %d\n", ret, slave_info.position, slave_info.product_code);

  //---------end get master / salve info----------------------

  for (ii = E_START; ii <= E_STOP; ii++) {
    printf("servo %d  begin init! \n", ii);
    ec_slave_config_t* sc;
    if (!(sc = ecrt_master_slave_config(master, 0, ii, VidPid))) {
      fprintf(stderr, "Failed to get slave configuration for Igh.\n");
      exit(EXIT_FAILURE);
    }

    printf("Configuring PDOs... %d \n", ii);
    if (ecrt_slave_config_pdos(sc, EC_END, Igh_syncs)) {
      fprintf(stderr, "Failed to configure Igh PDOs.\n");
      exit(EXIT_FAILURE);
    }

    // ecrt_slave_config_sdo16(sc, 0x6040, 0, 6);
    // sleep(1);
    ecrt_slave_config_sdo16(sc, 0x6040, 0, 128);  // 清错
    sleep(1);

    ecrt_slave_config_sdo16(sc, 0x1C32, 1, 2);  // set output synchronization triggered by  sync0 event DC mode
    ecrt_slave_config_sdo16(sc, 0x1C33, 1, 2);  // set input  synchronization triggered by  sync1 evnent DC mode
    ecrt_slave_config_sdo8(sc, 0x60c2, 1, (1000 / TASK_FREQUENCY));
    ecrt_slave_config_sdo8(sc, 0x60c2, 2, -3);
    ecrt_slave_config_sdo8(sc, 0x6060, 0, 8);
    ecrt_slave_config_dc(sc, 0x0300, PERIOD_NS, PERIOD_NS / 2, 0, 0);
  }

  if (ecrt_domain_reg_pdo_entry_list(domain1, domain1_regs)) {
    fprintf(stderr, "PDO entry registration failed!\n");
    exit(EXIT_FAILURE);
  }
}