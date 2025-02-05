#ifndef ETHERCAT_INTERFACE_L_H
#define ETHERCAT_INTERFACE_L_H

#include "ecrt.h"
#include <stdio.h>
#include <unistd.h>
#include <sched.h>
#include <sys/mman.h>
#include <errno.h>
#include <string.h>
#include <sys/resource.h>
#include <time.h>
#include <sys/types.h>
#include <sys/time.h>
#include <inttypes.h>
#include <math.h>
#include "joint_data.h"

#define TASK_FREQUENCY 125 /* Hz */
#define E_START 0
#define E_STOP 6
#define E_MASTER 1

#define VidPid 0x00522227, 0x00009253

bool running_thread = false;
bool isAllEnabled = false;
bool isAllReachedDefault = false;
int action_value_ = 0;
JointData leftArm;

void Ethercat_syncThread();
void Igh_master_activate();
void Igh_init();
void toDefaultPositions();
void toPositionsByQueue();
void toEnable();
void toDisable();
void consoleJoints();
void consoleQueue();
void Igh_rechekTime();
struct timespec timespec_add(struct timespec time1, struct timespec time2);

#endif // ETHERCAT_INTERFACE_L_H
