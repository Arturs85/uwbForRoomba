
#ifndef UWBMSGSENDER_HPP
#define UWBMSGSENDER_HPP

#include <stdio.h>
#include <iostream>
#include <pthread.h>
#include <cstring>
#include <time.h>       /* clock_t, clock, CLOCKS_PER_SEC */
#include <termios.h>

extern "C" {
#include "deca_device_api.h"
#include "deca_regs.h"
#include "platform.h"

}
using namespace std;

class UwbMsgSender
{
public:
 pthread_t receivingThreadUwb;
static pthread_mutex_t mutexSend; 
static char tx_buffer[];
static int tx_size;
static struct termios orig_termios;


UwbMsgSender();
~UwbMsgSender();
static void initialize();
static void* sendingLoop(void* arg);
void startThread();
static void readDeviceData();


//static void* receive(void* arg);
static void* sendingLoop(void* arg);
void waitUwbThreadsEnd();

static void set_conio_terminal_mode();
static void reset_terminal_mode();
static int kbhit();
};






#endif //UWBMSGSENDER

