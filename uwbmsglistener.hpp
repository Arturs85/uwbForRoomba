
#ifndef UWBMSGLISTENER_HPP
#define UWBMSGLISTENER_HPP

#include <stdio.h>
#include <iostream>
#include <pthread.h>
#include <cstring>
#include <time.h>       /* clock_t, clock, CLOCKS_PER_SEC */
#include <termios.h>
#include <deque>

extern "C" {
#include "deca_device_api.h"
#include "deca_regs.h"
#include "platform.h"

}
using namespace std;

class UwbMsgListener
{
public:
static int uart0_filestream;
 pthread_t receivingThreadUwb;
 pthread_t sendingThreadUwb;
 pthread_mutex_t txBufferLock;
 
//static pthread_mutex_t mutexSend; 
static char tx_buffer[];
static int tx_size;
static struct termios orig_termios;
static bool isSending;
static bool isReceivingThreadRunning;
//size_t rawTxMessageNominalSize = 150;// header+127=?
typedef struct RawTxMessage {char macHeader[10]; char data[127]; } RawTxMessage;
static std::deque<RawTxMessage> txDeque;


UwbMsgListener();
~UwbMsgListener();
static void initialize();
static void* receivingLoop(void* arg);

void startReceiving();
void stopSending();
static void readDeviceData();


static void* receive(void* arg);
static void send();


static void* sendingLoop(void* arg);
void waitUwbThreadsEnd();

static void set_conio_terminal_mode();
static void reset_terminal_mode();
static int kbhit();
};






#endif //UWBMSGLISTENER

