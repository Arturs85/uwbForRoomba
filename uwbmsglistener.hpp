
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
	typedef struct RawTxMessage {char macHeader[10]; char data[127]; int dataLength; } RawTxMessage;
typedef unsigned long long uint64;

class UwbMsgListener
{

public:
static int uart0_filestream;
 pthread_t receivingThreadUwb;
 pthread_t sendingThreadUwb;
 static pthread_mutex_t txBufferLock;
 static pthread_mutex_t rangingInitBufferLock;

 static pthread_mutex_t dwmDeviceLock;
 
//static pthread_mutex_t mutexSend; 
static char tx_buffer[];
static int tx_size;//Unused?
static struct termios orig_termios;
static bool isSending;
static bool isReceivingThreadRunning;
//size_t rawTxMessageNominalSize = 150;// header+127=?
static std::deque<RawTxMessage> txDeque;
static std::deque<int> rangingInitDeque;


UwbMsgListener();
~UwbMsgListener();
static void initialize();
static void* receivingLoop(void* arg);

void startReceiving();
void startSending();
void stopSending();
static void readDeviceData();


static void* receive(void* arg);
static void addToTxDeque(std::string msgText);
static void addToRangingInitDeque(int rangingTarget);
static void respondToRangingRequest();
static void initiateRanging();

//static void final_msg_get_ts(uint8 *ts_field, uint64 ts);


static void* sendingLoop(void* arg);
void waitUwbThreadsEnd();

static void set_conio_terminal_mode();
static void reset_terminal_mode();
static int kbhit();
};






#endif //UWBMSGLISTENER

