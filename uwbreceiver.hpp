
#ifndef UWBRECEIVER_HPP
#define UWBRECEIVER_HPP

#include <stdio.h>

#include <pthread.h>
#include <cstring>
#include <time.h>       /* clock_t, clock, CLOCKS_PER_SEC */
extern "C" {
#include "deca_device_api.h"
#include "deca_regs.h"
#include "platform.h"
}
using namespace std;

class UwbReceiver
{
public:
static int uart0_filestream;
 pthread_t receivingThreadUwb;
static pthread_mutex_t mutexSend; 
static char tx_buffer[];
static int tx_size;


UwbReceiver();
~UwbReceiver();
static void initialize();
static void* receivingLoop(void* arg);
void startReceiveing();


void send();
static void setDataToTransmit(char* data, int size);

static void* receive(void* arg);
static void* sendingLoop(void* arg);
void waitUwbThreadsEnd();

};






#endif //UWBRECEIVER_HPP

