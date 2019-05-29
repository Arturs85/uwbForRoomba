
#ifndef UWBRECEIVER_HPP
#define UWBRECEIVER_HPP

#include <stdio.h>
#include <unistd.h>			//Used for UART
#include <fcntl.h>			//Used for UART
#include <termios.h>	 	//Used for UART
#include <pthread.h>
#include <cstring>

using namespace std;

class UwbReceiver
{
public:
static int uart0_filestream;
 pthread_t receivingThreadUart;
 pthread_t sendingThreadUart;
static pthread_mutex_t mutexSend; 
static char tx_buffer[];
static int tx_size;

UartTest();
~UartTest();
void initialize();
void send();
void startReceiveing();
static void setDataToTransmit(char* data, int size);

static void* receive(void* arg);
static void* sendingLoop(void* arg);
void waitUartThreadsEnd();

};






#endif //UWBRECEIVER_HPP

