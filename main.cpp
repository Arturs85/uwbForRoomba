	
	//#include "uwbreceiver.hpp"
	#include "uwbmsglistener.hpp"
	
	#include <string>
	#include <iostream>
	#include <fstream>
	#include <cstring>
	#include <linux/input.h>
	
	int main(int argc, char** argv)
	{
	printf("main started\n");
	
	//UwbReceiver uwbreceiver;
	UwbMsgListener uwbMsgListener;
	//uwbreceiver.initialize();
	uwbMsgListener.initialize();
	//uwbMsgListener.readDeviceData();
	//uartTest.send();
	//uwbreceiver.startReceiveing();
	
	std::string tmp = "test data";
	char *arr = &tmp[0];
	//uartTest.setDataToTransmit(arr, 9);
	//keyboard reading
	
	while(1){
	std::string userCommand;
	printf( "enter chars!\n");
	getline (cin, userCommand);
	char *ar = &userCommand[0];
	//printf("first char you entered: %d", (uint8_t)(*ar));
	//uartTest.setDataToTransmit(ar, userCommand.size());
	
	
	if(userCommand.find("send")!=std::string::npos)
	{
	uwbMsgListener.addToTxDeque(userCommand.substr(4,std::string::npos));
	}
	else if(!userCommand.compare("start"))
	{
	uwbMsgListener.startReceiving();
	uwbMsgListener.startSending();
	}
	else if(!userCommand.compare("end"))
	{
	break;
	}
	}//while
	//uwbreceiver.waitUwbThreadsEnd();
	uwbMsgListener.waitUwbThreadsEnd();
	return 0;
	}
	
	
