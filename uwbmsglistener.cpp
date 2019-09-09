/*! ----------------------------------------------------------------------------
 *  @file    main.c
 *  @brief   Double-sided two-way ranging (DS TWR) initiator example code
 *
 *           This is a simple code example which acts as the initiator in a DS TWR distance measurement exchange. This application sends a "poll"
 *           frame (recording the TX time-stamp of the poll), and then waits for a "response" message expected from the "DS TWR responder" example
 *           code (companion to this application). When the response is received its RX time-stamp is recorded and we send a "final" message to
 *           complete the exchange. The final message contains all the time-stamps recorded by this application, including the calculated/predicted TX
 *           time-stamp for the final message itself. The companion "DS TWR responder" example application works out the time-of-flight over-the-air
 *           and, thus, the estimated distance between the two devices.
 *
 * @attention
 *
 * Copyright 2015 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author Decawave
 */
#include <string>
#include <stdio.h>
#include "uwbmsglistener.hpp"

#include <stdlib.h>
#include <unistd.h>
#include <sys/select.h>
#include <termios.h>

#include <unistd.h>
#include <limits.h>

/* Example application name and version to display on LCD screen. */
#define APP_NAME "UWB Listener"
#define SLEEP_BETWEEN_SENDING_US 100000
/* Inter-ranging delay period, in milliseconds. */
#define RNG_DELAY_MS 1000

/* Default communication configuration. We use here EVK1000's default mode (mode 3). */
static dwt_config_t config = {
    2,               /* Channel number. */
    DWT_PRF_64M,     /* Pulse repetition frequency. */
    DWT_PLEN_2048,//1024,   /* Preamble length. Used in TX only. */
    DWT_PAC64,       /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    1,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
     DWT_BR_110K,//DWT_BR_6M8,//DWT_BR_110K,     /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (2049 + 64 - 64) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};



/* Frames used in the ranging process. See NOTE 2 below. */
static uint8 rx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x21, 0, 0};
static uint8 tx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0x10, 0x02, 0, 0, 0, 0};
static uint8 rx_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8 tx_poll_mod_msg[] = {0x41, 0x88, 0x41, 0xCA, 0xDE, 'V', 'I', 'A', 'B', 0x21, 0, 0};
//from initiaator.c
static uint8 tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x21, 0, 0};
static uint8 rx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0x10, 0x02, 0, 0, 0, 0};
static uint8 tx_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
/* Length of the common part of the message (up to and including the function code, see NOTE 2 below). */
#define ALL_MSG_COMMON_LEN 10
/* Index to access some of the fields in the frames involved in the process. */
#define ALL_MSG_SN_IDX 2
#define FINAL_MSG_POLL_TX_TS_IDX 10
#define FINAL_MSG_RESP_RX_TS_IDX 14
#define FINAL_MSG_FINAL_TX_TS_IDX 18
#define FINAL_MSG_TS_LEN 4
/* Frame sequence number, incremented after each transmission. */
static uint8 frame_seq_nb = 0;

/* Buffer to store received messages.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 24
static uint8 rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32 status_reg = 0;

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
 * 1 uus = 512 / 499.2 �s and 1 �s = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536

//from responder.c
/* Delay between frames, in UWB microseconds. See NOTE 4 below. */
/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW1000's delayed TX function. This includes the
 * frame length of approximately 2.46 ms with above configuration. */
#define POLL_RX_TO_RESP_TX_DLY_UUS 7000//2800//2600
/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW1000's wait for response feature. */
#define RESP_TX_TO_FINAL_RX_DLY_UUS 700//500
/* Receive final timeout. See NOTE 5 below. */
#define FINAL_RX_TIMEOUT_UUS 9000//3300
/* Preamble timeout, in multiple of PAC size. See NOTE 6 below. */
#define PRE_TIMEOUT 256

//from initiator.c
/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW1000's wait for response feature. */
#define POLL_TX_TO_RESP_RX_DLY_UUS 200//150
/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW1000's delayed TX function. This includes the
 * frame length of approximately 2.66 ms with above configuration. */
#define RESP_RX_TO_FINAL_TX_DLY_UUS 7000//3100
/* Receive response timeout. See NOTE 5 below. */
#define RESP_RX_TIMEOUT_UUS 9000//2900//2700
/* Preamble timeout, in multiple of PAC size. See NOTE 6 below. */
//#define PRE_TIMEOUT 64//8

#define TX_ANT_DLY 16436
#define RX_ANT_DLY 16436

/* Timestamps of frames transmission/reception.
 * As they are 40-bit wide, we need to define a 64-bit int type to handle them. */
typedef signed long long int64;
//typedef unsigned long long uint64;
static uint64 poll_rx_ts;
static uint64 resp_tx_ts;
static uint64 final_rx_ts;
//from initiator.c
static uint64 poll_tx_ts;
static uint64 resp_rx_ts;
static uint64 final_tx_ts;
/* Speed of light in air, in metres per second. */
#define SPEED_OF_LIGHT 299702547

/* Hold copies of computed time of flight and distance here for reference so that it can be examined at a debug breakpoint. */
static double tof;
static double twrDistance;

/* Declaration of static functions. */
static uint64 get_tx_timestamp_u64(void);
static uint64 get_rx_timestamp_u64(void);
static void final_msg_get_ts(const uint8 *ts_field, uint32 *ts);
static void final_msg_set_ts(uint8 *ts_field, uint64 ts);


pthread_mutex_t UwbMsgListener::txBufferLock = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t UwbMsgListener::dwmDeviceLock = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t UwbMsgListener::rangingInitBufferLock = PTHREAD_MUTEX_INITIALIZER;

std::deque<RawTxMessage> UwbMsgListener::txDeque; 
std::deque<int> UwbMsgListener::rangingInitDeque;

//struct termios orig_termios;

struct termios UwbMsgListener::orig_termios;

void UwbMsgListener::reset_terminal_mode()
{
    tcsetattr(0, TCSANOW, &orig_termios);
}

void UwbMsgListener::set_conio_terminal_mode()
{
    struct termios new_termios;

    /* take two copies - one for now, one for later */
    tcgetattr(0, &orig_termios);
    memcpy(&new_termios, &orig_termios, sizeof(new_termios));

    /* register cleanup handler, and set the new terminal mode */
    atexit(reset_terminal_mode);
    cfmakeraw(&new_termios);
    tcsetattr(0, TCSANOW, &new_termios);
}

int UwbMsgListener::kbhit()
{
    struct timeval tv = { 0L, 0L };
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(0, &fds);
    return select(1, &fds, NULL, NULL, &tv);
}

// uint8 report_average = 0;
//  float sample_buffer[200] = {0};
// int next_sample_pos = 0, total_sample_count = 0, max_sample_count = 0;

UwbMsgListener::UwbMsgListener()
{
}
UwbMsgListener::~UwbMsgListener()
{
}


void UwbMsgListener::readDeviceData(){
    int sleepdelayus = 10;
    uint32 lotid = dwt_getlotid();
    uint32 partid = dwt_getpartid();

    printf("Lot ID: %x, Part ID: %x\n",lotid,partid);

    uint8 euiVal[8]={0,0,0,0,0,0,0,0};

    dwt_geteui(&euiVal[0]);

    printf("EUI: %x %x %x %x %x %x %x %x\n",euiVal[0],euiVal[1],euiVal[2],euiVal[3],euiVal[4],euiVal[5],euiVal[6],euiVal[7]);

    uint16 p = 0x1234;
    //dwt_setpanid(p);
    dwt_write32bitoffsetreg(0x1E, TX_POWER_LEN, 0x1f1f1f1f) ;

    //dwt_write16bitoffsetreg(0x03, PANADR_PAN_ID_OFFSET, p) ;
    usleep(1);
    uint16 panId =0;
    panId= dwt_read16bitoffsetreg(PANADR_ID,PANADR_PAN_ID_OFFSET);
    printf("PAN ID h: %x\n",panId);
    usleep(1);

    uint16 shortAdr = dwt_read16bitoffsetreg(PANADR_ID, PANADR_SHORT_ADDR_OFFSET);
    printf("Short adress h: %x\n",shortAdr);
    usleep(sleepdelayus);

    uint32 sysConfig = dwt_read32bitoffsetreg(SYS_CFG_ID, 0);
    printf("System configuration h: %x\n",sysConfig);
    usleep(sleepdelayus);

    uint32 txFctrl1 = dwt_read32bitoffsetreg(TX_FCTRL_ID,0);
    printf("Tarnsmit frame control 1 h: %x\n",txFctrl1);
    usleep(sleepdelayus);

    uint32 txFctrl2 = dwt_read32bitoffsetreg(TX_FCTRL_ID,4);
    printf("Tarnsmit frame control 2 h: %x\n",txFctrl2);
    usleep(sleepdelayus);

    uint16 rxFwto = dwt_read16bitoffsetreg(RX_FWTO_ID,RX_FWTO_OFFSET);
    printf("Receive frame wait timeout period h: %x\n",rxFwto);
    usleep(sleepdelayus);

    uint32 sysCtrl = dwt_read32bitoffsetreg(SYS_CTRL_ID,0);
    printf("System control register h: %x\n",sysCtrl);
    usleep(sleepdelayus);

    uint32 sysMask = dwt_read32bitoffsetreg(SYS_MASK_ID,0);
    printf("System event mask register h: %x\n",sysMask);
    usleep(sleepdelayus);

    uint32 sysStatus1 = dwt_read32bitoffsetreg(SYS_STATUS_ID,0);
    printf("System event status register 1 h: %x\n",sysStatus1);
    usleep(sleepdelayus);

    uint32 sysStatus2 = dwt_read32bitoffsetreg(SYS_STATUS_ID,4);
    printf("System event status register 2 h: %x\n",sysStatus2);
    usleep(sleepdelayus);

    uint32 sysState1 = dwt_read32bitoffsetreg(SYS_STATE_ID,0);
    printf("System state information 1 h: %x\n",sysState1);
    usleep(sleepdelayus);

    uint32 sysState2 = dwt_read32bitoffsetreg(SYS_STATE_ID,4);
    printf("System state information 2 h: %x\n",sysState2);
    usleep(sleepdelayus);

    uint32 ackRespTime = dwt_read32bitoffsetreg(ACK_RESP_T_ID,0);
    printf("Acknowledgement time and response time h: %x\n",ackRespTime);
    usleep(sleepdelayus);

    uint32 rxSniff = dwt_read32bitoffsetreg(RX_SNIFF_ID,0);
    printf("Pulsed preamble reception config h: %x\n",rxSniff);
    usleep(sleepdelayus);

    uint32 txPower = dwt_read32bitoffsetreg(TX_POWER_ID,0);
    printf("Tx power control h: %x\n",txPower);
    usleep(sleepdelayus);

    uint32 chanelControl = dwt_read32bitoffsetreg(CHAN_CTRL_ID,0);
    printf("Channel controlh: %x\n",chanelControl);
    usleep(sleepdelayus);


}

void UwbMsgListener::initialize()
{



    char hostname[HOST_NAME_MAX];
    int res =gethostname(hostname, HOST_NAME_MAX);
    if(!res)
        printf ("device hostname: %s\n",hostname);
    else
        printf ("unable to get device hostname\n");

    //if (argc > 2 && strcmp(argv[1], "average") == 0) {
    //report_average = 1;
    //set_conio_terminal_mode();
    //max_sample_count = atoi(argv[2]);
    //if (max_sample_count < 1 || max_sample_count > 200) {
    //printf("Sample count must be between 1 and 200!\n");
    //return(1);
    //}
    //}
    /* Start with board specific hardware init. */
    peripherals_init();

    /* Display application name on LCD. */
    printf(APP_NAME "\n");

    /* Reset and initialise DW1000.
     * For initialisation, DW1000 clocks must be temporarily set to crystal speed. After initialisation SPI rate can be increased for optimum
     * performance. */
    reset_DW1000(); /* Target specific drive of RSTn line into DW1000 low for a period. */
    spi_set_rate_low();
    if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)
    {
        printf("INIT FAILED\n");
        return;
    }
    spi_set_rate_high();
    readDeviceData();
    /* Configure DW1000. See NOTE 7 below. */
    dwt_configure(&config);

    readDeviceData();
    /* Apply default antenna delay value. See NOTE 1 below. */
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);

    /* Set preamble timeout for expected frames. See NOTE 6 below. */
    dwt_setpreambledetecttimeout(PRE_TIMEOUT);

}

bool UwbMsgListener::isReceivingThreadRunning =1;

void *UwbMsgListener::receivingLoop(void *arg)
{
    clock_t t;
    initialize();
    uint32_t cycleCounter= 0;
    /* Loop forever initiating ranging exchanges. */
    while (isReceivingThreadRunning)
    {
        pthread_mutex_lock(&dwmDeviceLock); //wait until device is free to use

        /* Clear reception timeout to start next ranging process. */
        dwt_setrxtimeout(0);

        /* Activate reception immediately. */
        dwt_rxenable(DWT_START_RX_IMMEDIATE);

        /* Poll for reception of a frame or error/timeout. See NOTE 8 below. */
        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
        { };

        if (status_reg & SYS_STATUS_RXFCG)
        {
            uint32 frame_len=0;

            /* Clear good RX frame event in the DW1000 status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

            /* A frame has been received, read it into the local buffer. */
            frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
            cout<<"received frame with length "<<frame_len<<"\n";
            
            if (frame_len <= RX_BUFFER_LEN)
            {
                dwt_readrxdata(rx_buffer, frame_len, 0);
            }

            /* Check that the frame is a poll sent by "DS TWR initiator" example.
             * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
            rx_buffer[ALL_MSG_SN_IDX] = 0;

            if (memcmp(rx_buffer, rx_poll_msg, ALL_MSG_COMMON_LEN) == 0)
                respondToRangingRequest();
            else
            {
                std::size_t length = frame_len;
                char* rxData = reinterpret_cast<char*>(rx_buffer);
                string rxString= string(rxData,length);
                cout<<"rx: "<<rxString<<"\n";

                t = clock();
                //usleep(100000);

            }
        }
        else
        {
            /* Clear RX error/timeout events in the DW1000 status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

            /* Reset RX to properly reinitialise LDE operation. */
            dwt_rxreset();
            //               cout<<"rx error/timeout \n";

        }
        // usleep(1000000);
        //cout<<"UwbMsgListener thread running"<< cycleCounter++<<"\n";
        pthread_mutex_unlock(&dwmDeviceLock); // now sending thread can use device

    }
}

bool UwbMsgListener::isSending =0;

void *UwbMsgListener::sendingLoop(void *arg)
{
    while(isSending){
        pthread_mutex_lock(&txBufferLock); //take and block access to tx buffer

        if(!txDeque.empty())// if there is any message for sending
        {
            pthread_mutex_unlock(&txBufferLock); //release tx buffer

            pthread_mutex_lock(&dwmDeviceLock); //wait until device is free to use
            pthread_mutex_lock(&txBufferLock); //lock tx buffer once dwm is released

            RawTxMessage msg = txDeque.back();
            dwt_writetxdata(ALL_MSG_COMMON_LEN, (uint8*)msg.macHeader, 0); /* Zero offset in TX buffer. */
            dwt_writetxdata(ALL_MSG_COMMON_LEN+msg.dataLength+2, (uint8*)msg.data, ALL_MSG_COMMON_LEN); /* header offset in TX buffer. */

            dwt_writetxfctrl(msg.dataLength+ALL_MSG_COMMON_LEN+2, 0, 0); /* Zero offset in TX buffer, no ranging. */
            txDeque.pop_back();// delete message after sending it(after writing it to tx buffer)
            dwt_starttx(DWT_START_TX_IMMEDIATE );

            pthread_mutex_unlock(&dwmDeviceLock); //release device for receiving

        }
        pthread_mutex_unlock(&txBufferLock); //release blocking access to tx buffer
        //initiate ranging if there is any requests

        pthread_mutex_lock(&rangingInitBufferLock); //take and block access to ranging initation Deque buffer
        if(!rangingInitDeque.empty()){

            int rangingTargetNr = rangingInitDeque.back();
            rangingInitDeque.pop_back();// remove ranging request
            pthread_mutex_unlock(&rangingInitBufferLock); //release blocking access to ranging initation Deque  buffer

            pthread_mutex_lock(&dwmDeviceLock); //wait until device is free to use

            initiateRanging();// later target will be added as parameter

            pthread_mutex_unlock(&dwmDeviceLock); //release device for receiving

        }
        else
        pthread_mutex_unlock(&rangingInitBufferLock); //release blocking access to ranging initation Deque  buffer

        usleep(SLEEP_BETWEEN_SENDING_US);
    }
}

void UwbMsgListener::addToTxDeque(std::string msgText){
    //while(isSending){
    RawTxMessage msg;

    tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
    memcpy(msg.macHeader,tx_poll_mod_msg,ALL_MSG_COMMON_LEN);

    snprintf(msg.data,30," --message nr %d",frame_seq_nb++);
    
    std::string s((char*)msg.data);
    //cout<< s<<"\n";
    msgText.append(s);
    
    memcpy(msg.data, msgText.c_str(),msgText.size() );
    msg.dataLength = msgText.size();

    pthread_mutex_lock(&txBufferLock); //take and block access to tx buffer
    txDeque.push_front(msg);
    pthread_mutex_unlock(&txBufferLock);

}

void UwbMsgListener::addToRangingInitDeque(int rangingTarget)
{
    pthread_mutex_lock(&rangingInitBufferLock); //take and block access to tx buffer
    rangingInitDeque.push_front(rangingTarget);
    pthread_mutex_unlock(&rangingInitBufferLock);

}

void UwbMsgListener::respondToRangingRequest()
{
    uint32 resp_tx_time;
    int ret;
    printf("resp received poll msg\n");

    /* Retrieve poll reception timestamp. */
    poll_rx_ts = get_rx_timestamp_u64();

    /* Set send time for response. See NOTE 9 below. */
    resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
    dwt_setdelayedtrxtime(resp_tx_time);

    /* Set expected delay and timeout for final message reception. See NOTE 4 and 5 below. */
    dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);
    dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);

    /* Write and send the response message. See NOTE 10 below.*/
    tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
    dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0); /* Zero offset in TX buffer. */
    dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1); /* Zero offset in TX buffer, ranging. */
    ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);

    /* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. See NOTE 11 below. */
    if (ret == DWT_ERROR)
    {
        printf("tx1 err\n");
        return; // replaced continue with return, to be tested
    }

    /* Poll for reception of expected "final" frame or error/timeout. See NOTE 8 below. */
    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
    { };
    printf("statusReg h: %x mask: %x\n",status_reg,(SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR));

    /* Increment frame sequence number after transmission of the response message (modulo 256). */
    frame_seq_nb++;

    if (status_reg & SYS_STATUS_RXFCG)
    {
        /* Clear good RX frame event and TX frame sent in the DW1000 status register. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);
        printf("final msg received\n");

        /* A frame has been received, read it into the local buffer. */
       uint32 frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
        if (frame_len <= RX_BUF_LEN)
        {
            dwt_readrxdata(rx_buffer, frame_len, 0);
        }

        /* Check that the frame is a final message sent by "DS TWR initiator" example.
                       * As the sequence number field of the frame is not used in this example, it can be zeroed to ease the validation of the frame. */
        rx_buffer[ALL_MSG_SN_IDX] = 0;
        if (memcmp(rx_buffer, rx_final_msg, ALL_MSG_COMMON_LEN) == 0)
        {
            uint32 poll_tx_ts, resp_rx_ts, final_tx_ts;
            uint32 poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
            double Ra, Rb, Da, Db;
            int64 tof_dtu;

            /* Retrieve response transmission and final reception timestamps. */
            resp_tx_ts = get_tx_timestamp_u64();
            final_rx_ts = get_rx_timestamp_u64();

            /* Get timestamps embedded in the final message. */
            final_msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX], &poll_tx_ts);
            final_msg_get_ts(&rx_buffer[FINAL_MSG_RESP_RX_TS_IDX], &resp_rx_ts);
            final_msg_get_ts(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts);

            /* Compute time of flight. 32-bit subtractions give correct answers even if clock has wrapped. See NOTE 12 below. */
            poll_rx_ts_32 = (uint32)poll_rx_ts;
            resp_tx_ts_32 = (uint32)resp_tx_ts;
            final_rx_ts_32 = (uint32)final_rx_ts;
            Ra = (double)(resp_rx_ts - poll_tx_ts);
            Rb = (double)(final_rx_ts_32 - resp_tx_ts_32);
            Da = (double)(final_tx_ts - resp_rx_ts);
            Db = (double)(resp_tx_ts_32 - poll_rx_ts_32);
            tof_dtu = (int64)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));

            tof = tof_dtu * DWT_TIME_UNITS;
            twrDistance = tof * SPEED_OF_LIGHT;


            /* Display computed twrDistance on LCD. */
            printf("DIST: %3.2f m\n", twrDistance);


        }
    }
    else
    {
        /* Clear RX error/timeout events in the DW1000 status register. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        printf("final rx err\n");
        /* Reset RX to properly reinitialise LDE operation. */
        dwt_rxreset();
    }

}

void UwbMsgListener::initiateRanging()
{
  dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
    dwt_setpreambledetecttimeout(PRE_TIMEOUT);

  //  dwt_setpreambledetecttimeout(PRE_TIMEOUT);
    
    /* Write frame data to DW1000 and prepare transmission. See NOTE 8 below. */
    tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
    dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0); /* Zero offset in TX buffer. */
    dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1); /* Zero offset in TX buffer, ranging. */

    /* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
          * set by dwt_setrxaftertxdelay() has elapsed. */
    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

    /* We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout. See NOTE 9 below. */
    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
    { };
    //printf("statusReg h: %x mask: %x\n",status_reg,(SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR));

    /* Increment frame sequence number after transmission of the poll message (modulo 256). */
    frame_seq_nb++;

    if (status_reg & SYS_STATUS_RXFCG)
    {
        uint32 frame_len;

        /* Clear good RX frame event and TX frame sent in the DW1000 status register. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);

        /* A frame has been received, read it into the local buffer. */
        frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
        if (frame_len <= RX_BUF_LEN)
        {
            dwt_readrxdata(rx_buffer, frame_len, 0);
        }

        /* Check that the frame is the expected response from the companion "DS TWR responder" example.
              * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
        rx_buffer[ALL_MSG_SN_IDX] = 0;
        if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0)
        {
            uint32 final_tx_time;
            int ret;
            printf("response received\n");
            /* Retrieve poll transmission and response reception timestamp. */
            poll_tx_ts = get_tx_timestamp_u64();
            resp_rx_ts = get_rx_timestamp_u64();

            /* Compute final message transmission time. See NOTE 10 below. */
            final_tx_time = (resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
            dwt_setdelayedtrxtime(final_tx_time);

            /* Final TX timestamp is the transmission time we programmed plus the TX antenna delay. */
            final_tx_ts = (((uint64)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

            /* Write all timestamps in the final message. See NOTE 11 below. */
            final_msg_set_ts(&tx_final_msg[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);
            final_msg_set_ts(&tx_final_msg[FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts);
            final_msg_set_ts(&tx_final_msg[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);

            /* Write and send final message. See NOTE 8 below. */
            tx_final_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
            dwt_writetxdata(sizeof(tx_final_msg), tx_final_msg, 0); /* Zero offset in TX buffer. */
            dwt_writetxfctrl(sizeof(tx_final_msg), 0, 1); /* Zero offset in TX buffer, ranging. */
            ret = dwt_starttx(DWT_START_TX_DELAYED);

            /* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. See NOTE 12 below. */
            if (ret == DWT_SUCCESS)
            {

                /* Poll DW1000 until TX frame sent event set. See NOTE 9 below. */
                while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
                { };

                /* Clear TXFRS event. */
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

                /* Increment frame sequence number after transmission of the final message (modulo 256). */
                frame_seq_nb++;
            }
            else
            {
                printf("final tx timeout/err\n ");

            }
        }
    }
    else
    {
        /* Clear RX error/timeout events in the DW1000 status register. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        printf("response rx timeout/err \n");
        //printf("statusRegafter clear h: %x\n",status_reg);

        /* Reset RX to properly reinitialise LDE operation. */
        dwt_rxreset();
    }

    /* Execute a delay between ranging exchanges. */
    deca_sleep(RNG_DELAY_MS);
}

void UwbMsgListener::stopSending()
{
    isSending = 0;
}

void UwbMsgListener::waitUwbThreadsEnd(){
    isReceivingThreadRunning =0;
    if(pthread_join(receivingThreadUwb,NULL)){
        printf("error joining receiving thread");
    }else
    {
        printf("joining receiving thread successful");

    }
    if(pthread_join(sendingThreadUwb,NULL)){
        printf("error joining receiving thread");
    }else
    {
        printf("joining sending thread successful");

    }


}

void UwbMsgListener::startReceiving()
{
    int iret1 = pthread_create( &receivingThreadUwb, NULL,receivingLoop , 0);

    if(iret1)
    {
        fprintf(stderr,"Error creating receiving thread return code: %d\n",iret1);
        return;//exit(-1);
    }
    cout<< "started receiving thread\n";
}
void UwbMsgListener::startSending()
{
    int iret1 = pthread_create( &sendingThreadUwb, NULL,sendingLoop , 0);
    isSending = 1;
    if(iret1)
    {
        fprintf(stderr,"Error creating sending thread return code: %d\n",iret1);
        return;//exit(-1);
    }
    cout<< "started sending thread\n";
}


/*! ------------------------------------------------------------------------------------------------------------------
 * @fn final_msg_get_ts()
 *
 * @brief Read a given timestamp value from the final message. In the timestamp fields of the final message, the least
 *        significant byte is at the lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to read
 *         ts  timestamp value
 *
 * @return none
 */
static void final_msg_get_ts(const uint8 *ts_field, uint32 *ts)
{
    int i;
    *ts = 0;
    for (i = 0; i < FINAL_MSG_TS_LEN; i++)
    {
        *ts += ts_field[i] << (i * 8);
    }
}
/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_tx_timestamp_u64()
 *
 * @brief Get the TX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static uint64 get_tx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readtxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_rx_timestamp_u64()
 *
 * @brief Get the RX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static uint64 get_rx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readrxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn final_msg_set_ts()
 *
 * @brief Fill a given timestamp field in the final message with the given value. In the timestamp fields of the final
 *        message, the least significant byte is at the lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to fill
 *         ts  timestamp value
 *
 * @return none
 */
 static void final_msg_set_ts(uint8 *ts_field, uint64 ts)
{
    int i;
    for (i = 0; i < FINAL_MSG_TS_LEN; i++)
    {
        ts_field[i] = (uint8) ts;
        ts >>= 8;
    }
}

/*****************************************************************************************************************************************************
 * NOTES:
 *
 * 1. The sum of the values is the TX to RX antenna delay, experimentally determined by a calibration process. Here we use a hard coded typical value
 *    but, in a real application, each device should have its own antenna delay properly calibrated to get the best possible precision when performing
 *    range measurements.
 * 2. The messages here are similar to those used in the DecaRanging ARM application (shipped with EVK1000 kit). They comply with the IEEE
 *    802.15.4 standard MAC data frame encoding and they are following the ISO/IEC:24730-62:2013 standard. The messages used are:
 *     - a poll message sent by the initiator to trigger the ranging exchange.
 *     - a response message sent by the responder allowing the initiator to go on with the process
 *     - a final message sent by the initiator to complete the exchange and provide all information needed by the responder to compute the
 *       time-of-flight (distance) estimate.
 *    The first 10 bytes of those frame are common and are composed of the following fields:
 *     - byte 0/1: frame control (0x8841 to indicate a data frame using 16-bit addressing).
 *     - byte 2: sequence number, incremented for each new frame.
 *     - byte 3/4: PAN ID (0xDECA).
 *     - byte 5/6: destination address, see NOTE 3 below.
 *     - byte 7/8: source address, see NOTE 3 below.
 *     - byte 9: function code (specific values to indicate which message it is in the ranging process).
 *    The remaining bytes are specific to each message as follows:
 *    Poll message:
 *     - no more data
 *    Response message:
 *     - byte 10: activity code (0x02 to tell the initiator to go on with the ranging exchange).
 *     - byte 11/12: activity parameter, not used here for activity code 0x02.
 *    Final message:
 *     - byte 10 -> 13: poll message transmission timestamp.
 *     - byte 14 -> 17: response message reception timestamp.
 *     - byte 18 -> 21: final message transmission timestamp.
 *    All messages end with a 2-byte checksum automatically set by DW1000.
 * 3. Source and destination addresses are hard coded constants in this example to keep it simple but for a real product every device should have a
 *    unique ID. Here, 16-bit addressing is used to keep the messages as short as possible but, in an actual application, this should be done only
 *    after an exchange of specific messages used to define those short addresses for each device participating to the ranging exchange.
 * 4. Delays between frames have been chosen here to ensure proper synchronisation of transmission and reception of the frames between the initiator
 *    and the responder and to ensure a correct accuracy of the computed distance. The user is referred to DecaRanging ARM Source Code Guide for more
 *    details about the timings involved in the ranging process.
 * 5. This timeout is for complete reception of a frame, i.e. timeout duration must take into account the length of the expected frame. Here the value
 *    is arbitrary but chosen large enough to make sure that there is enough time to receive the complete response frame sent by the responder at the
 *    110k data rate used (around 3 ms).
 * 6. The preamble timeout allows the receiver to stop listening in situations where preamble is not starting (which might be because the responder is
 *    out of range or did not receive the message to respond to). This saves the power waste of listening for a message that is not coming. We
 *    recommend a minimum preamble timeout of 5 PACs for short range applications and a larger value (e.g. in the range of 50% to 80% of the preamble
 *    length) for more challenging longer range, NLOS or noisy environments.
 * 7. In a real application, for optimum performance within regulatory limits, it may be necessary to set TX pulse bandwidth and TX power, (using
 *    the dwt_configuretxrf API call) to per device calibrated values saved in the target system or the DW1000 OTP memory.
 * 8. dwt_writetxdata() takes the full size of the message as a parameter but only copies (size - 2) bytes as the check-sum at the end of the frame is
 *    automatically appended by the DW1000. This means that our variable could be two bytes shorter without losing any data (but the sizeof would not
 *    work anymore then as we would still have to indicate the full length of the frame to dwt_writetxdata()).
 * 9. We use polled mode of operation here to keep the example as simple as possible but all status events can be used to generate interrupts. Please
 *    refer to DW1000 User Manual for more details on "interrupts". It is also to be noted that STATUS register is 5 bytes long but, as the event we
 *    use are all in the first bytes of the register, we can use the simple dwt_read32bitreg() API call to access it instead of reading the whole 5
 *    bytes.
 * 10. As we want to send final TX timestamp in the final message, we have to compute it in advance instead of relying on the reading of DW1000
 *     register. Timestamps and delayed transmission time are both expressed in device time units so we just have to add the desired response delay to
 *     response RX timestamp to get final transmission time. The delayed transmission time resolution is 512 device time units which means that the
 *     lower 9 bits of the obtained value must be zeroed. This also allows to encode the 40-bit value in a 32-bit words by shifting the all-zero lower
 *     8 bits.
 * 11. In this operation, the high order byte of each 40-bit timestamps is discarded. This is acceptable as those time-stamps are not separated by
 *     more than 2**32 device time units (which is around 67 ms) which means that the calculation of the round-trip delays (needed in the
 *     time-of-flight computation) can be handled by a 32-bit subtraction.
 * 12. When running this example on the EVB1000 platform with the RESP_RX_TO_FINAL_TX_DLY response delay provided, the dwt_starttx() is always
 *     successful. However, in cases where the delay is too short (or something else interrupts the code flow), then the dwt_starttx() might be issued
 *     too late for the configured start time. The code below provides an example of how to handle this condition: In this case it abandons the
 *     ranging exchange to try another one after 1 second. If this error handling code was not here, a late dwt_starttx() would result in the code
 *     flow getting stuck waiting for a TX frame sent event that will never come. The companion "responder" example (ex_05b) should timeout from
 *     awaiting the "final" and proceed to have its receiver on ready to poll of the following exchange.
 * 13. The user is referred to DecaRanging ARM application (distributed with EVK1000 product) for additional practical example of usage, and to the
 *     DW1000 API Guide for more details on the DW1000 driver functions.
 ****************************************************************************************************************************************************/




