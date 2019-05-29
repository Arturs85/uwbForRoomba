#include <string>
#include <stdio.h>
#include "uwbmsgsender.hpp"

#include <stdlib.h>
#include <unistd.h>
#include <sys/select.h>
#include <termios.h>


/* Example application name and version to display on LCD screen. */
#define APP_NAME "UWB Sender"

void UwbMsgListener::initialize()
{

   

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
