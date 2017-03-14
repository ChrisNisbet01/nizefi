#include <inttypes.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <coocox.h>
#include <serial.h>
#include "hi_res_timer.h"
#include "pulser.h"

#include <stm32f4xx_gpio.h>

#define CLI_TASK_STACK_SIZE 1024
#define SERIAL_TASK_PRIORITY 4

typedef struct serialCli_st
{
	serial_port_st *cli_uart;
} serialCli_st;

static OS_TID serialTaskID;

static const serial_port_t serial_ports[] =
{
    SERIAL_UART_1,
};

#define ARRAY_SIZE(a) (sizeof(a)/sizeof((a)[0]))
#define UNUSED(x) ((void)(x))

static serialCli_st serialCli[ARRAY_SIZE(serial_ports)];

serial_port_st *debug_port;
static struct cli_context_st
{
    OS_FlagID periodicTasksTimerFlag;
    OS_FlagID cliUartFlag;
    OS_STK cli_task_stack[CLI_TASK_STACK_SIZE];
} cli_context;

int uartPutChar( void * port, int ch )
{
	serial_port_st * serialPort = port;

    int result = serialPort->methods->writeCharBlockingWithTimeout(serialPort->serialCtx, ch, 10);

	return result;
}

void debug_put_char(char ch)
{
    if (debug_port != NULL)
    {
        uartPutChar(debug_port, ch);
    }
}

int debug_put_block(void * data, size_t len)
{
    size_t x;
    char * pch;

    for (x = 0, pch = data; x < len; x++, pch++)
    {
        debug_put_char(*pch);
    }
    return len;
}

uint32_t new_time;

static void debugTimer( void )
{
    new_time = SysTick->VAL;
    isr_SetFlag(cli_context.periodicTasksTimerFlag);
}

static void newUartData( void *pv )
{
	UNUSED(pv);

    CoEnterISR();

    isr_SetFlag(cli_context.cliUartFlag);

	CoExitISR();
}

static void handleNewSerialData( void )
{
	unsigned int uart_index;

	for (uart_index = 0; uart_index < ARRAY_SIZE(serialCli); uart_index++ )
	{
		if ( serialCli[uart_index].cli_uart != NULL )
		{
			while ( serialCli[uart_index].cli_uart->methods->rxReady( serialCli[uart_index].cli_uart->serialCtx ) )
			{
				int ch;

				ch = serialCli[uart_index].cli_uart->methods->readChar( serialCli[uart_index].cli_uart->serialCtx );

                /* Just echo the char back for now. */
                if (ch >= 0)
                {
                    //uartPutChar(serialCli[uart_index].cli_uart, ch);
                    if (ch == 'r')
                    {
                        reset_pulse_details();
                    }
                    print_pulse_details();
                    if (ch == ' ')
                    {
                        float rpm_get(void);
                        float const rpm = rpm_get();
                        uint32_t rpm_32 = lrintf(rpm);

                        //printf("rpm: %f\r\n", rpm);
                        printf("rpm_32: %"PRIu32"\r\n", rpm_32);
                    }
                    if (ch == 'm')
                    {
                        printf("min length %d\r\n", min_queue_length_get());
                    }
                    if (ch == 'c')
                    {
                        float crank_angle_get(void);
                        float const crank_angle = crank_angle_get();
                        uint32_t crank_angle_32 = lrintf(crank_angle);

                        printf("crank: %f %"PRIu32"\r\n", crank_angle, crank_angle_32);
                    }
                    if (ch == 'e')
                    {
                        float engine_cycle_angle_get(void);
                        float const engine_cycle_angle = engine_cycle_angle_get();
                        uint32_t engine_cycle_angle_32 = lrintf(engine_cycle_angle);

                        printf("engine: %"PRIu32" %f\r\n", engine_cycle_angle_32, engine_cycle_angle);
                    }
                    if (ch == 'i')
                    {
                        void print_injector_debug(void);

                        print_injector_debug();
                    }
                }
			}
		}
	}
}

void show_sysclock_info(uint32_t val)
{
    RCC_ClocksTypeDef clocks;

    RCC_GetClocksFreq(&clocks);
    printf("clock rate count %"PRIu32"\r\n", val);
    printf("new_time         %"PRIu32"\r\n", new_time);
    printf("hi_res_counter_val %"PRIu32"\r\n", hi_res_counter_val());
    printf("pclk1 %"PRIu32"\r\n", clocks.PCLK1_Frequency);
}

static void doDebugOutput(uint32_t val)
{

    //show_sysclock_info(val);
    //print_pulse_details();

}

static void doPeriodicSerialTasks(uint32_t val)
{
	doDebugOutput(val);
}

static void cli_task(void * pv)
{
    OS_TCID debugTimerID;

    UNUSED(pv);

    debugTimerID = CoCreateTmr(TMR_TYPE_PERIODIC, CFG_SYSTICK_FREQ, CFG_SYSTICK_FREQ, debugTimer);
    CoStartTmr(debugTimerID);

    while (1)
    {
        StatusType err;
        U32 readyFlags = (1 << cli_context.cliUartFlag);

        readyFlags = CoWaitForMultipleFlags((1 << cli_context.periodicTasksTimerFlag) | (1 << cli_context.cliUartFlag), OPT_WAIT_ANY, 0, &err);
        if ((readyFlags & (1 << cli_context.cliUartFlag)))
        {
            handleNewSerialData();
        }

        if ((readyFlags & (1 << cli_context.periodicTasksTimerFlag)))
        {
            doPeriodicSerialTasks(SysTick->VAL);
        }
    }
}

bool setDebugPort( int port )
{
	bool debugPortAssigned = true;

    if (port < 0)

    {
		debug_port = NULL;
    }
    else if ((unsigned)port < ARRAY_SIZE(serialCli) && serialCli[port].cli_uart != NULL)
    {
		debug_port = serialCli[port].cli_uart;
    }
    else
    {
		debugPortAssigned = false;
    }

	return debugPortAssigned;
}

void initSerialTask( void )
{
	unsigned int cli_index;

    cli_context.periodicTasksTimerFlag = CoCreateFlag(Co_TRUE, Co_FALSE);
    cli_context.cliUartFlag = CoCreateFlag(Co_TRUE, Co_FALSE);

	for ( cli_index = 0 ; cli_index < ARRAY_SIZE(serial_ports); cli_index++ )
	{
		serialCli[cli_index].cli_uart = serialOpen( serial_ports[cli_index], 115200, uart_mode_rx | uart_mode_tx, newUartData );
	}
    setDebugPort(0); /* Temp debug assign the debug port right now until we have a CLI command that allows it to be turned on/off. */

    serialTaskID = CoCreateTask(cli_task, Co_NULL, SERIAL_TASK_PRIORITY, &cli_context.cli_task_stack[CLI_TASK_STACK_SIZE - 1], CLI_TASK_STACK_SIZE);
}
