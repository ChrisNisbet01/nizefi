#include <inttypes.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <coocox.h>
#include <serial.h>

#include <stm32f4xx_gpio.h>

#define CLI_TASK_STACK_SIZE 0x200
#define SERIAL_TASK_PRIORITY 4

typedef struct serialCli_st
{
	serial_port_st *cli_uart;
} serialCli_st;

static OS_STK cli_task_stack[CLI_TASK_STACK_SIZE];
static OS_TID serialTaskID;

static const serial_port_t serial_ports[] =
{
#if defined(STM32F30X)
	SERIAL_UART_2,
	SERIAL_USB
#elif defined(STM32F4XX)
    SERIAL_UART_2,
#elif defined(STM32F10X)
	SERIAL_UART_1
#endif
};

#define ARRAY_SIZE(a) (sizeof(a)/sizeof((a)[0]))
#define UNUSED(x) ((void)(x))

static serialCli_st serialCli[ARRAY_SIZE(serial_ports)];

serial_port_st *debug_port;
static OS_FlagID cliUartFlag;
static OS_FlagID periodicTasksTimerFlag;

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

static void debugTimer( void )
{
	isr_SetFlag( periodicTasksTimerFlag );
}

static void newUartData( void *pv )
{
	UNUSED(pv);

	CoEnterISR();

	isr_SetFlag(cliUartFlag);

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
				uint8_t ch;

				ch = serialCli[uart_index].cli_uart->methods->readChar( serialCli[uart_index].cli_uart->serialCtx );

                /* Just echo the char back for now. */
                uartPutChar(serialCli[uart_index].cli_uart, ch);
			}
		}
	}
}

static void doDebugOutput( void )
{
}

static void doPeriodicTasks( void )
{
	doDebugOutput();
}

static void cli_task( void *pv )
{
	OS_TCID debugTimerID;

	UNUSED(pv);

	debugTimerID = CoCreateTmr( TMR_TYPE_PERIODIC, CFG_SYSTICK_FREQ, CFG_SYSTICK_FREQ, debugTimer );
	CoStartTmr( debugTimerID );

	while (1)
	{
		StatusType err;
		U32 readyFlags;

		readyFlags = CoWaitForMultipleFlags( (1 << periodicTasksTimerFlag) | (1 << cliUartFlag), OPT_WAIT_ANY, 0, &err );
        if ((readyFlags & (1 << cliUartFlag)))
        {
	 		handleNewSerialData();
        }

        if ((readyFlags & (1 << periodicTasksTimerFlag)))
        {
	 		doPeriodicTasks();
        }
	}
}

bool setDebugPort( int port )
{
	bool debugPortAssigned = true;

	if ( port < 0 )
		debug_port = NULL;
	else if ( (unsigned)port < ARRAY_SIZE(serialCli) && serialCli[port].cli_uart != NULL )
		debug_port = serialCli[port].cli_uart;
	else
		debugPortAssigned = false;

	return debugPortAssigned;
}

void initSerialTask( void )
{
	unsigned int cli_index;

	cliUartFlag = CoCreateFlag( Co_TRUE, Co_FALSE );
	periodicTasksTimerFlag = CoCreateFlag( Co_TRUE, Co_FALSE );

	for ( cli_index = 0 ; cli_index < ARRAY_SIZE(serial_ports); cli_index++ )
	{
		serialCli[cli_index].cli_uart = serialOpen( serial_ports[cli_index], 115200, uart_mode_rx | uart_mode_tx, newUartData );
	}

    serialTaskID = CoCreateTask(cli_task, Co_NULL, SERIAL_TASK_PRIORITY, &cli_task_stack[CLI_TASK_STACK_SIZE - 1], CLI_TASK_STACK_SIZE);
}
