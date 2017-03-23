#include "serial_task.h"
#include "serial.h"
#include "main_input_timer.h"
#include "pulser.h"
#include "utils.h"

#include "stm32f4xx_gpio.h"

#include "CoOS.h"

#include <inttypes.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>

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

static serialCli_st serialCli[ARRAY_SIZE(serial_ports)];

serial_port_st *debug_port;
static struct cli_context_st
{
    OS_FlagID periodic_tasks_timer_flag;
    uint32_t periodic_tasks_timer_bit;

    OS_FlagID cli_data_ready_flag;
    uint32_t cli_data_ready_bit;

    uint32_t all_flags;

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

static void periodic_timer_cb( void )
{
    isr_SetFlag(cli_context.periodic_tasks_timer_flag);
}

static void new_uart_data_cb( void *pv )
{
	UNUSED(pv);

    CoEnterISR();

    isr_SetFlag(cli_context.cli_data_ready_flag);

	CoExitISR();
}

static void handle_new_serial_data(void)
{
	unsigned int uart_index;
    static size_t output_index;

	for (uart_index = 0; uart_index < ARRAY_SIZE(serialCli); uart_index++ )
	{
		if ( serialCli[uart_index].cli_uart != NULL )
		{
			while ( serialCli[uart_index].cli_uart->methods->rxReady( serialCli[uart_index].cli_uart->serialCtx ) )
			{
				int ch;

				ch = serialCli[uart_index].cli_uart->methods->readChar( serialCli[uart_index].cli_uart->serialCtx );

                if (ch >= 0)
                {
                    //uartPutChar(serialCli[uart_index].cli_uart, ch);
                    if (ch == ' ')
                    {
                        float rpm_get(void);
                        float const rpm = rpm_get();
                        uint32_t rpm_32 = lrintf(rpm);

                        //printf("rpm: %f\r\n", rpm);
                        printf("rpm_32: %"PRIu32"\r\n", rpm_32);
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
                        void print_injector_debug(size_t const index);

                        print_injector_debug(output_index);
                    }
                    if (ch == 'g')
                    {
                        void print_ignition_debug(size_t index);

                        print_ignition_debug(output_index);
                    }
                    if (ch == 'p')
                    {
                        void print_pulser_debug(size_t const index);

                        print_pulser_debug(output_index);
                    }
                    if (ch == '0' || ch == '1' || ch == '2' || ch == '3')
                    {
                        output_index = ch - '0';
                    }
                }
			}
		}
	}
}

static void do_periodic_serial_tasks(void)
{
}

static void cli_task(void * pv)
{
    OS_TCID debugTimerID;

    UNUSED(pv);

    debugTimerID = CoCreateTmr(TMR_TYPE_PERIODIC, CFG_SYSTICK_FREQ, CFG_SYSTICK_FREQ, periodic_timer_cb);
    CoStartTmr(debugTimerID);

    while (1)
    {
        StatusType err;
        U32 readyFlags;

        readyFlags = CoWaitForMultipleFlags(cli_context.all_flags, OPT_WAIT_ANY, 0, &err);
        if ((readyFlags & cli_context.cli_data_ready_bit) != 0)
        {
            handle_new_serial_data();
        }

        if ((readyFlags & cli_context.periodic_tasks_timer_bit) != 0)
        {
            do_periodic_serial_tasks();
        }
    }
}

bool set_debug_port( int port )
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

void serial_task_init(void)
{
	unsigned int cli_index;

    cli_context.periodic_tasks_timer_flag = CoCreateFlag(Co_TRUE, Co_FALSE);
    cli_context.periodic_tasks_timer_bit = 1 << cli_context.periodic_tasks_timer_flag;
    cli_context.cli_data_ready_flag = CoCreateFlag(Co_TRUE, Co_FALSE);
    cli_context.cli_data_ready_bit = 1 << cli_context.cli_data_ready_flag;

    cli_context.all_flags = cli_context.periodic_tasks_timer_bit | cli_context.cli_data_ready_bit;
    
	for ( cli_index = 0 ; cli_index < ARRAY_SIZE(serial_ports); cli_index++ )
	{
        serialCli[cli_index].cli_uart = serialOpen(serial_ports[cli_index], 115200, uart_mode_rx | uart_mode_tx, new_uart_data_cb);
	}
    set_debug_port(0); /* Temp debug assign the debug port right now until we have a CLI command that allows it to be turned on/off. */

    serialTaskID = CoCreateTask(cli_task, Co_NULL, SERIAL_TASK_PRIORITY, &cli_context.cli_task_stack[CLI_TASK_STACK_SIZE - 1], CLI_TASK_STACK_SIZE);
}
