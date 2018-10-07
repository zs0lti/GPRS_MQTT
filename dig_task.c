#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>

#include <inc/hw_memmap.h>
#include <inc/hw_ints.h>
#include <driverlib/gpio.h>
#include <driverlib/sysctl.h>
#include <driverlib/pin_map.h>
#include <driverlib/uart.h>
#include <driverlib/interrupt.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/hal/Hwi.h>
#include <ti/sysbios/knl/Mailbox.h>

#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>

#include "uartstdio.h"
#include "sim800.h"

#define DIGITAL_TRANSITION_FIFO_SIZE 64
int digital_state[DIGITAL_TRANSITION_FIFO_SIZE];
static uint32_t ribuf_rh = 0;
static uint32_t ribuf_wh = 0;

/*======================= peek dig state =======================*/
int peek_dig_state()
{
    if (ribuf_rh == ribuf_wh) {
        return -1;
    }

    return digital_state[ribuf_rh];
}

/*======================= digital get state =======================*/
int dig_get_state(void)
{
    int state;
    if (ribuf_rh == ribuf_wh) {
        return -1;
    }

    state = digital_state[ribuf_rh++];
    ribuf_rh %= DIGITAL_TRANSITION_FIFO_SIZE;

    return state;
}

/*======================= digital GPIO-L ISR =======================*/
volatile int current_dig_state = 0;

void dig_gpiol_isr(UArg arg0)
{
    uint32_t int_status;
    uint32_t gpio_status;
    int new_dig_state = current_dig_state;

    int_status = GPIOIntStatus(GPIO_PORTL_BASE, true);
    gpio_status = GPIOPinRead(GPIO_PORTL_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    if (int_status & GPIO_PIN_0) {                  //detecting interrupt on pin0
        if (gpio_status & GPIO_PIN_0)
            new_dig_state |= (1 << 0);              //set bit 0 to 1
        else
            new_dig_state &= ~(1 << 0);             //set bit 0 to 0
    }
    if (int_status & GPIO_PIN_1) {                  //detecting interrupt on pin1
        if (gpio_status & GPIO_PIN_1)
            new_dig_state |= (1 << 1);              //set bit 1 to 1
        else
            new_dig_state &= ~(1 << 1);             //set bit 1 to 0
    }

    digital_state[ribuf_wh++] = new_dig_state;
    ribuf_wh %= DIGITAL_TRANSITION_FIFO_SIZE;
    current_dig_state = new_dig_state;

    GPIOIntClear(GPIO_PORTL_BASE, int_status);
}

/*======================= digital init =======================*/
// digital inputs are on PL0 and PL1
int dig_init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
    GPIOPinTypeGPIOInput(GPIO_PORTL_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    //GPIOPadConfigSet(GPIO_PORTL_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    GPIOPadConfigSet(GPIO_PORTL_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);

    {
        Hwi_Params hwiParams;
        Hwi_Handle hwiHandle;
        Error_Block eb;

        Error_init(&eb);
        Hwi_Params_init(&hwiParams);
        hwiParams.arg = 0;
        hwiParams.enableInt = false;
        hwiParams.maskSetting = ti_sysbios_interfaces_IHwi_MaskingOption_SELF;
        hwiParams.priority = 1;

        hwiHandle = Hwi_create(INT_GPIOL, dig_gpiol_isr, &hwiParams, &eb);
        if (hwiHandle == NULL) {
            UARTprintf("ERROR: Failed to create GPIOL interrupt\n");
            System_abort("ERROR: Failed to create GPIOL interrupt\n");
        }

        GPIOIntTypeSet(GPIO_PORTL_BASE, GPIO_INT_PIN_0 | GPIO_INT_PIN_1, GPIO_BOTH_EDGES);

        Hwi_enableInterrupt(INT_GPIOL);
        GPIOIntEnable(GPIO_PORTL_BASE, GPIO_INT_PIN_1 | GPIO_INT_PIN_0);
    }

    return 0;
}

/*======================= digital task =======================*/
void dig_task(UArg arg0, UArg arg1)
{
    mqtt_msg mboxmsg;
    dig_init();

    while (1 > 0) {

        if (peek_dig_state() != -1) {
            int state;

            while ((state = dig_get_state()) != -1) {
                    mboxmsg.value_1 = state;
                    mboxmsg.typ = MSG_TYPE_DIG;
                    Mailbox_post(mboxHandleIP,&mboxmsg,BIOS_WAIT_FOREVER);
            }
        }
        Task_sleep(10);
    }
}

/*======================= setup digital task =======================*/
int setup_dig_task(void)
{
    Task_Params dig_task_params;
    Task_Handle dig_task_handle;
    Error_Block eb;

    Error_init(&eb);
    Task_Params_init(&dig_task_params);
    dig_task_params.stackSize = 2048;
    dig_task_params.priority = 13;
    dig_task_params.arg0 = NULL;
    dig_task_params.arg1 = NULL;
    dig_task_handle = Task_create((Task_FuncPtr)dig_task, &dig_task_params, &eb);

    if (dig_task_handle == NULL) {
        System_abort("Failed to create sim800 task");
    }

    return 0;
}
