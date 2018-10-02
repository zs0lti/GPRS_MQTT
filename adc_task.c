#include <stdbool.h>
#include <stdint.h>

#include <inc/hw_memmap.h>
#include <inc/hw_ints.h>

#include <driverlib/gpio.h>
#include <driverlib/sysctl.h>
#include <driverlib/pin_map.h>
#include <driverlib/adc.h>
#include <driverlib/uart.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Mailbox.h>

#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>

#include "uartstdio.h"
#include "sim800.h"

void adc_task(UArg arg0, UArg arg1)
{

    mqtt_msg mboxmsg;
    uint32_t val_1 = 0;         // raw value 1
    float value_1 = 0;          // Value 1
    float value_tmp_1 = 0;      // Val_1 temp
    float tol_1 = 5;            // Tolerance 1
    float hyst_1 = 0.6;         // Hysterisis 1

    char buff[64];

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOL))
    {
    }

    GPIOPinTypeGPIOInput(GPIO_PORTL_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);

    while(1 > 0){

        // Enable the ADC0 module.
        SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

        // Wait for the ADC0 module to be ready.
        while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0)) {
        }

        // Enable the first sample sequencer to capture the value of channel 0 when
        // the processor trigger occurs.
        ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);
        ADCSequenceStepConfigure(ADC0_BASE, 0, 0,
        ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH0);
        ADCSequenceEnable(ADC0_BASE, 0);

        // Trigger the sample sequence.
        ADCProcessorTrigger(ADC0_BASE, 0);

        // Wait until the sample sequence has completed.
        while(!ADCIntStatus(ADC0_BASE, 0, 0)) {
        }

        // Read the value from the ADC.
        ADCSequenceDataGet(ADC0_BASE, 0, &val_1);
        //UARTprintf("IP Task - ADC 0 raw value: %d \n", val_1);

        // 4096 -> 100
        value_1 = val_1 / 41;

        //UP
        if(value_1 >= value_tmp_1 + tol_1 * hyst_1 & value_tmp_1 < 100) {
            value_tmp_1 = value_tmp_1 + tol_1;
            mboxmsg.value_1 = value_tmp_1;
            mboxmsg.typ = MSG_TYPE_ANA;
            Mailbox_post(mboxHandleIP,&mboxmsg,BIOS_WAIT_FOREVER);
        }

        //DOWN
        if(value_1 <= value_tmp_1 - tol_1 + tol_1 * (1 - hyst_1)) {
            value_tmp_1 = value_tmp_1 - tol_1;
            mboxmsg.value_1 = value_tmp_1;
            mboxmsg.typ = MSG_TYPE_ANA;
            Mailbox_post(mboxHandleIP,&mboxmsg,BIOS_WAIT_FOREVER);
        }

        // DECREASING to ZERO
        if(value_1 < 1 & value_1 < value_tmp_1) {
            value_tmp_1 = value_tmp_1 - tol_1;
            mboxmsg.value_1 = value_tmp_1;
            mboxmsg.typ = MSG_TYPE_ANA;
            Mailbox_post(mboxHandleIP,&mboxmsg,BIOS_WAIT_FOREVER);
        }

        Task_sleep(100);
    }
}

int setup_adc_task(void)
{
    Task_Params adc_task_params;
    Task_Handle adc_task_handle;
    Error_Block eb;

    Error_init(&eb);
    Task_Params_init(&adc_task_params);
    adc_task_params.stackSize = 2048;
    adc_task_params.priority = 15;
    adc_task_params.arg0 = NULL;
    adc_task_params.arg1 = NULL;
    adc_task_handle = Task_create((Task_FuncPtr)adc_task, &adc_task_params, &eb);

    if (adc_task_handle == NULL) {
        System_abort("Failed to create adc task");
    }

    return 0;
}
