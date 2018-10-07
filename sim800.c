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
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/hal/Hwi.h>
#include <ti/sysbios/knl/Mailbox.h>

#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>

#include "sim800.h"
#include "uartstdio.h"
#include "mqtt_pal.h"
#include "mqtt.h"

//#define SIM800_DEBUG		/* Enable some debug output */
//#define SIM800_FAST_SERIAL	/* Switch SIM800 baudrate between 115200 and 9600 */
#define SIM800_QUICKSEND	/* Enable quick send feature */

#define SIM800_APN	"webaut"

#define MQTT_HOST	"178.62.209.197"
#define MQTT_PORT	1883
#define MQTT_USER	"zsolt"
#define MQTT_PASS	"8AiSNeZxaRhtQN6n"

extern uint32_t sysclk;

Mailbox_Handle mboxHandleIP;

#define RESPONSE_INPUT_RINGBUFFER_SIZE 2048
char response_input_ringbuffer[RESPONSE_INPUT_RINGBUFFER_SIZE];
static uint32_t ribuf_rh = 0;
static uint32_t ribuf_wh = 0;

uint32_t gl_timeout = 0;
uint32_t gl_timeout_start = 0;

/*======================= starts timeout =======================*/
void start_timeout(uint32_t new_timeout)
{
	gl_timeout = new_timeout;
	gl_timeout_start = Clock_getTicks();
}

/*======================= stopps timeout =======================*/
void stop_timeout(void)
{
	gl_timeout = 0;
	gl_timeout_start = 0;
}

/*======================= UART6 =======================*/
bool timed_out(void)
{
	if (gl_timeout == 0 || gl_timeout_start == 0)
		return false;

	if (Clock_getTicks() > (gl_timeout_start + gl_timeout))
		return true;

	return false;
}

void sim800_uart6_flush(void)
{
	ribuf_rh = 0;
	ribuf_wh = 0;
}

/*======================= UART6 get character =======================*/
int sim800_uart6_getc(void)
{
	int ch;
	if (ribuf_rh == ribuf_wh) {                             //when readhead is equal to writehead the buffer is empty
		return -1;
	}

	ch = response_input_ringbuffer[ribuf_rh++];             //reading from the response buffer, increasing readhead
	ribuf_rh %= RESPONSE_INPUT_RINGBUFFER_SIZE;             //resets read index to 0 when reaching ringbuffer size

	return ch;
}

/*======================= UART6 get character blocking =======================*/
int sim800_uart6_getc_blocking(void)
{
	int ch;

	do {
		ch = sim800_uart6_getc();

		if (timed_out())
			return -1;
	} while (ch == -1);

	return ch;
}

/*======================= UART6 isr =======================*/
void sim800_uart6_isr(UArg arg0)
{
	uint32_t int_status;

	int_status = UARTIntStatus(UART6_BASE, true);
	UARTIntClear(UART6_BASE, int_status);

	while (UARTCharsAvail(UART6_BASE)) {
		int32_t ch = UARTCharGetNonBlocking(UART6_BASE);

		if (ch == -1)
			continue;

		response_input_ringbuffer[ribuf_wh++] = ch & 0xFF;      //writing into the response buffer and masking the int32 to the size of a character
		ribuf_wh %= RESPONSE_INPUT_RINGBUFFER_SIZE;             //resets write index to 0 when reaching ringbuffer size

#ifdef SIM800_DEBUG
#ifndef SIM800_FAST_SERIAL
		if (isprint(ch))                                        //check if ASCII character
			UARTprintf("[ %c ]\n", ch);                         //print ASCII character
		else if (ch == '\n')
			UARTprintf("[ \\n ]\n");
		else if (ch == '\r')
			UARTprintf("[ \\r ]\n");
		else
			UARTprintf("[ \\x%02x ]\n", ch);                    //else print HEX
#endif
#endif
	}
}

/*======================= Send AT command =======================*/
int sim800_send_at_command(const char *command)
{
	int i;

#ifdef SIM800_DEBUG
	UARTprintf("\nSending AT command: %s\n", command);
#endif

	/* Send out the command until empty and terminate it properly with \r */
	for (i = 0; command[i] != '\0'; i++) {
		UARTCharPut(UART6_BASE, command[i]);
	}
	UARTCharPut(UART6_BASE, '\r');

	return 0;
}

/*======================= Expect char =======================*/
/* reads characters until the expected char arrives or the timeout is reached */
int sim800_expect_char(int ch)
{
	int ech;

	do {
		ech = sim800_uart6_getc_blocking();

		if (timed_out())
			return -1;
	} while (ech != ch);

	return 0;
}

/*======================= Get response into the buffer =======================*/
int sim800_get_response(char *buf, size_t s, uint32_t timeout)
{
	size_t i = 0;
	int ret;
	int ch;

	if (timeout != 0) {
		start_timeout(timeout);
	}

	ret = sim800_expect_char('\r');
	if (ret < 0) {
		stop_timeout();
		return ret;
	}
	ret = sim800_expect_char('\n');
	if (ret < 0) {
		stop_timeout();
		return ret;
	}

	do {
		ch = sim800_uart6_getc();

		if (buf != NULL && ch != -1) {
			if (i >= s)
				return -1;

			if (ch != '\r')
				buf[i++] = ch;
			else
				buf[i++] = '\0';
		}
	} while (ch == -1 || ch != '\r');

	ret = sim800_expect_char('\n');
	if (ret < 0) {
		stop_timeout();
		return ret;
	}

	return 0;
}

/*======================= Handle URC =======================*/
void handle_urc(char *rsp)
{
#ifdef SIM800_DEBUG
	UARTprintf("Got new URC: `%s`\n", rsp);
#endif
}

/*======================= Expect normal response =======================*/
// Returns -1 on timeout and 1 on ERROR response, 0 if response matches
int sim800_expect_normal_response(const char *rsp, int timeout)
{
	char rsp_buf[256];
	int ret;

	while (1 > 0) {
		ret = sim800_get_response(rsp_buf, sizeof(rsp_buf), timeout);
		if (ret < 0) {
			return ret;
		}

		if (strncmp(rsp_buf, rsp, sizeof(rsp_buf)) == 0) {
			return 0;
		} else if (strncmp(rsp_buf, "ERROR", sizeof(rsp_buf)) == 0) {
			UARTprintf("Got ERROR response\n");
			return 1;
		} else {
			handle_urc(rsp_buf);
		}
	}

	return 0;                        // We should never arrive here, returns should be handled in the upper conditions
}

/*======================= Expect CIFSR response =======================*/
int sim800_expect_cifsr_response(char *ipbuf, size_t len, uint32_t timeout)
{
	char rsp_buf[256];
	int ret;

	while (1 > 0) {
		ret = sim800_get_response(rsp_buf, sizeof(rsp_buf), timeout);
		if (ret < 0) {
			return ret;
		}

		if (rsp_buf[0] != '+') {
			strncpy(ipbuf, rsp_buf, len);
			return 0;
		} else {
			handle_urc(rsp_buf);
		}
	}
}

/*======================= Expect CIPSTART response =======================*/
int sim800_expect_cipstart_response(uint32_t timeout)
{
	int ret;
	char rsp_buf[256];

	ret = sim800_expect_normal_response("OK", 2000);
	if (ret != 0) {
		return ret;
	}

	while (1 > 0) {
		ret = sim800_get_response(rsp_buf, sizeof(rsp_buf), timeout);
		if (ret < 0) {
			return ret;
		}

		if (strncmp(rsp_buf, "CONNECT OK", sizeof(rsp_buf)) == 0) {
			return 0;
		} else if (strncmp(rsp_buf, "CONNECT FAIL", sizeof(rsp_buf)) == 0) {
			return 1;
		} else {
			handle_urc(rsp_buf);
		}
	}
}

/*======================= Send raw data =======================*/
int sim800_send_raw_data(uint8_t *buffer, uint16_t len)
{
	uint16_t i;

	for(i = 0; i < len; i++) {
		UARTCharPut(UART6_BASE, buffer[i]);
	}

	return 0;
}

/*======================= Reset SIM800 =======================*/
// solution for missing status pin in HW - state detection / case of reset
void sim800_reset(void)
{
	int ret;
	GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_4, GPIO_PIN_4);
	Task_sleep(1600);
	GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_4, 0);
	Task_sleep(500);

	ret = sim800_expect_normal_response("NORMAL POWER DOWN", 1000);
	if (ret == 0) {
		UARTprintf("SIM800 powered down, powering back up!\n");
		GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_4, GPIO_PIN_4);
		Task_sleep(1600);
		GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_4, 0);
		Task_sleep(500);
	}

	UARTprintf("SIM800 powering up!\n");
	sim800_send_at_command("AT");                   //send twice for automatic baudrate detection
	Task_sleep(100);
	sim800_send_at_command("AT");
	UARTprintf("Waiting for garbage on power on\n");
	Task_sleep(5000);
	sim800_send_at_command("AT");

	Task_sleep(1000);
	sim800_uart6_flush();
}

/*======================= SIM800 init =======================*/
/* Based on: https://www.exploreembedded.com/wiki/Setting_up_GPRS_with_SIM800L */
int sim800_init(void)
{
	int ret;

	UARTprintf("Called sim800_init()\n");

	/* Connections: PWRKEY => PN4 | Rx/Tx  => PP0/PP1 (UART6) */
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
	GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_4);
	GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_4, 0);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART6);

    GPIOPinConfigure(GPIO_PP0_U6RX);
    GPIOPinConfigure(GPIO_PP1_U6TX);
    GPIOPinTypeUART(GPIO_PORTP_BASE, GPIO_PIN_0 | GPIO_PIN_1);

#ifdef SIM800_FAST_SERIAL
    UARTConfigSetExpClk(UART6_BASE, sysclk, 115200, UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);
#else
    UARTConfigSetExpClk(UART6_BASE, sysclk, 9600, UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);
#endif
    UARTFIFODisable(UART6_BASE);            //disable hw buffer for simplicity reasons of interrupt handler

    {
    	Hwi_Params hwiParams;                   //configure Hwi for UART6
    	Hwi_Handle hwiHandle;
    	Error_Block eb;

		Error_init(&eb);
		Hwi_Params_init(&hwiParams);
		hwiParams.arg = 0;
		hwiParams.enableInt = false;
		hwiParams.maskSetting = ti_sysbios_interfaces_IHwi_MaskingOption_SELF;

		hwiHandle = Hwi_create(INT_UART6, sim800_uart6_isr, &hwiParams, &eb);
		if (hwiHandle == NULL) {
			UARTprintf("ERROR: Failed to create UART6 interrupt\n");
			System_abort("ERROR: Failed to create UART6 interrupt\n");
		}
    }
	Hwi_enableInterrupt(INT_UART6);                //enable Hwi for UART6
    UARTIntEnable(UART6_BASE, UART_INT_RX);

    sim800_reset();

    /* Disable echo mode */
	sim800_send_at_command("ATE0");     //send command twice because of automatic baudrate detect
	Task_sleep(100);
	sim800_send_at_command("ATE0");
	Task_sleep(100);
	sim800_uart6_flush();

	sim800_send_at_command("AT");
	ret = sim800_expect_normal_response("OK", 1000);
	if (ret != 0) {
		return ret;
	}
	UARTprintf("SIM800 ready\n");

	/* Disable flow control */
	sim800_send_at_command("AT+IFC=0,0");
	ret = sim800_expect_normal_response("OK", 1000);
	if (ret != 0) {
		return ret;
	}

	/* Disable call URC */
	sim800_send_at_command("AT+CIURC=0");
	ret = sim800_expect_normal_response("OK", 1000);
	if (ret != 0) {
		return ret;
	}

	/* Enable manual data receive */
	sim800_send_at_command("AT+CIPRXGET=1");
	ret = sim800_expect_normal_response("OK", 1000);
	if (ret != 0) {
		return ret;
	}

	/* Enable quick send*/
#ifdef SIM800_QUICKSEND
	sim800_send_at_command("AT+CIPQSEND=1");
	ret = sim800_expect_normal_response("OK", 1000);
	if (ret != 0) {
		return ret;
	}
#endif

	sim800_send_at_command("AT+CFUN=1");
	ret = sim800_expect_normal_response("OK", 1000);
	if (ret != 0) {
		return ret;
	}

	sim800_send_at_command("AT+CPIN?");
	ret = sim800_expect_normal_response("OK", 1000);
	if (ret != 0) {
		return ret;
	}

	sim800_send_at_command("AT+CSTT=\""SIM800_APN"\",\"\",\"\"");
	ret = sim800_expect_normal_response("OK", 1000);
	if (ret != 0) {
		return ret;
	}

	/*
	 * Sleep for 5 seconds to make sure we are connected to the network,
	 * if we are too fast here we will get a `+PDP: DEACT` URC.
	 */
	UARTprintf("Waiting 5 seconds to make sure we are connected to the GPRS network\n");
	Task_sleep(5000);
	sim800_send_at_command("AT+CIICR");
	ret = sim800_expect_normal_response("OK", 2000);
	if (ret != 0) {
		return ret;
	}

	char ip_addr[16];                       //not sure if needed
	sim800_send_at_command("AT+CIFSR");
	ret = sim800_expect_cifsr_response(ip_addr, sizeof(ip_addr), 2000);
	if (ret != 0) {
		return ret;
	}
	UARTprintf("Got IP addr: %s\n", ip_addr);

	/* At this point the sim800 module is ready to make a connection */
	return 0;
}

/*======================= TCP connect =======================*/
/* TODO: handler errors like ALREADY CONNECTED */
int sim800_tcp_connect(const char *hostname, uint16_t port)
{
	int ret;
	char buffer[128];

	/* Drop any pending connections */
	sim800_send_at_command("AT+CIPCLOSE=0");
	ret = sim800_expect_normal_response("OK", 2000);
	if (ret != 0 && ret != 1) {
		return -1;
	}

	/* assemble full string in local stack buffer */
	snprintf(buffer, sizeof(buffer), "AT+CIPSTART=\"TCP\",\"%s\",\"%d\"", hostname, port);

	sim800_send_at_command(buffer);
	ret = sim800_expect_cipstart_response(5000);
	if (ret != 0) {
		UARTprintf("Failed to connect to %s:%u\n", hostname, port);
		return ret;
	}

	return 0;
}

/*======================= TCP disconnect =======================*/
int sim800_tcp_disconnect(void)
{
	/* TODO: implement me */
	return 0;
}

/*======================= Expect dataprompt =======================*/
int sim800_expect_dataprompt(int timeout)
{
	size_t i = 0;
	int ret;
	int ch;
	char buf[256];
	bool got_dataprompt = false;

	if (timeout != 0) {
		start_timeout(timeout);
	}

	do {                                                //do
		ret = sim800_expect_char('\r');
		if (ret < 0) {
			stop_timeout();
			return ret;
		}
		ret = sim800_expect_char('\n');
		if (ret < 0) {
			stop_timeout();
			return ret;
		}

		start_timeout(timeout);
		ch = sim800_uart6_getc_blocking();
		if (timed_out()) {
			stop_timeout();
			return -1;
		}
		if (ch != '>') {                                // We got an URC and need to handle it (Used commands AT+CIPSEND)
			do {
				ch = sim800_uart6_getc();

				if (buf != NULL && ch != -1) {          // at a valid buffer pointer and data in the response input buffer
					if (i >= sizeof(buf))               //and when the response is greater than the buffer
						return -1;                      //return -1
					                                    //write the response into the buf buffer
					if (ch != '\r')                     //if the response is not '\r' (end of response)
						buf[i++] = ch;                  //write response char into buf and ++
					else
						buf[i++] = '\0';                //else terminate string by null character
				}
			} while (ch == -1 || ch != '\r');           //while buffer empty or first end-response character
			ret = sim800_expect_char('\n');             //return 0 when second end-response character
			if (ret < 0) {
				stop_timeout();
				return ret;
			}
			handle_urc(buf);                            //call handle_urc
		} else {
			got_dataprompt = true;
		}
	} while (!got_dataprompt);                          //while dataprompt == 0

	ret = sim800_expect_char(' ');
	if (ret < 0) {
		stop_timeout();
		return ret;
	}

	stop_timeout();
	return 0;
}

/*======================= Expect data accept =======================*/
/* returns 0 on success, -1 on timeout and 1 on transmission error */
int sim800_expect_data_accept(int len, int timeout)
{
	int ret;
	char buf[64];
	int accepted;

	do {
		ret = sim800_get_response(buf, sizeof(buf), timeout);
		if (ret != 0) {
			return -1;
		}

		if (sscanf(buf, "DATA ACCEPT:%d", &accepted) == 1) {
			if (accepted != len) {
				UARTprintf("WARN: short write, sent %d, accepted %d\n", len, accepted);
				return 1;
			} else {
				return 0;
			}
		} else if (strncmp(buf, "SEND FAIL", sizeof(buf)) == 0) {
			UARTprintf("WARN: send failed");                                    // send failed return 1
			return 1;
		} else {
			handle_urc(buf);                                                    //failed to parse the line, we got an URC
		}
	} while (1 > 0);

	return -1;                                        // We should never arrive here, returns should be handled in the upper conditions
}

/*======================= TCP send =======================*/
int sim800_tcp_send(uint8_t *data, uint16_t len)
{
	int ret;
	char buffer[64];

	/* tell sim800 module that we want to send len bytes over the connection */
	snprintf(buffer, 64, "AT+CIPSEND=%d", len);

	sim800_send_at_command(buffer);
	ret = sim800_expect_dataprompt(2000);
	if (ret != 0)
		return ret;

	sim800_send_raw_data(data, len);
#ifdef SIM800_QUICKSEND
	ret = sim800_expect_data_accept(len, 2000);
#else
	ret = sim800_expect_normal_response("SEND OK", 10000);
#endif
	return ret;
}

/*======================= MQTT pal sendall =======================*/
ssize_t mqtt_pal_sendall(mqtt_pal_socket_handle fd, const void* buf, size_t len, int flags)
{
	int ret;
	int retry_count = 5;

	do {
		retry_count--;
		ret = sim800_tcp_send((uint8_t *)buf, len);
		if (ret == 0)
			break;

		Task_sleep(250);
	} while (retry_count > 0);

	if (ret != 0)
		return MQTT_ERROR_SOCKET_ERROR;

	return len;
}

/*======================= Expect CIPRXGET response =======================*/
// Return available bytes or negative number on error
int sim800_expect_ciprxget_response(int timeout)
{
	int ret;
	char buf[256];
	int requested, confirmed;

	do {
		ret = sim800_get_response(buf, sizeof(buf), timeout);
		if (ret != 0) {
			return -1;
		}

		if (sscanf(buf, "+CIPRXGET: 2,%d,%d", &requested, &confirmed) != 2) {
			handle_urc(buf);                        //failed to parse the line, we got an URC
		} else {
			return requested;
		}
	} while (1 > 0);

	return -1;                   //we should never arrive here, returns should be handled in the upper conditions
}

/*======================= MQTT pal recvall =======================*/
ssize_t mqtt_pal_recvall(mqtt_pal_socket_handle fd, void* buf, size_t bufsz, int flags)
{
	int ret;
	int avail;
	char cmdbuffer[64];


	/* request up to bufsz bytes from SIM800 module */
	snprintf(cmdbuffer, sizeof(cmdbuffer), "AT+CIPRXGET=2,%d", bufsz);
	sim800_send_at_command(cmdbuffer);

	/* parse response which will also return available bytes */
	ret = sim800_expect_ciprxget_response(2000);
	if (ret < 0) {
		return MQTT_ERROR_SOCKET_ERROR;
	}
	avail = ret;

	start_timeout(10000);
	for (int i = 0; i < avail; i++) {
		int ch = sim800_uart6_getc_blocking();
		if (ch == -1) {
			stop_timeout();
			return MQTT_ERROR_SOCKET_ERROR;
		}
		((char *)buf)[i] = ch;
	}

	ret = sim800_expect_normal_response("OK", 1000);
	if (ret != 0) {
		return MQTT_ERROR_SOCKET_ERROR;
	}

	return avail;
}

/*======================= Publish callback =======================*/
void publish_callback(void **unused, struct mqtt_response_publish *published)
{
	char topic_name[64];
	int len = published->topic_name_size < (sizeof(topic_name) - 1) ? published->topic_name_size : sizeof(topic_name) - 1;
	memcpy(topic_name, published->topic_name, len);
	topic_name[len] = '\0';

	char message[64];
	len = published->application_message_size < (sizeof(message) - 1) ? published->application_message_size : sizeof(message) - 1;
	memcpy(message, published->application_message, len);
	message[len] = '\0';
	UARTprintf("Got a new publish for %s: %s\n", topic_name, message);
}

uint8_t mqtt_sendbuf[512];
uint8_t mqtt_recvbuf[512];

/*======================= SIM800 Task =======================*/
void sim800_task(UArg arg0, UArg arg1)
{
	int ret;
	struct mqtt_client client;

	ret = sim800_init();
	if (ret != 0) {
		UARTprintf("sim800_init() failed\n");
		Task_sleep(100);
		System_abort("sim800_init() failed");
	}

	/* TODO: maybe retry a few times */
	ret = sim800_tcp_connect(MQTT_HOST, MQTT_PORT);
	if (ret != 0) {
		UARTprintf("sim800_tcp_connect() failed\n");
		Task_sleep(100);
		System_abort("sim800_tcp_connect() failed\n");
	}

	mqtt_init(&client, 0, mqtt_sendbuf, sizeof(mqtt_sendbuf), mqtt_recvbuf, sizeof(mqtt_recvbuf), publish_callback);
	mqtt_connect(&client, "sim800", NULL, NULL, 0, MQTT_USER, MQTT_PASS, 0, 400);

    if (client.error != MQTT_OK) {
    	UARTprintf("Connection to mqtt server failed: %s\n", mqtt_error_str(client.error));
    	Task_sleep(100);
    	System_abort("Connection to mqtt server failed");
    }
    UARTprintf("MQTT connect done\n");

	while (1 > 0) {

		enum MQTTErrors err;
		mqtt_msg mboxmsg;

		if (Mailbox_pend(mboxHandleIP, &mboxmsg, 100) != 0)
		{
		    UARTprintf("mailbox event \n");
		    if (mboxmsg.typ == MSG_TYPE_ANA)
		    {
		        char msgbuff[32];
		        sprintf(msgbuff, "ADC 1: %d", mboxmsg.value_1);
		        err = mqtt_publish(&client, "values/ADC1", msgbuff, strlen(msgbuff) + 1, MQTT_PUBLISH_QOS_1);
                if (err != MQTT_OK) {
                    UARTprintf("MQTT error: %s\n", mqtt_error_str(err));
                }
		    }
		    else if (mboxmsg.typ == MSG_TYPE_DIG)
		    {
		        UARTprintf("Got new DIG state: %x\n", mboxmsg.value_1);
                char msgbuff[32];
                sprintf(msgbuff, "DIG: %x", mboxmsg.value_1);
                err = mqtt_publish(&client, "values/DIG", msgbuff, strlen(msgbuff) + 1, MQTT_PUBLISH_QOS_1);
                if (err != MQTT_OK) {
                    UARTprintf("MQTT error: %s\n", mqtt_error_str(err));
                }

		    }
		    else
		    {
		        UARTprintf("WARNING: unknown message type! \n");
		    }
		}

		mqtt_sync(&client);
	}
}

/*======================= Setup SIM800 Task =======================*/
int setup_sim800_task(void)
{
	Task_Params sim800_task_params;
	Task_Handle sim800_task_handle;
	Error_Block eb;

	Error_init(&eb);

	Mailbox_Params mboxParamsIP;


    Mailbox_Params_init (&mboxParamsIP);
    mboxHandleIP = Mailbox_create (sizeof(mqtt_msg), 50, &mboxParamsIP, &eb);
    if (mboxHandleIP == NULL) {
       System_abort("MQTT Mailbox create failed");
    };

	Task_Params_init(&sim800_task_params);
	sim800_task_params.stackSize = 2048;
	sim800_task_params.priority = 14;
	sim800_task_params.arg0 = NULL;
	sim800_task_params.arg1 = NULL;
	sim800_task_handle = Task_create((Task_FuncPtr)sim800_task, &sim800_task_params, &eb);

	if (sim800_task_handle == NULL) {
		System_abort("Failed to create sim800 task");
	}

	return 0;
}

