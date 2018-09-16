#ifndef SIM800_H_
#define SIM800_H_

#include <stdint.h>
#include <ti/sysbios/knl/Mailbox.h>

extern Mailbox_Handle mboxHandleIP;
#define MSG_TYPE_ANA 0
#define MSG_TYPE_DIG 1

typedef struct  {
    int typ;
    uint32_t value_1;
    } mqtt_msg;

int setup_sim800_task(void);



#endif /* SIM800_H_ */
