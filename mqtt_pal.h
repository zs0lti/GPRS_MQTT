#ifndef __MQTT_PAL_H__
#define __MQTT_PAL_H__

/**
 * @file
 * @brief Includes/supports the types/calls required by the MQTT-C client.
 * 
 * @note This is the \em only file included in mqtt.h, and mqtt.c. It is therefore 
 *       responsible for including/supporting all the required types and calls. 
 * 
 * @defgroup pal Platform abstraction layer
 * @brief Documentation of the types and calls required to port MQTT-C to a new platform.
 * 
 * mqtt_pal.h is the \em only header file included in mqtt.c. Therefore, to port MQTT-C to a 
 * new platform the following types, functions, constants, and macros must be defined in 
 * mqtt_pal.h:
 *  - Types:
 *      - \c size_t, \c ssize_t
 *      - \c uint8_t, \c uint16_t, \c uint32_t
 *      - \c va_list
 *      - \c mqtt_pal_time_t : return type of \c MQTT_PAL_TIME() 
 *      - \c mqtt_pal_mutex_t : type of the argument that is passed to \c MQTT_PAL_MUTEX_LOCK and 
 *        \c MQTT_PAL_MUTEX_RELEASE
 *  - Functions:
 *      - \c memcpy, \c strlen
 *      - \c va_start, \c va_arg, \c va_end
 *  - Constants:
 *      - \c INT_MIN
 * 
 * Additionally, three macro's are required:
 *  - \c MQTT_PAL_HTONS(s) : host-to-network endian conversion for uint16_t.
 *  - \c MQTT_PAL_NTOHS(s) : network-to-host endian conversion for uint16_t.
 *  - \c MQTT_PAL_TIME()   : returns [type: \c mqtt_pal_time_t] current time in seconds. 
 *  - \c MQTT_PAL_MUTEX_LOCK(mtx_pointer) : macro that locks the mutex pointed to by \c mtx_pointer.
 *  - \c MQTT_PAL_MUTEX_RELEASE(mtx_pointer) : macro that unlocks the mutex pointed to by 
 *    \c mtx_pointer.
 * 
 * Lastly, \ref mqtt_pal_sendall and \ref mqtt_pal_recvall, must be implemented in mqtt_pal.c 
 * for sending and receiving data using the platforms socket calls.
 */

#include <limits.h>
#include <stdint.h>
#include <sys/types.h>
#include <ti/sysbios/knl/Clock.h>
#include <string.h>

/* imported from the Linux kernel */
#define htons(a) ( (((a)>>8)&0xff) + (((a)<<8)&0xff00) )
#define htonl(a) ((((a) & 0xff000000) >> 24) | (((a) & 0x00ff0000) >> 8) | \
                  (((a) & 0x0000ff00) << 8)  | (((a) & 0x000000ff) << 24) )

#define ntohl(a) htonl(a)
#define ntohs(a) htons(a)

#define MQTT_PAL_HTONS(s) htons(s)
#define MQTT_PAL_NTOHS(s) ntohs(s)

/* TODO: handle overflows (after ~50 days) */
#define MQTT_PAL_TIME() (Clock_getTicks() / 1000)
typedef uint32_t mqtt_pal_time_t;
typedef int mqtt_pal_mutex_t;

/* we do not require any mutexes because we only have a single task sending MQTT messages from the mailbox */
#define MQTT_PAL_MUTEX_INIT(mtx_ptr) (0)
#define MQTT_PAL_MUTEX_LOCK(mtx_ptr) (0)
#define MQTT_PAL_MUTEX_UNLOCK(mtx_ptr) (0)

typedef int mqtt_pal_socket_handle;

/**
 * @brief Sends all the bytes in a buffer.
 * @ingroup pal
 * 
 * @param[in] fd The file-descriptor (or handle) of the socket.
 * @param[in] buf A pointer to the first byte in the buffer to send.
 * @param[in] len The number of bytes to send (starting at \p buf).
 * @param[in] flags Flags which are passed to the underlying socket.
 * 
 * @returns The number of bytes sent if successful, an \ref MQTTErrors otherwise.
 */
ssize_t mqtt_pal_sendall(mqtt_pal_socket_handle fd, const void* buf, size_t len, int flags);

/**
 * @brief Non-blocking receive all the byte available.
 * @ingroup pal
 * 
 * @param[in] fd The file-descriptor (or handle) of the socket.
 * @param[in] buf A pointer to the receive buffer.
 * @param[in] bufsz The max number of bytes that can be put into \p buf.
 * @param[in] flags Flags which are passed to the underlying socket.
 * 
 * @returns The number of bytes received if successful, an \ref MQTTErrors otherwise.
 */
ssize_t mqtt_pal_recvall(mqtt_pal_socket_handle fd, void* buf, size_t bufsz, int flags);

#endif
