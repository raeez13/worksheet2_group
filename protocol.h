#ifndef PRIORITY_EVENT_TELEMETRY_PROTOCOL_H
#define PRIORITY_EVENT_TELEMETRY_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PROTOCOL_MAGIC_0 0x50u
#define PROTOCOL_MAGIC_1 0x45u
#define PROTOCOL_MAX_PAYLOAD 64u
#define PROTOCOL_TX_QUEUE_DEPTH 4u
#define PROTOCOL_DEFAULT_TIMEOUT_MS 1000u
#define PROTOCOL_ACK_TIMEOUT_MS 300u
#define PROTOCOL_MAX_RETRIES 2u

typedef enum {
    PROTOCOL_PKT_CONNECT = 0x01,
    PROTOCOL_PKT_CONNECT_ACK = 0x02,
    PROTOCOL_PKT_TELEMETRY = 0x03,
    PROTOCOL_PKT_EVENT = 0x04,
    PROTOCOL_PKT_ACK = 0x05,
    PROTOCOL_PKT_NACK = 0x06,
    PROTOCOL_PKT_DISCONNECT = 0x07,
    PROTOCOL_PKT_COMMAND = 0x08
} protocol_packet_type_t;

typedef enum {
    PROTOCOL_PRIORITY_NORMAL = 0x01,
    PROTOCOL_PRIORITY_HIGH = 0x02,
    PROTOCOL_PRIORITY_CONTROL = 0x03
} protocol_priority_t;

typedef enum {
    PROTOCOL_OK = 0,
    PROTOCOL_ERR_TIMEOUT = -1,
    PROTOCOL_ERR_CHECKSUM = -2,
    PROTOCOL_ERR_LENGTH = -3,
    PROTOCOL_ERR_SEQUENCE = -4,
    PROTOCOL_ERR_QUEUE_FULL = -5,
    PROTOCOL_ERR_NOT_CONNECTED = -6,
    PROTOCOL_ERR_NACK = -7,
    PROTOCOL_ERR_IO = -8
} protocol_status_t;

typedef enum {
    PROTOCOL_NACK_CHECKSUM = 0x01,
    PROTOCOL_NACK_LENGTH = 0x02,
    PROTOCOL_NACK_SEQUENCE = 0x03,
    PROTOCOL_NACK_UNSUPPORTED = 0x04
} protocol_nack_reason_t;

typedef struct {
    uint8_t type;
    uint8_t priority;
    uint8_t sequence;
    uint16_t length;
    uint8_t payload[PROTOCOL_MAX_PAYLOAD];
} protocol_packet_t;

void protocol_init(void);
bool protocol_connect(uint32_t timeout_ms);
protocol_status_t protocol_send(uint8_t packet_type, const uint8_t *payload, uint16_t payload_len);
protocol_status_t protocol_receive(protocol_packet_t *packet, uint32_t timeout_ms);
protocol_status_t protocol_disconnect(void);
void protocol_cleanup(void);

const char *protocol_status_string(protocol_status_t status);

#ifdef __cplusplus
}
#endif

#endif
