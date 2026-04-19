#include "protocol.h"

#include <stddef.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/stdio.h"
#include "pico/stdio_usb.h"

#define PROTOCOL_HEADER_BYTES 5u

typedef struct {
    uint8_t type;
    uint8_t priority;
    uint16_t length;
    uint8_t payload[PROTOCOL_MAX_PAYLOAD];
} tx_item_t;

typedef struct {
    tx_item_t items[PROTOCOL_TX_QUEUE_DEPTH];
    uint8_t head;
    uint8_t tail;
    uint8_t count;
} tx_queue_t;

static tx_queue_t high_queue;
static tx_queue_t normal_queue;
static uint8_t next_tx_sequence;
static uint8_t expected_rx_sequence;
static bool is_connected;
static bool pending_rx_valid;
static protocol_packet_t pending_rx_packet;

static uint8_t priority_for_type(uint8_t type) {
    switch (type) {
        case PROTOCOL_PKT_TELEMETRY:
            return PROTOCOL_PRIORITY_NORMAL;
        case PROTOCOL_PKT_EVENT:
            return PROTOCOL_PRIORITY_HIGH;
        case PROTOCOL_PKT_CONNECT:
        case PROTOCOL_PKT_CONNECT_ACK:
        case PROTOCOL_PKT_ACK:
        case PROTOCOL_PKT_NACK:
        case PROTOCOL_PKT_DISCONNECT:
        case PROTOCOL_PKT_COMMAND:
        default:
            return PROTOCOL_PRIORITY_CONTROL;
    }
}

static bool requires_ack(uint8_t type) {
    return type != PROTOCOL_PKT_ACK &&
           type != PROTOCOL_PKT_NACK &&
           type != PROTOCOL_PKT_CONNECT_ACK;
}

static uint8_t crc8_update(uint8_t crc, uint8_t data) {
    crc ^= data;
    for (uint8_t bit = 0; bit < 8; bit++) {
        if ((crc & 0x80u) != 0u) {
            crc = (uint8_t)((crc << 1u) ^ 0x07u);
        } else {
            crc = (uint8_t)(crc << 1u);
        }
    }
    return crc;
}

static uint8_t packet_crc8(uint8_t type,
                           uint8_t priority,
                           uint8_t sequence,
                           uint16_t length,
                           const uint8_t *payload) {
    uint8_t crc = 0x00u;
    crc = crc8_update(crc, type);
    crc = crc8_update(crc, priority);
    crc = crc8_update(crc, sequence);
    crc = crc8_update(crc, (uint8_t)(length & 0xffu));
    crc = crc8_update(crc, (uint8_t)((length >> 8u) & 0xffu));

    for (uint16_t i = 0; i < length; i++) {
        crc = crc8_update(crc, payload[i]);
    }

    return crc;
}

static void queue_reset(tx_queue_t *queue) {
    queue->head = 0u;
    queue->tail = 0u;
    queue->count = 0u;
}

static bool queue_push(tx_queue_t *queue,
                       uint8_t type,
                       uint8_t priority,
                       const uint8_t *payload,
                       uint16_t length) {
    if (queue->count >= PROTOCOL_TX_QUEUE_DEPTH) {
        return false;
    }

    tx_item_t *item = &queue->items[queue->tail];
    item->type = type;
    item->priority = priority;
    item->length = length;
    if (length > 0u && payload != NULL) {
        memcpy(item->payload, payload, length);
    }

    queue->tail = (uint8_t)((queue->tail + 1u) % PROTOCOL_TX_QUEUE_DEPTH);
    queue->count++;
    return true;
}

static bool queue_pop(tx_queue_t *queue, tx_item_t *item) {
    if (queue->count == 0u) {
        return false;
    }

    *item = queue->items[queue->head];
    queue->head = (uint8_t)((queue->head + 1u) % PROTOCOL_TX_QUEUE_DEPTH);
    queue->count--;
    return true;
}

static bool dequeue_next(tx_item_t *item) {
    if (queue_pop(&high_queue, item)) {
        return true;
    }

    return queue_pop(&normal_queue, item);
}

static bool deadline_has_time(absolute_time_t deadline) {
    return absolute_time_diff_us(get_absolute_time(), deadline) > 0;
}

static bool read_byte_until(uint8_t *byte_out, absolute_time_t deadline) {
    while (deadline_has_time(deadline)) {
        int value = getchar_timeout_us(1000);
        if (value == PICO_ERROR_TIMEOUT) {
            continue;
        }

        *byte_out = (uint8_t)value;
        return true;
    }

    return false;
}

static void write_frame(uint8_t type,
                        uint8_t priority,
                        uint8_t sequence,
                        const uint8_t *payload,
                        uint16_t length) {
    uint8_t crc = packet_crc8(type, priority, sequence, length, payload);

    putchar_raw(PROTOCOL_MAGIC_0);
    putchar_raw(PROTOCOL_MAGIC_1);
    putchar_raw(type);
    putchar_raw(priority);
    putchar_raw(sequence);
    putchar_raw((uint8_t)(length & 0xffu));
    putchar_raw((uint8_t)((length >> 8u) & 0xffu));

    for (uint16_t i = 0; i < length; i++) {
        putchar_raw(payload[i]);
    }

    putchar_raw(crc);
    stdio_flush();
}

static void send_ack(uint8_t sequence) {
    uint8_t payload[1] = {sequence};
    write_frame(PROTOCOL_PKT_ACK,
                PROTOCOL_PRIORITY_CONTROL,
                sequence,
                payload,
                sizeof(payload));
}

static void send_nack(uint8_t sequence, uint8_t expected, uint8_t reason) {
    uint8_t payload[3] = {sequence, expected, reason};
    write_frame(PROTOCOL_PKT_NACK,
                PROTOCOL_PRIORITY_CONTROL,
                sequence,
                payload,
                sizeof(payload));
}

static void send_connect_ack(uint8_t sequence) {
    uint8_t payload[2] = {'O', 'K'};
    write_frame(PROTOCOL_PKT_CONNECT_ACK,
                PROTOCOL_PRIORITY_CONTROL,
                sequence,
                payload,
                sizeof(payload));
}

static protocol_status_t read_frame(protocol_packet_t *packet, uint32_t timeout_ms) {
    absolute_time_t deadline = make_timeout_time_ms(timeout_ms);
    uint8_t byte = 0u;
    uint8_t matched = 0u;

    while (deadline_has_time(deadline)) {
        if (!read_byte_until(&byte, deadline)) {
            return PROTOCOL_ERR_TIMEOUT;
        }

        if (matched == 0u) {
            if (byte == PROTOCOL_MAGIC_0) {
                matched = 1u;
            }
        } else {
            if (byte == PROTOCOL_MAGIC_1) {
                break;
            }

            matched = (byte == PROTOCOL_MAGIC_0) ? 1u : 0u;
        }
    }

    if (matched != 1u || !deadline_has_time(deadline)) {
        return PROTOCOL_ERR_TIMEOUT;
    }

    uint8_t header[PROTOCOL_HEADER_BYTES];
    for (uint8_t i = 0; i < PROTOCOL_HEADER_BYTES; i++) {
        if (!read_byte_until(&header[i], deadline)) {
            return PROTOCOL_ERR_TIMEOUT;
        }
    }

    packet->type = header[0];
    packet->priority = header[1];
    packet->sequence = header[2];
    packet->length = (uint16_t)header[3] | ((uint16_t)header[4] << 8u);

    if (packet->length > PROTOCOL_MAX_PAYLOAD) {
        return PROTOCOL_ERR_LENGTH;
    }

    for (uint16_t i = 0; i < packet->length; i++) {
        if (!read_byte_until(&packet->payload[i], deadline)) {
            return PROTOCOL_ERR_TIMEOUT;
        }
    }

    uint8_t received_crc = 0u;
    if (!read_byte_until(&received_crc, deadline)) {
        return PROTOCOL_ERR_TIMEOUT;
    }

    uint8_t expected_crc = packet_crc8(packet->type,
                                       packet->priority,
                                       packet->sequence,
                                       packet->length,
                                       packet->payload);
    if (received_crc != expected_crc) {
        return PROTOCOL_ERR_CHECKSUM;
    }

    return PROTOCOL_OK;
}

static protocol_status_t accept_incoming_packet(const protocol_packet_t *packet, bool store_for_app) {
    switch (packet->type) {
        case PROTOCOL_PKT_CONNECT:
            expected_rx_sequence = 0u;
            next_tx_sequence = 0u;
            is_connected = true;
            send_connect_ack(packet->sequence);
            return PROTOCOL_OK;

        case PROTOCOL_PKT_CONNECT_ACK:
            is_connected = true;
            return PROTOCOL_OK;

        case PROTOCOL_PKT_ACK:
        case PROTOCOL_PKT_NACK:
            return PROTOCOL_OK;

        case PROTOCOL_PKT_TELEMETRY:
        case PROTOCOL_PKT_EVENT:
        case PROTOCOL_PKT_COMMAND:
        case PROTOCOL_PKT_DISCONNECT:
            if (packet->sequence != expected_rx_sequence) {
                send_nack(packet->sequence,
                          expected_rx_sequence,
                          PROTOCOL_NACK_SEQUENCE);
                return PROTOCOL_ERR_SEQUENCE;
            }

            send_ack(packet->sequence);
            expected_rx_sequence = (uint8_t)(expected_rx_sequence + 1u);

            if (packet->type == PROTOCOL_PKT_DISCONNECT) {
                is_connected = false;
            }

            if (store_for_app) {
                pending_rx_packet = *packet;
                pending_rx_valid = true;
            }

            return PROTOCOL_OK;

        default:
            send_nack(packet->sequence,
                      expected_rx_sequence,
                      PROTOCOL_NACK_UNSUPPORTED);
            return PROTOCOL_ERR_IO;
    }
}

static protocol_status_t wait_for_ack(uint8_t sequence, uint32_t timeout_ms) {
    absolute_time_t deadline = make_timeout_time_ms(timeout_ms);

    while (deadline_has_time(deadline)) {
        uint32_t remaining_ms = (uint32_t)(absolute_time_diff_us(get_absolute_time(), deadline) / 1000);
        if (remaining_ms == 0u) {
            remaining_ms = 1u;
        }

        protocol_packet_t packet;
        protocol_status_t status = read_frame(&packet, remaining_ms);

        if (status == PROTOCOL_ERR_TIMEOUT) {
            return PROTOCOL_ERR_TIMEOUT;
        }

        if (status == PROTOCOL_ERR_LENGTH) {
            send_nack(packet.sequence, expected_rx_sequence, PROTOCOL_NACK_LENGTH);
            continue;
        }

        if (status == PROTOCOL_ERR_CHECKSUM) {
            send_nack(packet.sequence, expected_rx_sequence, PROTOCOL_NACK_CHECKSUM);
            continue;
        }

        if (status != PROTOCOL_OK) {
            return status;
        }

        if (packet.type == PROTOCOL_PKT_ACK && packet.sequence == sequence) {
            return PROTOCOL_OK;
        }

        if (packet.type == PROTOCOL_PKT_NACK && packet.sequence == sequence) {
            return PROTOCOL_ERR_NACK;
        }

        protocol_status_t incoming_status = accept_incoming_packet(&packet, true);
        if (incoming_status != PROTOCOL_OK && incoming_status != PROTOCOL_ERR_SEQUENCE) {
            return incoming_status;
        }
    }

    return PROTOCOL_ERR_TIMEOUT;
}

static protocol_status_t transmit_item_with_retry(const tx_item_t *item) {
    uint8_t sequence = next_tx_sequence++;

    for (uint8_t attempt = 0; attempt <= PROTOCOL_MAX_RETRIES; attempt++) {
        write_frame(item->type,
                    item->priority,
                    sequence,
                    item->payload,
                    item->length);

        if (!requires_ack(item->type)) {
            return PROTOCOL_OK;
        }

        protocol_status_t status = wait_for_ack(sequence, PROTOCOL_ACK_TIMEOUT_MS);
        if (status == PROTOCOL_OK) {
            return PROTOCOL_OK;
        }

        if (status != PROTOCOL_ERR_TIMEOUT && status != PROTOCOL_ERR_NACK) {
            return status;
        }
    }

    return PROTOCOL_ERR_TIMEOUT;
}

static protocol_status_t flush_tx_queue(void) {
    tx_item_t item;

    while (dequeue_next(&item)) {
        protocol_status_t status = transmit_item_with_retry(&item);
        if (status != PROTOCOL_OK) {
            return status;
        }
    }

    return PROTOCOL_OK;
}

void protocol_init(void) {
    queue_reset(&high_queue);
    queue_reset(&normal_queue);
    next_tx_sequence = 0u;
    expected_rx_sequence = 0u;
    is_connected = false;
    pending_rx_valid = false;
    memset(&pending_rx_packet, 0, sizeof(pending_rx_packet));
}

bool protocol_connect(uint32_t timeout_ms) {
    absolute_time_t deadline = make_timeout_time_ms(timeout_ms);
    uint8_t hello_payload[4] = {'P', 'I', 'C', 'O'};

    next_tx_sequence = 0u;
    expected_rx_sequence = 0u;

    while (deadline_has_time(deadline) && !stdio_usb_connected()) {
        sleep_ms(10);
    }

    while (deadline_has_time(deadline)) {
        uint8_t sequence = 0u;
        write_frame(PROTOCOL_PKT_CONNECT,
                    PROTOCOL_PRIORITY_CONTROL,
                    sequence,
                    hello_payload,
                    sizeof(hello_payload));

        absolute_time_t round_deadline = make_timeout_time_ms(500u);
        while (deadline_has_time(deadline) && deadline_has_time(round_deadline)) {
            protocol_packet_t packet;
            protocol_status_t status = read_frame(&packet, 50u);
            if (status == PROTOCOL_ERR_TIMEOUT) {
                continue;
            }

            if (status == PROTOCOL_ERR_LENGTH) {
                send_nack(packet.sequence, expected_rx_sequence, PROTOCOL_NACK_LENGTH);
                continue;
            }

            if (status == PROTOCOL_ERR_CHECKSUM) {
                send_nack(packet.sequence, expected_rx_sequence, PROTOCOL_NACK_CHECKSUM);
                continue;
            }

            if (status != PROTOCOL_OK) {
                continue;
            }

            if (packet.type == PROTOCOL_PKT_CONNECT_ACK) {
                is_connected = true;
                return true;
            }

            if (packet.type == PROTOCOL_PKT_CONNECT) {
                accept_incoming_packet(&packet, false);
                return true;
            }
        }
    }

    return false;
}

protocol_status_t protocol_send(uint8_t packet_type,
                                const uint8_t *payload,
                                uint16_t payload_len) {
    if (payload_len > PROTOCOL_MAX_PAYLOAD) {
        return PROTOCOL_ERR_LENGTH;
    }

    if (payload_len > 0u && payload == NULL) {
        return PROTOCOL_ERR_IO;
    }

    if (!is_connected &&
        packet_type != PROTOCOL_PKT_CONNECT &&
        packet_type != PROTOCOL_PKT_CONNECT_ACK) {
        return PROTOCOL_ERR_NOT_CONNECTED;
    }

    uint8_t priority = priority_for_type(packet_type);
    tx_queue_t *queue = (priority == PROTOCOL_PRIORITY_NORMAL) ? &normal_queue : &high_queue;

    if (!queue_push(queue, packet_type, priority, payload, payload_len)) {
        return PROTOCOL_ERR_QUEUE_FULL;
    }

    return flush_tx_queue();
}

protocol_status_t protocol_receive(protocol_packet_t *packet, uint32_t timeout_ms) {
    if (packet == NULL) {
        return PROTOCOL_ERR_IO;
    }

    if (pending_rx_valid) {
        *packet = pending_rx_packet;
        pending_rx_valid = false;
        return PROTOCOL_OK;
    }

    absolute_time_t deadline = make_timeout_time_ms(timeout_ms);

    while (deadline_has_time(deadline)) {
        uint32_t remaining_ms = (uint32_t)(absolute_time_diff_us(get_absolute_time(), deadline) / 1000);
        if (remaining_ms == 0u) {
            remaining_ms = 1u;
        }

        protocol_status_t status = read_frame(packet, remaining_ms);
        if (status == PROTOCOL_ERR_TIMEOUT) {
            return PROTOCOL_ERR_TIMEOUT;
        }

        if (status == PROTOCOL_ERR_LENGTH) {
            send_nack(packet->sequence, expected_rx_sequence, PROTOCOL_NACK_LENGTH);
            continue;
        }

        if (status == PROTOCOL_ERR_CHECKSUM) {
            send_nack(packet->sequence, expected_rx_sequence, PROTOCOL_NACK_CHECKSUM);
            continue;
        }

        if (status != PROTOCOL_OK) {
            return status;
        }

        status = accept_incoming_packet(packet, false);
        if (status == PROTOCOL_OK) {
            if (packet->type == PROTOCOL_PKT_CONNECT ||
                packet->type == PROTOCOL_PKT_CONNECT_ACK ||
                packet->type == PROTOCOL_PKT_ACK ||
                packet->type == PROTOCOL_PKT_NACK) {
                continue;
            }

            return PROTOCOL_OK;
        }

        if (status != PROTOCOL_ERR_SEQUENCE) {
            return status;
        }
    }

    return PROTOCOL_ERR_TIMEOUT;
}

protocol_status_t protocol_disconnect(void) {
    protocol_status_t status = protocol_send(PROTOCOL_PKT_DISCONNECT, NULL, 0u);
    is_connected = false;
    return status;
}

void protocol_cleanup(void) {
    queue_reset(&high_queue);
    queue_reset(&normal_queue);
    pending_rx_valid = false;
    is_connected = false;
    stdio_flush();
}

const char *protocol_status_string(protocol_status_t status) {
    switch (status) {
        case PROTOCOL_OK:
            return "OK";
        case PROTOCOL_ERR_TIMEOUT:
            return "timeout";
        case PROTOCOL_ERR_CHECKSUM:
            return "checksum error";
        case PROTOCOL_ERR_LENGTH:
            return "invalid length";
        case PROTOCOL_ERR_SEQUENCE:
            return "sequence mismatch";
        case PROTOCOL_ERR_QUEUE_FULL:
            return "tx queue full";
        case PROTOCOL_ERR_NOT_CONNECTED:
            return "not connected";
        case PROTOCOL_ERR_NACK:
            return "negative acknowledgement";
        case PROTOCOL_ERR_IO:
        default:
            return "I/O error";
    }
}
