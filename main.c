#include <stdint.h>
#include <string.h>

#include "pico/stdlib.h"

#include "protocol.h"

#define EVENT_ID_STARTUP 0x01u
#define EVENT_ID_BUTTON 0x02u
#define EVENT_ID_PRIORITY_TEST 0x03u
#define EVENT_ID_PING 0x04u
#define COMMAND_SET_LED 0x01u
#define COMMAND_PING 0x02u
#define TELEMETRY_PERIOD_MS 1000u
#define PRIORITY_TEST_INTERVAL 5u

typedef struct __attribute__((packed)) {
    uint8_t event_id;
    uint32_t uptime_ms;
    uint16_t detail;
} event_payload_t;

typedef struct __attribute__((packed)) {
    uint32_t uptime_ms;
    uint16_t sample_id;
    int16_t temperature_c_x100;
    uint16_t voltage_mv;
    uint8_t led_state;
} telemetry_payload_t;

static event_payload_t make_event_payload(uint8_t event_id, uint16_t detail) {
    event_payload_t event = {
        .event_id = event_id,
        .uptime_ms = to_ms_since_boot(get_absolute_time()),
        .detail = detail
    };

    return event;
}

static telemetry_payload_t make_telemetry_payload(uint16_t sample_id, bool led_state) {
    telemetry_payload_t telemetry = {
        .uptime_ms = to_ms_since_boot(get_absolute_time()),
        .sample_id = sample_id,
        .temperature_c_x100 = (int16_t)(2150 + (sample_id % 80)),
        .voltage_mv = 3300u,
        .led_state = led_state ? 1u : 0u
    };

    return telemetry;
}

static void send_startup_event(void) {
    event_payload_t event = make_event_payload(EVENT_ID_STARTUP, 0x1234u);

    protocol_send(PROTOCOL_PKT_EVENT, (const uint8_t *)&event, sizeof(event));
}

static void send_telemetry(uint16_t sample_id, bool led_state) {
    telemetry_payload_t telemetry = make_telemetry_payload(sample_id, led_state);

    protocol_status_t status = protocol_send(PROTOCOL_PKT_TELEMETRY,
                                             (const uint8_t *)&telemetry,
                                             sizeof(telemetry));
    if (status != PROTOCOL_OK) {
        event_payload_t event = make_event_payload(
            EVENT_ID_BUTTON,
            (uint16_t)(0xE000u | ((uint16_t)(-status) & 0x00ffu))
        );
        protocol_send(PROTOCOL_PKT_EVENT, (const uint8_t *)&event, sizeof(event));
    }
}

static void send_priority_test_cycle(uint16_t sample_id, bool led_state) {
    telemetry_payload_t telemetry = make_telemetry_payload(sample_id, led_state);
    event_payload_t event = make_event_payload(EVENT_ID_PRIORITY_TEST, sample_id);

    /*
     * Priority demonstration:
     * 1. Queue normal TELEMETRY first.
     * 2. Queue urgent EVENT second.
     * 3. Flush once.
     *
     * If the protocol were FIFO only, the host would see TELEMETRY then EVENT.
     * Because the protocol has a high-priority queue, the host should see EVENT
     * first, then TELEMETRY, with consecutive sequence numbers. This models an
     * embedded system where a fault/alarm must jump ahead of routine sensor data.
     */
    protocol_status_t status = protocol_queue(PROTOCOL_PKT_TELEMETRY,
                                              (const uint8_t *)&telemetry,
                                              sizeof(telemetry));
    if (status == PROTOCOL_OK) {
        status = protocol_queue(PROTOCOL_PKT_EVENT,
                                (const uint8_t *)&event,
                                sizeof(event));
    }

    if (status == PROTOCOL_OK) {
        status = protocol_flush();
    }

    if (status != PROTOCOL_OK) {
        event_payload_t error_event = make_event_payload(
            EVENT_ID_BUTTON,
            (uint16_t)(0xD000u | ((uint16_t)(-status) & 0x00ffu))
        );
        protocol_send(PROTOCOL_PKT_EVENT,
                      (const uint8_t *)&error_event,
                      sizeof(error_event));
    }
}

static void handle_command(const protocol_packet_t *packet, bool *led_state) {
    if (packet->length < 1u) {
        return;
    }

    uint8_t command = packet->payload[0];

    if (command == COMMAND_SET_LED && packet->length >= 2u) {
        *led_state = packet->payload[1] != 0u;
        gpio_put(PICO_DEFAULT_LED_PIN, *led_state);

        event_payload_t event = make_event_payload(EVENT_ID_BUTTON, *led_state ? 1u : 0u);
        protocol_send(PROTOCOL_PKT_EVENT, (const uint8_t *)&event, sizeof(event));
    } else if (command == COMMAND_PING) {
        event_payload_t event = make_event_payload(EVENT_ID_PING, 0xBEEFu);
        protocol_send(PROTOCOL_PKT_EVENT, (const uint8_t *)&event, sizeof(event));
    }
}

int main(void) {
    stdio_init_all();
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    protocol_init();

    while (!protocol_connect(10000u)) {
        sleep_ms(500);
    }

    bool led_state = false;
    uint16_t sample_id = 0u;
    absolute_time_t next_telemetry = make_timeout_time_ms(TELEMETRY_PERIOD_MS);

    send_startup_event();

    while (true) {
        protocol_packet_t packet;
        protocol_status_t rx_status = protocol_receive(&packet, 10u);

        if (rx_status == PROTOCOL_OK && packet.type == PROTOCOL_PKT_COMMAND) {
            handle_command(&packet, &led_state);
        }

        if (absolute_time_diff_us(get_absolute_time(), next_telemetry) <= 0) {
            /*
             * Every fifth telemetry cycle, deliberately queue TELEMETRY first
             * and EVENT second. The host output should still show EVENT before
             * TELEMETRY, proving that urgent traffic is prioritised while the
             * normal telemetry stream is already running.
             */
            if (sample_id > 0u && (sample_id % PRIORITY_TEST_INTERVAL) == 0u) {
                send_priority_test_cycle(sample_id, led_state);
            } else {
                send_telemetry(sample_id, led_state);
            }

            sample_id++;
            next_telemetry = make_timeout_time_ms(TELEMETRY_PERIOD_MS);
        }

        tight_loop_contents();
    }
}
