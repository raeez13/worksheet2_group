#include <stdint.h>
#include <string.h>

#include "pico/stdlib.h"

#include "protocol.h"

#define EVENT_ID_STARTUP 0x01u
#define EVENT_ID_BUTTON 0x02u
#define COMMAND_SET_LED 0x01u
#define COMMAND_PING 0x02u

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

static void send_startup_event(void) {
    event_payload_t event = {
        .event_id = EVENT_ID_STARTUP,
        .uptime_ms = to_ms_since_boot(get_absolute_time()),
        .detail = 0x1234u
    };

    protocol_send(PROTOCOL_PKT_EVENT, (const uint8_t *)&event, sizeof(event));
}

static void send_telemetry(uint16_t sample_id, bool led_state) {
    uint32_t now_ms = to_ms_since_boot(get_absolute_time());
    telemetry_payload_t telemetry = {
        .uptime_ms = now_ms,
        .sample_id = sample_id,
        .temperature_c_x100 = (int16_t)(2150 + (sample_id % 80)),
        .voltage_mv = 3300u,
        .led_state = led_state ? 1u : 0u
    };

    protocol_status_t status = protocol_send(PROTOCOL_PKT_TELEMETRY,
                                             (const uint8_t *)&telemetry,
                                             sizeof(telemetry));
    if (status != PROTOCOL_OK) {
        event_payload_t event = {
            .event_id = EVENT_ID_BUTTON,
            .uptime_ms = now_ms,
            .detail = (uint16_t)(0xE000u | ((uint16_t)(-status) & 0x00ffu))
        };
        protocol_send(PROTOCOL_PKT_EVENT, (const uint8_t *)&event, sizeof(event));
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

        event_payload_t event = {
            .event_id = EVENT_ID_BUTTON,
            .uptime_ms = to_ms_since_boot(get_absolute_time()),
            .detail = *led_state ? 1u : 0u
        };
        protocol_send(PROTOCOL_PKT_EVENT, (const uint8_t *)&event, sizeof(event));
    } else if (command == COMMAND_PING) {
        event_payload_t event = {
            .event_id = COMMAND_PING,
            .uptime_ms = to_ms_since_boot(get_absolute_time()),
            .detail = 0xBEEFu
        };
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
    absolute_time_t next_telemetry = make_timeout_time_ms(1000u);

    send_startup_event();

    while (true) {
        protocol_packet_t packet;
        protocol_status_t rx_status = protocol_receive(&packet, 10u);

        if (rx_status == PROTOCOL_OK && packet.type == PROTOCOL_PKT_COMMAND) {
            handle_command(&packet, &led_state);
        }

        if (absolute_time_diff_us(get_absolute_time(), next_telemetry) <= 0) {
            send_telemetry(sample_id++, led_state);
            next_telemetry = make_timeout_time_ms(1000u);
        }

        tight_loop_contents();
    }
}
