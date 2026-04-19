# Priority Event + Telemetry Protocol

University project implementation of a lightweight custom binary communication
protocol for a Raspberry Pi Pico and a Python host. The link uses USB Serial
through the Pico SDK stdio USB backend and PySerial on the host.

The protocol is called **Priority Event + Telemetry Protocol** because it gives
urgent event packets priority over routine telemetry packets while still adding
framing, sequence numbers, ACK/NACK reliability, checksum validation, timeout
handling, and stream resynchronisation.

## Project Overview

Raw USB CDC serial is only a byte stream. It does not know where one message
starts or ends, whether a payload is corrupted, whether a packet has been lost,
or whether an urgent warning should be transmitted before routine sensor data.

This project adds a compact binary protocol layer above that byte stream:

- Magic bytes mark the start of every packet.
- A fixed header describes type, priority, sequence number, and payload length.
- A CRC-8 checksum detects corrupted packets.
- Sequence numbers detect duplicates, drops, and out-of-order packets.
- ACK and NACK packets provide a simple reliability mechanism.
- A priority-aware transmit queue sends urgent EVENT packets before normal
  TELEMETRY packets when both are pending.

The implementation is intentionally small enough for the Pico. It avoids dynamic
allocation on the embedded side and limits payloads to 64 bytes.

## Why This Is Useful for Raspberry Pi Pico

The Raspberry Pi Pico is often used for sensing, control, and real-time
interaction. A common beginner design is to print text lines over serial. That
works for debugging, but it is weak for embedded communication because text is
larger, slower to parse, and ambiguous when bytes are dropped or mixed.

This protocol is useful because it gives the Pico a more realistic embedded
communication interface:

- Binary frames are compact and fast to parse.
- Telemetry can be streamed periodically without blocking urgent events.
- The host can detect corrupted data and request retransmission.
- The receiver can recover after noise by scanning for the magic bytes.
- The API resembles a small socket layer: initialise, connect, send, receive,
  disconnect, and clean up.

## Why It Is Creative

The outside-the-box part is the combination of priority scheduling and
reliability on a very simple USB serial link. USB CDC does not expose network
features such as packet priority, session state, or application-level ACK/NACK.
This project builds those behaviours above the byte stream while staying small
enough for a microcontroller.

The result behaves like a miniature embedded transport protocol:

- TELEMETRY is treated like regular background traffic.
- EVENT is treated like interrupt-style urgent traffic.
- Control packets such as CONNECT, ACK, NACK, and DISCONNECT use control
  priority.
- The same framing rules work in both directions between Pico and Python.

This is creative for embedded systems because it models a common real-world
need: routine sensor data must not hide a fault, button press, alarm, or safety
event behind less important messages.

## Repository Files

```text
.
├── CMakeLists.txt
├── README.md
├── custom_protocol.py
├── main.c
├── protocol.c
├── protocol.h
└── tests
    └── test_custom_protocol.py
```

## Protocol Specification

All multi-byte integer fields are little-endian. The current implementation uses
a maximum payload size of 64 bytes.

### Packet Structure

| Offset | Size | Field | Description |
|---:|---:|---|---|
| 0 | 2 | Magic bytes | Constant `0x50 0x45`, ASCII `PE` |
| 2 | 1 | Packet type | CONNECT, TELEMETRY, EVENT, ACK, etc. |
| 3 | 1 | Priority | Normal, high, or control |
| 4 | 1 | Sequence number | 8-bit data sequence, wraps at 255 |
| 5 | 2 | Payload length | Unsigned little-endian length |
| 7 | N | Payload | Binary payload, `0..64` bytes |
| 7 + N | 1 | CRC-8 | CRC over type, priority, sequence, length, payload |

### Packet Diagram

```text
+---------+---------+------+----------+-----+----------+-------------+------+
| Magic 0 | Magic 1 | Type | Priority | Seq | Len (LE) | Payload (N) | CRC8 |
|  0x50   |  0x45   | 1 B  |   1 B    | 1 B |   2 B    |   0..64 B   | 1 B  |
+---------+---------+------+----------+-----+----------+-------------+------+
```

The CRC-8 uses polynomial `0x07`, initial value `0x00`, and no final XOR.
The magic bytes are not included in the CRC. This makes resynchronisation
simpler because the receiver can scan for the next magic sequence after an
error.

### Packet Types

| Name | Value | Direction | Purpose |
|---|---:|---|---|
| CONNECT | `0x01` | Both | Start a logical session |
| CONNECT_ACK | `0x02` | Both | Confirm CONNECT |
| TELEMETRY | `0x03` | Pico to host | Normal periodic sensor/status data |
| EVENT | `0x04` | Pico to host | Urgent event or alarm |
| ACK | `0x05` | Both | Positive acknowledgement |
| NACK | `0x06` | Both | Negative acknowledgement with reason |
| DISCONNECT | `0x07` | Both | End a logical session |
| COMMAND | `0x08` | Host to Pico | Demo command packet, for example LED control |

### Priorities

| Name | Value | Used by |
|---|---:|---|
| NORMAL | `0x01` | TELEMETRY |
| HIGH | `0x02` | EVENT |
| CONTROL | `0x03` | CONNECT, CONNECT_ACK, ACK, NACK, DISCONNECT, COMMAND |

## Priority Handling

The Pico implementation keeps two transmit queues:

- A normal queue for TELEMETRY packets.
- A high/control queue for EVENT and control packets.

When the protocol is ready to send, it always removes a packet from the
high/control queue before the normal queue. Therefore, if a TELEMETRY packet and
an EVENT packet are both waiting, the EVENT packet is transmitted first.

This is not a replacement for hard real-time interrupt handling. It is an
application-layer scheduling rule for the serial protocol. The design is still
valuable because routine telemetry often builds up during normal operation, but
faults and alerts should be reported as soon as possible.

## Reliability and Error Handling

### Corrupted Packets

If the CRC-8 check fails, the receiver sends a NACK with reason
`NACK_CHECKSUM`. Because the length field has already been read, the receiver
can consume the payload and CRC byte, reject the packet, and continue scanning.

### Invalid Length

If the length field is greater than 64 bytes, the receiver sends a NACK with
reason `NACK_LENGTH`. It then searches for the next magic byte sequence to
recover framing.

### Sequence Mismatch

Each side keeps an expected receive sequence number for application data.
CONNECT resets the session sequence state and does not consume a data sequence
number. ACK and NACK use the sequence field to identify the packet being
acknowledged. Reliable application packets must arrive with the expected
sequence. If the value is wrong, the receiver sends a NACK with reason
`NACK_SEQUENCE`. This catches dropped, duplicated, or reordered packets.

### Timeout

Sends wait for an ACK. If no ACK arrives before the ACK timeout, the packet is
retried. After the retry limit is reached, the send operation fails. Receives
also have a timeout so the application loop is not permanently blocked.

### Resynchronisation

The receiver treats the serial link as an arbitrary byte stream. It scans byte
by byte until it sees the magic sequence `0x50 0x45`. If noise or a malformed
packet appears, later valid packets can still be found.

## Pico API

The embedded side exposes the required socket-like functions:

```c
void protocol_init(void);
bool protocol_connect(uint32_t timeout_ms);
protocol_status_t protocol_send(uint8_t packet_type,
                                const uint8_t *payload,
                                uint16_t payload_len);
protocol_status_t protocol_receive(protocol_packet_t *packet,
                                   uint32_t timeout_ms);
protocol_status_t protocol_disconnect(void);
void protocol_cleanup(void);
```

## Python API

The host side exposes the required `CustomProtocol` class:

```python
protocol = CustomProtocol("/dev/ttyACM0")
protocol.connect()
protocol.send(PKT_COMMAND, bytes([0x01, 1]))
packet = protocol.receive()
protocol.disconnect()
protocol.cleanup()
```

## Build Instructions for Pico

Install the Raspberry Pi Pico SDK and set `PICO_SDK_PATH`:

```bash
export PICO_SDK_PATH=/path/to/pico-sdk
```

Build the firmware:

```bash
mkdir build
cd build
cmake ..
cmake --build .
```

Flash the generated UF2 file:

```bash
cp priority_event_telemetry.uf2 /Volumes/RPI-RP2/
```

On Linux the Pico usually appears as `/dev/ttyACM0`. On macOS it is usually a
`/dev/tty.usbmodem...` device. On Windows it appears as a COM port.

## Host Setup

Install PySerial:

```bash
python3 -m pip install pyserial
```

Run the host:

```bash
python3 custom_protocol.py /dev/ttyACM0
```

Send an LED command after connecting:

```bash
python3 custom_protocol.py /dev/ttyACM0 --led on
python3 custom_protocol.py /dev/ttyACM0 --led off
```

## Demo Behaviour

The Pico demo does the following:

1. Initialises USB Serial and the custom protocol.
2. Performs a CONNECT / CONNECT_ACK handshake with the Python host.
3. Sends a startup EVENT packet.
4. Sends TELEMETRY once per second.
5. Receives COMMAND packets from Python.
6. Uses COMMAND `0x01` to set the onboard LED.
7. Uses COMMAND `0x02` as a ping command and responds with an EVENT.

The telemetry payload is binary:

| Field | Type | Meaning |
|---|---|---|
| uptime_ms | `uint32_t` | Pico uptime in milliseconds |
| sample_id | `uint16_t` | Telemetry sample counter |
| temperature_c_x100 | `int16_t` | Example temperature multiplied by 100 |
| voltage_mv | `uint16_t` | Example voltage in millivolts |
| led_state | `uint8_t` | Current LED state |

The event payload is binary:

| Field | Type | Meaning |
|---|---|---|
| event_id | `uint8_t` | Event identifier |
| uptime_ms | `uint32_t` | Pico uptime in milliseconds |
| detail | `uint16_t` | Event-specific detail value |

## Testing Methodology

The repository includes Python unit tests that validate the protocol logic
without requiring hardware. They use an injected fake serial object to simulate
the USB byte stream.

Run tests:

```bash
python3 -m unittest discover -s tests
```

Included tests:

1. **Normal telemetry packet send/receive** - builds a TELEMETRY frame, receives
   it, validates fields, and checks that an ACK is sent.
2. **Urgent event priority test** - queues TELEMETRY and EVENT together and
   verifies that EVENT is transmitted first.
3. **Corrupted packet handling test** - sends a bad CRC packet followed by a
   valid packet, verifies NACK generation, and confirms resynchronisation.
4. **Large payload test** - checks that payloads larger than 64 bytes are
   rejected before bytes are written to serial.

For hardware testing, connect the Pico, run the Python host, and confirm that:

- CONNECT completes.
- A startup EVENT is printed.
- TELEMETRY arrives once per second.
- `--led on` and `--led off` cause EVENT responses from the Pico.
- Unplugging or pausing the host causes timeout behaviour instead of a permanent
  block.

## Limitations

- The sequence number is 8-bit, so it wraps after 255 packets.
- Payload size is fixed at 64 bytes to keep Pico memory use low.
- The reliability model is stop-and-wait per packet, not a sliding window.
- The priority scheme is queue-based; it does not interrupt a packet already
  being transmitted.
- USB CDC is usually reliable on a short cable, so corruption tests are mostly
  simulated. The protocol is still useful because it makes framing and recovery
  explicit.

## Future Improvements

- Add a protocol version field for future compatibility.
- Add selective retransmission or a sliding window for higher throughput.
- Add optional payload compression for repeated telemetry structures.
- Add a timestamp field to the common header.
- Add message authentication for hostile environments.
- Add separate queue depths for EVENT and control traffic.
- Generate protocol documentation automatically from shared constants.

## Academic Summary

The Priority Event + Telemetry Protocol demonstrates how a microcontroller can
turn a plain serial byte stream into a structured communication channel. The
design is lightweight, deterministic, and practical for the Raspberry Pi Pico.
It adds framing, integrity checking, sequencing, acknowledgements, negative
acknowledgements, timeout recovery, and priority scheduling while keeping the
implementation small enough for an embedded project.
