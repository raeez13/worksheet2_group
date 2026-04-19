#!/usr/bin/env python3
"""Host implementation of the Priority Event + Telemetry Protocol.

The code uses PySerial for the real USB CDC link. The serial object can also be
injected for unit tests, which lets the binary framing and priority behaviour be
tested without a Raspberry Pi Pico attached.
"""

from __future__ import annotations

import argparse
import struct
import time
from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional

try:
    import serial
except ImportError:  # pragma: no cover - exercised only on systems without pyserial
    serial = None


MAGIC = b"\x50\x45"
MAX_PAYLOAD = 64
ACK_TIMEOUT = 0.3
MAX_RETRIES = 2

PKT_CONNECT = 0x01
PKT_CONNECT_ACK = 0x02
PKT_TELEMETRY = 0x03
PKT_EVENT = 0x04
PKT_ACK = 0x05
PKT_NACK = 0x06
PKT_DISCONNECT = 0x07
PKT_COMMAND = 0x08

PRIORITY_NORMAL = 0x01
PRIORITY_HIGH = 0x02
PRIORITY_CONTROL = 0x03

NACK_CHECKSUM = 0x01
NACK_LENGTH = 0x02
NACK_SEQUENCE = 0x03
NACK_UNSUPPORTED = 0x04

DATA_TYPES = {PKT_TELEMETRY, PKT_EVENT, PKT_COMMAND, PKT_DISCONNECT}
NO_ACK_TYPES = {PKT_ACK, PKT_NACK, PKT_CONNECT_ACK}


class ProtocolError(Exception):
    """Base class for protocol failures."""


class ProtocolTimeoutError(ProtocolError):
    """Raised when a complete packet or acknowledgement is not received."""


class ProtocolChecksumError(ProtocolError):
    """Raised when a packet CRC-8 does not match."""

    def __init__(self, sequence: int, expected: int, received: int):
        super().__init__(
            f"CRC mismatch for sequence {sequence}: expected 0x{expected:02x}, "
            f"received 0x{received:02x}"
        )
        self.sequence = sequence
        self.expected = expected
        self.received = received


class ProtocolLengthError(ProtocolError):
    """Raised when a packet advertises a payload larger than MAX_PAYLOAD."""

    def __init__(self, sequence: int, length: int):
        super().__init__(f"invalid payload length {length} for sequence {sequence}")
        self.sequence = sequence
        self.length = length


class ProtocolSequenceError(ProtocolError):
    """Raised when the received sequence does not match the expected value."""

    def __init__(self, received: int, expected: int):
        super().__init__(
            f"sequence mismatch: received {received}, expected {expected}"
        )
        self.received = received
        self.expected = expected


@dataclass(frozen=True)
class Packet:
    packet_type: int
    priority: int
    sequence: int
    payload: bytes

    @property
    def length(self) -> int:
        return len(self.payload)


@dataclass
class _OutgoingPacket:
    packet_type: int
    priority: int
    payload: bytes


class CustomProtocol:
    def __init__(
        self,
        port: Optional[str] = None,
        baudrate: int = 115200,
        timeout: float = 0.05,
        serial_instance=None,
        ack_timeout: float = ACK_TIMEOUT,
        max_retries: int = MAX_RETRIES,
    ):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ack_timeout = ack_timeout
        self.max_retries = max_retries

        if serial_instance is not None:
            self.serial = serial_instance
        else:
            if serial is None:
                raise RuntimeError("pyserial is required: pip install pyserial")
            if port is None:
                raise ValueError("port is required when serial_instance is not used")
            self.serial = serial.Serial(
                port=port,
                baudrate=baudrate,
                timeout=timeout,
                write_timeout=timeout,
            )

        self.next_tx_sequence = 0
        self.expected_rx_sequence = 0
        self.connected = False
        self._high_queue: Deque[_OutgoingPacket] = deque()
        self._normal_queue: Deque[_OutgoingPacket] = deque()
        self._inbox: Deque[Packet] = deque()

    def connect(self, timeout: float = 5.0) -> bool:
        """Open the logical protocol session with the Pico."""
        if hasattr(self.serial, "reset_input_buffer"):
            self.serial.reset_input_buffer()

        self.next_tx_sequence = 0
        self.expected_rx_sequence = 0

        deadline = time.monotonic() + timeout
        next_hello = 0.0

        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_hello:
                self._write_packet(PKT_CONNECT, PRIORITY_CONTROL, 0, b"HOST")
                next_hello = now + 0.5

            try:
                packet = self._read_packet_with_error_handling(0.05)
            except ProtocolTimeoutError:
                continue

            if packet.packet_type == PKT_CONNECT_ACK:
                self.connected = True
                return True

            if packet.packet_type == PKT_CONNECT:
                self._handle_incoming_packet(packet, queue_for_app=False)
                self.connected = True
                return True

        return False

    def send(
        self,
        packet_type: int,
        payload: bytes | bytearray | list[int] = b"",
        priority: Optional[int] = None,
        require_ack: bool = True,
    ) -> int:
        """Queue and transmit one packet, returning its transmitted sequence."""
        payload_bytes = bytes(payload)
        if len(payload_bytes) > MAX_PAYLOAD:
            raise ProtocolLengthError(self.next_tx_sequence, len(payload_bytes))

        if not self.connected and packet_type not in {PKT_CONNECT, PKT_CONNECT_ACK}:
            raise ProtocolError("protocol is not connected")

        chosen_priority = priority if priority is not None else self._priority_for_type(packet_type)
        self._enqueue_packet(packet_type, payload_bytes, chosen_priority)
        return self._flush_outgoing(require_ack=require_ack)

    def receive(self, timeout: float = 1.0) -> Packet:
        """Receive the next application packet and ACK/NACK it automatically."""
        if self._inbox:
            return self._inbox.popleft()

        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:
            remaining = max(0.001, deadline - time.monotonic())
            try:
                packet = self._read_packet_with_error_handling(remaining)
                accepted = self._handle_incoming_packet(packet, queue_for_app=False)
            except ProtocolSequenceError:
                continue

            if accepted is None:
                continue
            if accepted.packet_type in {PKT_ACK, PKT_NACK}:
                return accepted
            return accepted

        raise ProtocolTimeoutError("receive timed out")

    def disconnect(self) -> None:
        if self.connected:
            try:
                self.send(PKT_DISCONNECT, b"")
            finally:
                self.connected = False

    def cleanup(self) -> None:
        if hasattr(self.serial, "close"):
            self.serial.close()

    def _priority_for_type(self, packet_type: int) -> int:
        if packet_type == PKT_TELEMETRY:
            return PRIORITY_NORMAL
        if packet_type == PKT_EVENT:
            return PRIORITY_HIGH
        return PRIORITY_CONTROL

    def _take_next_sequence(self) -> int:
        sequence = self.next_tx_sequence
        self.next_tx_sequence = (self.next_tx_sequence + 1) & 0xFF
        return sequence

    def _enqueue_packet(
        self, packet_type: int, payload: bytes, priority: Optional[int] = None
    ) -> None:
        chosen_priority = priority if priority is not None else self._priority_for_type(packet_type)
        item = _OutgoingPacket(packet_type, chosen_priority, payload)
        if chosen_priority == PRIORITY_NORMAL:
            self._normal_queue.append(item)
        else:
            self._high_queue.append(item)

    def _pop_next_outgoing(self) -> Optional[_OutgoingPacket]:
        if self._high_queue:
            return self._high_queue.popleft()
        if self._normal_queue:
            return self._normal_queue.popleft()
        return None

    def _flush_outgoing(self, require_ack: bool = True) -> int:
        last_sequence = -1

        while True:
            item = self._pop_next_outgoing()
            if item is None:
                break

            sequence = self._take_next_sequence()
            last_sequence = sequence

            for attempt in range(self.max_retries + 1):
                self._write_packet(
                    item.packet_type,
                    item.priority,
                    sequence,
                    item.payload,
                )

                if not require_ack or item.packet_type in NO_ACK_TYPES:
                    break

                try:
                    self._wait_for_ack(sequence, self.ack_timeout)
                    break
                except ProtocolTimeoutError:
                    if attempt >= self.max_retries:
                        raise
                except ProtocolError:
                    if attempt >= self.max_retries:
                        raise
            else:
                raise ProtocolError(f"failed to transmit sequence {sequence}")

        return last_sequence

    def _wait_for_ack(self, sequence: int, timeout: float) -> None:
        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:
            remaining = max(0.001, deadline - time.monotonic())
            packet = self._read_packet_with_error_handling(remaining)

            if packet.packet_type == PKT_ACK and packet.sequence == sequence:
                return

            if packet.packet_type == PKT_NACK and packet.sequence == sequence:
                raise ProtocolError(f"NACK received for sequence {sequence}")

            try:
                accepted = self._handle_incoming_packet(packet, queue_for_app=True)
            except ProtocolSequenceError:
                continue

            if accepted is not None and accepted.packet_type not in {PKT_ACK, PKT_NACK}:
                self._inbox.append(accepted)

        raise ProtocolTimeoutError(f"ACK timeout for sequence {sequence}")

    def _handle_incoming_packet(
        self, packet: Packet, queue_for_app: bool = False
    ) -> Optional[Packet]:
        if packet.packet_type == PKT_CONNECT:
            self.expected_rx_sequence = 0
            self.next_tx_sequence = 0
            self.connected = True
            self._send_connect_ack(packet.sequence)
            return None

        if packet.packet_type == PKT_CONNECT_ACK:
            self.connected = True
            return None

        if packet.packet_type in {PKT_ACK, PKT_NACK}:
            return packet

        if packet.packet_type not in DATA_TYPES:
            self._send_nack(packet.sequence, NACK_UNSUPPORTED)
            raise ProtocolError(f"unsupported packet type 0x{packet.packet_type:02x}")

        if packet.sequence != self.expected_rx_sequence:
            self._send_nack(packet.sequence, NACK_SEQUENCE)
            raise ProtocolSequenceError(packet.sequence, self.expected_rx_sequence)

        self._send_ack(packet.sequence)
        self.expected_rx_sequence = (self.expected_rx_sequence + 1) & 0xFF

        if packet.packet_type == PKT_DISCONNECT:
            self.connected = False

        if queue_for_app:
            self._inbox.append(packet)
            return None

        return packet

    def _read_packet_with_error_handling(self, timeout: float) -> Packet:
        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:
            remaining = max(0.001, deadline - time.monotonic())
            try:
                return self._read_packet_raw(remaining)
            except ProtocolLengthError as exc:
                self._send_nack(exc.sequence, NACK_LENGTH)
            except ProtocolChecksumError as exc:
                self._send_nack(exc.sequence, NACK_CHECKSUM)

        raise ProtocolTimeoutError("packet timeout")

    def _read_packet_raw(self, timeout: float) -> Packet:
        deadline = time.monotonic() + timeout
        matched = 0

        while time.monotonic() < deadline:
            byte = self._read_exact(1, deadline)
            value = byte[0]

            if matched == 0:
                if value == MAGIC[0]:
                    matched = 1
            else:
                if value == MAGIC[1]:
                    break
                matched = 1 if value == MAGIC[0] else 0
        else:
            raise ProtocolTimeoutError("magic bytes not found")

        header = self._read_exact(5, deadline)
        packet_type = header[0]
        priority = header[1]
        sequence = header[2]
        length = header[3] | (header[4] << 8)

        if length > MAX_PAYLOAD:
            raise ProtocolLengthError(sequence, length)

        payload = self._read_exact(length, deadline)
        received_crc = self._read_exact(1, deadline)[0]
        expected_crc = self._crc8(header + payload)

        if received_crc != expected_crc:
            raise ProtocolChecksumError(sequence, expected_crc, received_crc)

        return Packet(packet_type, priority, sequence, payload)

    def _read_exact(self, size: int, deadline: float) -> bytes:
        data = bytearray()

        while len(data) < size:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise ProtocolTimeoutError("short serial read")

            if hasattr(self.serial, "timeout"):
                self.serial.timeout = min(max(remaining, 0.001), self.timeout)

            chunk = self.serial.read(size - len(data))
            if chunk:
                data.extend(chunk)
            else:
                time.sleep(0.001)

        return bytes(data)

    def _write_packet(
        self, packet_type: int, priority: int, sequence: int, payload: bytes
    ) -> None:
        frame = self._build_packet(packet_type, priority, sequence, payload)
        self.serial.write(frame)
        if hasattr(self.serial, "flush"):
            self.serial.flush()

    def _send_ack(self, sequence: int) -> None:
        self._write_packet(PKT_ACK, PRIORITY_CONTROL, sequence, bytes([sequence]))

    def _send_nack(self, sequence: int, reason: int) -> None:
        payload = bytes([sequence, self.expected_rx_sequence, reason])
        self._write_packet(PKT_NACK, PRIORITY_CONTROL, sequence, payload)

    def _send_connect_ack(self, sequence: int) -> None:
        self._write_packet(PKT_CONNECT_ACK, PRIORITY_CONTROL, sequence, b"OK")

    def _build_packet(
        self, packet_type: int, priority: int, sequence: int, payload: bytes
    ) -> bytes:
        if len(payload) > MAX_PAYLOAD:
            raise ProtocolLengthError(sequence, len(payload))

        header = bytes(
            [
                packet_type & 0xFF,
                priority & 0xFF,
                sequence & 0xFF,
                len(payload) & 0xFF,
                (len(payload) >> 8) & 0xFF,
            ]
        )
        return MAGIC + header + payload + bytes([self._crc8(header + payload)])

    @staticmethod
    def _crc8(data: bytes) -> int:
        crc = 0x00
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = ((crc << 1) ^ 0x07) & 0xFF
                else:
                    crc = (crc << 1) & 0xFF
        return crc


def packet_type_name(packet_type: int) -> str:
    names = {
        PKT_CONNECT: "CONNECT",
        PKT_CONNECT_ACK: "CONNECT_ACK",
        PKT_TELEMETRY: "TELEMETRY",
        PKT_EVENT: "EVENT",
        PKT_ACK: "ACK",
        PKT_NACK: "NACK",
        PKT_DISCONNECT: "DISCONNECT",
        PKT_COMMAND: "COMMAND",
    }
    return names.get(packet_type, f"UNKNOWN_0x{packet_type:02x}")


def describe_packet(packet: Packet) -> str:
    if packet.packet_type == PKT_TELEMETRY and packet.length == struct.calcsize("<IHhHB"):
        uptime_ms, sample_id, temp_x100, voltage_mv, led_state = struct.unpack(
            "<IHhHB", packet.payload
        )
        return (
            f"TELEMETRY seq={packet.sequence} uptime={uptime_ms}ms "
            f"sample={sample_id} temp={temp_x100 / 100:.2f}C "
            f"voltage={voltage_mv}mV led={led_state}"
        )

    if packet.packet_type == PKT_EVENT and packet.length == struct.calcsize("<BIH"):
        event_id, uptime_ms, detail = struct.unpack("<BIH", packet.payload)
        event_names = {
            0x01: "STARTUP",
            0x02: "LED_OR_ERROR",
            0x03: "PRIORITY_TEST",
            0x04: "PING",
        }
        event_name = event_names.get(event_id, "UNKNOWN")
        return (
            f"EVENT seq={packet.sequence} id=0x{event_id:02x} name={event_name} "
            f"uptime={uptime_ms}ms detail=0x{detail:04x}"
        )

    return (
        f"{packet_type_name(packet.packet_type)} seq={packet.sequence} "
        f"priority={packet.priority} payload={packet.payload.hex()}"
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Priority Event + Telemetry Protocol host"
    )
    parser.add_argument("port", help="Serial port, for example /dev/ttyACM0 or COM5")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--led", choices=["on", "off"], help="Send LED command after connect")
    args = parser.parse_args()

    protocol = CustomProtocol(args.port, baudrate=args.baudrate)
    try:
        if not protocol.connect(timeout=args.timeout):
            raise ProtocolTimeoutError("Pico did not complete CONNECT handshake")

        print("Connected to Pico")

        if args.led is not None:
            led_value = 1 if args.led == "on" else 0
            protocol.send(PKT_COMMAND, bytes([0x01, led_value]))

        protocol.send(PKT_COMMAND, bytes([0x02]))

        while True:
            try:
                packet = protocol.receive(timeout=2.0)
                print(describe_packet(packet))
            except ProtocolTimeoutError:
                print("No packet received within 2 seconds")
    except KeyboardInterrupt:
        print("\nDisconnecting")
    finally:
        try:
            protocol.disconnect()
        finally:
            protocol.cleanup()


if __name__ == "__main__":
    main()
