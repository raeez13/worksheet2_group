import time
import unittest

from custom_protocol import (
    CustomProtocol,
    MAX_PAYLOAD,
    NACK_CHECKSUM,
    PKT_ACK,
    PKT_EVENT,
    PKT_NACK,
    PKT_TELEMETRY,
    PRIORITY_HIGH,
    PRIORITY_NORMAL,
)


class ScriptedSerial:
    def __init__(self, incoming=b""):
        self.incoming = bytearray(incoming)
        self.written = bytearray()
        self.timeout = 0.001
        self.closed = False

    def read(self, size=1):
        if not self.incoming:
            time.sleep(self.timeout)
            return b""

        take = min(size, len(self.incoming))
        data = bytes(self.incoming[:take])
        del self.incoming[:take]
        return data

    def write(self, data):
        self.written.extend(data)
        return len(data)

    def flush(self):
        pass

    def close(self):
        self.closed = True

    def reset_input_buffer(self):
        self.incoming.clear()

    def append(self, data):
        self.incoming.extend(data)


def build_frame(packet_type, priority, sequence, payload=b""):
    protocol = CustomProtocol(serial_instance=ScriptedSerial())
    return protocol._build_packet(packet_type, priority, sequence, payload)


def decode_frames(data):
    serial = ScriptedSerial(data)
    protocol = CustomProtocol(serial_instance=serial)
    frames = []
    while serial.incoming:
        frames.append(protocol._read_packet_raw(0.1))
    return frames


class CustomProtocolTests(unittest.TestCase):
    def test_normal_telemetry_packet_receive_sends_ack(self):
        payload = bytes([1, 2, 3, 4])
        frame = build_frame(PKT_TELEMETRY, PRIORITY_NORMAL, 0, payload)
        fake_serial = ScriptedSerial(frame)
        protocol = CustomProtocol(serial_instance=fake_serial)
        protocol.connected = True

        packet = protocol.receive(timeout=0.2)

        self.assertEqual(packet.packet_type, PKT_TELEMETRY)
        self.assertEqual(packet.priority, PRIORITY_NORMAL)
        self.assertEqual(packet.sequence, 0)
        self.assertEqual(packet.payload, payload)

        ack = decode_frames(bytes(fake_serial.written))[0]
        self.assertEqual(ack.packet_type, PKT_ACK)
        self.assertEqual(ack.sequence, 0)

    def test_urgent_event_is_sent_before_normal_telemetry(self):
        fake_serial = ScriptedSerial()
        protocol = CustomProtocol(serial_instance=fake_serial)
        protocol.connected = True

        protocol._enqueue_packet(PKT_TELEMETRY, b"T")
        protocol._enqueue_packet(PKT_EVENT, b"E")
        protocol._flush_outgoing(require_ack=False)

        frames = decode_frames(bytes(fake_serial.written))
        self.assertEqual([frame.packet_type for frame in frames], [PKT_EVENT, PKT_TELEMETRY])
        self.assertEqual([frame.priority for frame in frames], [PRIORITY_HIGH, PRIORITY_NORMAL])
        self.assertEqual([frame.sequence for frame in frames], [0, 1])

    def test_corrupted_packet_is_nacked_then_receiver_resynchronises(self):
        good_frame = build_frame(PKT_EVENT, PRIORITY_HIGH, 0, b"\x01")
        corrupt_frame = bytearray(good_frame)
        corrupt_frame[-1] ^= 0xFF
        stream = bytes(corrupt_frame) + good_frame

        fake_serial = ScriptedSerial(stream)
        protocol = CustomProtocol(serial_instance=fake_serial)
        protocol.connected = True

        packet = protocol.receive(timeout=0.2)

        self.assertEqual(packet.packet_type, PKT_EVENT)
        self.assertEqual(packet.sequence, 0)

        responses = decode_frames(bytes(fake_serial.written))
        self.assertEqual(responses[0].packet_type, PKT_NACK)
        self.assertEqual(responses[0].payload[2], NACK_CHECKSUM)
        self.assertEqual(responses[1].packet_type, PKT_ACK)

    def test_large_payload_is_rejected_before_serial_write(self):
        fake_serial = ScriptedSerial()
        protocol = CustomProtocol(serial_instance=fake_serial)
        protocol.connected = True

        with self.assertRaises(Exception):
            protocol.send(PKT_TELEMETRY, bytes(MAX_PAYLOAD + 1))

        self.assertEqual(bytes(fake_serial.written), b"")


if __name__ == "__main__":
    unittest.main()
