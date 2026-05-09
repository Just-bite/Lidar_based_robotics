import time
import serial
import numpy as np

GLOBAL_BAUD_RATE = 230400
GLOBAL_TIMEOUT = 1

PACKETS_PER_TURN = 16
DOTS_PER_PACKET = 80  # heuristic, specification says 52 but if so memory leak caused

PACKET_HEAD = 0xAA
PACKET_PAYLOAD_INDICATOR = 0xAD
PROTOCOL_VERSION = 0x01  # 0x00 - error
MIN_SIGNAL_LEVEL = 0x20  # still changeable


class Lidar:

    def __init__(self, port, baud_rate=GLOBAL_BAUD_RATE, timeout=GLOBAL_TIMEOUT):
        self.serial = serial.Serial(port, baud_rate, timeout=timeout)
        self.data = np.zeros((PACKETS_PER_TURN * DOTS_PER_PACKET, 2))
        self.sector = 0
        self.size = 0

    def close(self):
        if self.serial and self.serial.is_open: self.serial.close()

    def extract_packet(self, data):
        """Return index of a next packet and fills data array with [distance,angle]"""

        for index, byte in enumerate(data):
            if byte == PACKET_HEAD:
                if len(data) < index + 3: return 0
                pack = data[index:]
                packet_len = (pack[1] << 8) + pack[2]

                if len(pack) < packet_len + 2: return 0
                if pack[3] != PROTOCOL_VERSION: return index + 1
                if pack[5] != PACKET_PAYLOAD_INDICATOR: return index + 1

                payload_len = (pack[6] << 8) + pack[7]
                zero_angle = ((pack[11] << 8) + pack[12]) * 0.01

                crc = (pack[packet_len] << 8) + pack[packet_len + 1]
                if crc != sum(pack[:packet_len]) & 0xFFFF: return index + 1

                sample_amount = (payload_len - 5) // 3
                angular_step = 360 / (PACKETS_PER_TURN * sample_amount)

                for i in range(sample_amount):
                    signal_level = pack[13 + 3 * i]
                    if signal_level < MIN_SIGNAL_LEVEL: continue
                    angle = zero_angle + angular_step * i
                    distance = ((pack[14 + 3 * i] << 8) + pack[14 + 3 * i + 1]) * 0.25
                    idx = self.sector * sample_amount + i
                    if idx < self.data.shape[0]: self.data[idx] = [distance, angle]

                self.sector += 1
                self.size += sample_amount

                if self.sector >= PACKETS_PER_TURN:
                    self.sector = 0
                    return -(index + packet_len + 2)  # Flag of a turn end

                return index + packet_len + 2
        return 0

    def process_data(self):
        """Extracts packets from serial port if any"""

        self.data.fill(0)
        self.size = 0
        self.sector = 0
        raw_buffer = b''
        while True:
            waiting = self.serial.in_waiting
            if waiting == 0:
                time.sleep(0.001)
                continue

            raw_buffer += self.serial.read(waiting)

            while len(raw_buffer) > 0:
                cut = self.extract_packet(raw_buffer)
                if not cut: break
                if cut < 0: return
                raw_buffer = raw_buffer[cut:]
