# parser.py — 遥测帧流式解析: 对抗粘包/半包/假帧头, 任意切分结果一致
# 帧结构: AA 55 | TYPE | LEN | SEQ | PAYLOAD | CRC16(低字节在前)
import struct

HEADER = b"\xAA\x55"

def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc

class TelemParser:
    def __init__(self):
        self.buf = bytearray()
        self.bad_frames = 0          # 误码/假帧头计数 -> 线缆质量指标

    def feed(self, data: bytes):
        """喂入任意长度的串口字节, 产出 (theta_deg, thdot_rads, v_ms, u_Nm, bias_dps) 元组"""
        self.buf += data
        while True:
            i = self.buf.find(HEADER)
            if i < 0:
                del self.buf[:-1]    # 保留最后1字节(可能是帧头一半)
                return
            if i:
                del self.buf[:i]
            if len(self.buf) < 6:
                return               # 帧头+TYPE+LEN+SEQ+CRC 未齐
            typ, ln = self.buf[2], self.buf[3]
            total = 4 + ln + 2
            if ln > 240:
                del self.buf[:1]     # 非法长度: 假帧头, 滑1字节重扫
                continue
            if len(self.buf) < total:
                return               # 半包: 等下次数据
            frame = bytes(self.buf[:total])
            del self.buf[:total]
            if typ != 0x01 or ln != 20:
                self.bad_frames += 1
                continue             # 假帧头: 丢弃重扫
            payload = frame[4:4 + ln]
            crc_recv = frame[4 + ln] | (frame[5 + ln] << 8)
            if crc16(frame[2:4 + ln]) != crc_recv:
                self.bad_frames += 1
                continue             # 误码帧: 丢弃并计数
            th, thd, v, u, bias = struct.unpack("<5f", payload)
            yield th, thd, v, u, bias
