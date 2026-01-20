import serial
import struct
import time
import numpy as np
from serial.tools import list_ports


class spikeActivation():

    def __init__(self, frame_size=64, count_win=1000):
        self.ser = None
        self.frame_size = frame_size
        self.count_win = count_win
        self.collect = np.empty((frame_size, 0), dtype=np.uint16)
    
    def find_available_ports(self):
        """
        查找所有可用的COM端口
        """
        ports = []
        for port in list_ports.comports():
            ports.append(port.device)
            print(f"找到COM端口: {port.device} - {port.description}")
        return ports

    def write_binary(self, ser, data: bytes):
        """
        将二进制数据分段写入 CDC，每次最多 64 字节，并立即 flush
        """
        for offset in range(0, len(data), 64):
            chunk = data[offset:offset + 64]
            ser.write(chunk)
            ser.flush()

    def read_exact(self, ser, size: int, timeout: float) -> bytes:
        """
        从 CDC 中读取正好 size 字节，超时抛出 TimeoutError
        """
        deadline = time.time() + timeout
        buf = b''
        while len(buf) < size and time.time() < deadline:
            chunk = ser.read(size - len(buf))
            if chunk:
                buf += chunk
        if len(buf) < size:
            raise TimeoutError(f"Timeout: expected {size} bytes, got {len(buf)} bytes")
        return buf

    def send_handshake(self, ser, frame_size: int, count_win_us: int):
        # pack 两个 uint32（小端）共 8 字节
        data = struct.pack('<II', frame_size, count_win_us)
        self.write_binary(ser, data)

    def recv_handshake(self, ser, timeout=2.0):
        data = self.read_exact(ser, 8, timeout)
        frame_size, count_win_us = struct.unpack('<II', data)
        return frame_size, count_win_us
    
    def init(self, com_port='COM4', max_retries=3):
        """
        初始化串口连接
        :param com_port: 要尝试连接的COM端口 (默认 COM4)
        :param max_retries: 最大重试次数 (默认 3)
        """
        retry_count = 0
        
        while retry_count < max_retries:
            self.ser = None
            try:
                print(f"\n[尝试 {retry_count + 1}/{max_retries}] 连接到 {com_port}...")
                self.ser = serial.Serial(
                    port=com_port,
                    baudrate=115200,
                    timeout=0.01,
                    write_timeout=2
                )

                print("→ 发送握手信息:", self.frame_size, self.count_win)
                self.send_handshake(self.ser, self.frame_size, self.count_win)

                fs, cw = self.recv_handshake(self.ser, timeout=2.0)
                print("← 接收到ACK:", fs, cw)
                return 1

            except serial.serialutil.SerialException as e:
                print(f"❌ 串口错误: {e}")
                if self.ser and self.ser.is_open:
                    self.ser.close()
                
                retry_count += 1
                if retry_count < max_retries:
                    print(f"   3秒后重试...")
                    time.sleep(3)
                else:
                    print(f"\n❌ 达到最大重试次数({max_retries})")
                    print("\n📋 可用的COM端口:")
                    available_ports = self.find_available_ports()
                    if not available_ports:
                        print("   没有找到任何COM端口!")
                    return 0

            except TimeoutError as e:
                print("❌ 握手失败:", e)
                if self.ser and self.ser.is_open:
                    self.ser.close()
                return 0

    def activate(self, x):
        if self.ser and self.ser.is_open:
            
            try:
                # 小端打包成二进制块
                x_list = x.flatten().tolist()
                payload = struct.pack('<' + 'H' * self.frame_size, *x_list)
                self.write_binary(self.ser, payload)
                # 读取 MCU 回传的 frame_size * 2 字节
                reply = self.read_exact(self.ser, self.frame_size * 2, timeout=1)
                # 解包成 uint16 列表
                recv_vals = list(struct.unpack('<' + 'H' * self.frame_size, reply))
                # recv_vals = (x//100).flatten().tolist()
                print("← Received counts:", recv_vals)
                self.collect = np.hstack((self.collect, np.array(recv_vals).reshape(self.frame_size, 1)))
                return np.array(recv_vals, dtype=np.uint16).reshape(self.frame_size, 1)

            except KeyboardInterrupt:
                print("\nInterrupted by user")
                self.activate(np.zeros((self.frame_size, 1), dtype=np.uint16))
                if self.ser and self.ser.is_open:
                    self.ser.close()

    def deinit(self,):
        self.activate(np.zeros((self.frame_size, 1), dtype=np.uint16))
        if self.ser and self.ser.is_open:
            self.ser.close()
        np.savetxt('spike_counts.txt', self.collect)
        print("Output reset. Serial closed.")


if __name__ == '__main__':
    spike = spikeActivation(frame_size=100, count_win=1000)
    spike.init()
    for i in range(1000):
        spike.activate(np.ones((100, 1), dtype=np.uint16)*int(65535/7.5*7.5))
    spike.deinit()
