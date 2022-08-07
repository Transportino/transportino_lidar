import math
from typing import List, Callable
import serial
import time
from dataclasses import dataclass
from enum import Enum


def serial_read(ser : serial.Serial, byte_count : int):
    data = ser.read(byte_count)

    if len(data) != byte_count:
        print("Data size doesn't match desired byte count")
        return None

    return data

class LidarState(Enum):
    SYNC0 = 0
    SYNC1 = 1
    HEADER = 2
    DATA = 3

@dataclass
class Packet:
    type : int
    data_length : int
    start_angle : int
    end_angle : int
    angle_per_sample : float
    start_read_time : float


@dataclass
class ScanPoint:
    range : float
    angle : float
    intensity : int

class LidarModule:

    __packet : Packet
    __serial : serial.Serial
    __scan_points : List[ScanPoint]
    __on_scan_callback : Callable[[List[ScanPoint], float], None]

    def __init__(self, serial_port : str, on_scan_callback : Callable[[List[ScanPoint], float], None]):
        self.__on_scan_callback = on_scan_callback
        self.__packet = Packet(0, 0, 0, 0, 0, time.time())
        self.__scan_points = []
        try:
            self.__serial = serial.Serial(serial_port, 153600, timeout=0.1)
            time.sleep(1)
        except:
            print("Couldn't open lidar module serial port.")
            exit(1)
    
    def __del__(self):
        self.__serial.close()

    def update_data(self):
        self.__packet.start_read_time = time.time()
        run = True
        state = LidarState.SYNC0
        try:
            
            while run:
                
                if state == LidarState.SYNC0:

                    data = serial_read(self.__serial, 1)

                    if data == None:
                        continue

                    if data[0] == 0xAA:
                        state = LidarState.SYNC1
                        continue

                elif state == LidarState.SYNC1:
                    data = serial_read(self.__serial, 1)

                    if data == None:
                        continue

                    if data[0] == 0x55:
                        state = LidarState.HEADER
                    else:
                        state = LidarState.SYNC0
                    continue
                elif state == LidarState.HEADER:
                    data = serial_read(self.__serial, 8)

                    if data == None:
                        continue

                    self.__packet.type = data[0]
                    self.__packet.data_length = int(data[1])
                    self.__packet.start_angle = int(data[3] << 8) + int(data[2])
                    self.__packet.end_angle = int(data[5] << 8) + int(data[4])
                    
                    diff_angle = self.__packet.end_angle - self.__packet.start_angle

                    if self.__packet.end_angle < self.__packet.start_angle:
                        diff_angle = 0xB400 - self.__packet.start_angle + self.__packet.end_angle

                    self.__packet.angle_per_sample = 0

                    if diff_angle > 1 and (self.__packet.data_length - 1) > 0:
                        self.__packet.angle_per_sample = diff_angle / (self.__packet.data_length - 1)

                    # Only for debug
                    #print("type: 0x%02x,\tdata_length: %d,\tstart_angle: %d,\tend_angle: %d,\tdiff:%d" % (self.packet.type, self.packet.data_length, self.packet.start_angle, self.packet.end_angle, diff_angle))

                    state = LidarState.DATA

                    continue
                elif state == LidarState.DATA:
                    state = LidarState.SYNC0

                    data = serial_read(self.__serial, self.__packet.data_length * 3)

                    if data == None:
                        break

                    for i in range(0, self.__packet.data_length):

                        scan_point = ScanPoint(0, 0, 0)
                        
                        scan_point.intensity = int(data[(i * 3) + 0])

                        rangeL = int(data[(i * 3) + 1])
                        rangeH = int(data[(i * 3) + 2])

                        raw_range = (rangeH << 8) + rangeL

                        scan_point.range = float(raw_range) / 4000.0

                        raw_angle = self.__packet.start_angle + (self.__packet.angle_per_sample * i)
                        scan_point.angle = (raw_angle / 0xB400) * (2*math.pi)

                        self.__scan_points.append(scan_point)

                    if self.__packet.type != 0x38:
                        if self.__on_scan_callback != None:
                            self.__on_scan_callback(self.__scan_points, (time.time() - self.__packet.start_read_time))
                        self.__scan_points.clear()
                        self.__serial.reset_input_buffer()
                        run = False
                    
                    

        except serial.SerialException:
            print('Lost connection with lidar module')
            exit(1)

"""
Used this only for debug purposes

if __name__ == '__main__':
    lidar = LidarModule('/dev/ttyUSB0', None)

    while True:
        lidar.update_data()
        for point in lidar.scan_points:
            print('range: %0.2f,\tangle:%0.2f,\tintensity:%d' % (point.range, point.angle, point.intensity))

"""