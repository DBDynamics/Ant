#!/usr/bin/python3

import ctypes
import queue
import struct
import threading
import time

# import numpy
import serial


# Class for basic communication
# Each Stepper Group consist of 128 motors
class Ant:
    # Communication Profiles, do not change them!
    _INDEX_CONTROL_WORD = 0
    _INDEX_OPERATION_MODE = 1
    _INDEX_IO_OUT = 14
    _INDEX_MEMORY = 30
    _INDEX_DEVICE_ID = 31
    _INDEX_RUNNING_CURRENT = 40
    _INDEX_KEEPING_CURRENT = 42
    _INDEX_HOMING_DIRECTION = 45
    _INDEX_HOMING_LEVEL = 46
    _INDEX_ACC_TIME = 47
    _INDEX_TARGET_VELOCITY = 48
    _INDEX_TARGET_POSITION = 49
    _INDEX_ACTUAL_VELOCITY = 52
    _INDEX_ACTUAL_POSITION = 53
    _INDEX_IO_INPUT = 54
    _SUBINDEX_WRITE = 0
    _SUBINDEX_READ = 1
    _OPERATION_MODE_PROFILE_VELOCITY = 21
    _OPERATION_MODE_PROFILE_POSITION = 31
    _OPERATION_MODE_HOMING = 40
    _FUNC_CODE_TSDO = 0X580
    _FUNC_CODE_FREE = 0X780
    _FUNC_CODE_SYNC = 0X080

    # # Variables
    # _connection = 0
    # # Motors consist of 128 motor cells, each motor has 1024 parameters
    # _motors = numpy.zeros((128, 1024), dtype=numpy.int32)
    # _thread1 = 0
    # _thread_stop_flag = 0
    # _tx_queue = queue.Queue()
    # _tx_lock = threading.Lock()
    # _rx_lock = threading.Lock()

    # Connect to the serial port and establish the communication
    def _connect(self):
        self._connection = serial.Serial(self._portName, 1500000, timeout=0.1)

    # Disconnect form the serial port and close the communication
    def _disconnect(self):
        self._thread_stop_flag = 1
        self._connection.close()

    # Analysis the Rx Message and Put the parameters into Each Motor
    def _analysis(self, rx_message):
        if len(rx_message) == 11:
            self._rx_lock.acquire()
            id = struct.unpack_from('h', rx_message, 0)
            index = struct.unpack_from('h', rx_message, 2)
            subindex = struct.unpack_from('h', rx_message, 4)
            data = struct.unpack_from('i', rx_message, 6)
            size = struct.unpack_from('B', rx_message, 10)
            func_code = id[0] & 0xff80
            if func_code != self._FUNC_CODE_FREE:
                device_id = id[0] & 0x007f
                self._motors[device_id][index[0]] = data[0]
            self._rx_lock.release()

    # Establish the Low Level Communication Process
    # This Process Runs in a Background Process
    def _linkProcess(self):
        while self._thread_stop_flag == 0:
            self._tx_lock.acquire()
            # print('msg length:', self._tx_queue.qsize())
            if not self._tx_queue.empty():
                msg = self._tx_queue.get()
                print([hex(i) for i in msg])
                self._tx_lock.release()
                self._connection.write(msg)
                rxmsg = self._connection.read(11)
                print([hex(i) for i in rxmsg])
                self._analysis(rxmsg)
                self._connection.reset_input_buffer()
            else:
                # msg = ctypes.create_string_buffer(10)
                # struct.pack_into('h', msg, 0, self._FUNC_CODE_FREE)
                # struct.pack_into('h', msg, 2, *(0,))
                # struct.pack_into('h', msg, 4, *(0,))
                # struct.pack_into('i', msg, 6, *(0,))
                self._tx_lock.release()
                time.sleep(0.01)

            #

    # Send Message Function, Users would Call this Function to Send Messages
    def _sendMessage(self, func_code, device_id, index, sub_index, data):
        data = int(data)
        id = (func_code + device_id,)
        message = bytearray(10)
        struct.pack_into('h', message, 0, *id)
        struct.pack_into('h', message, 2, *(index,))
        struct.pack_into('h', message, 4, *(sub_index,))
        struct.pack_into('i', message, 6, *(data,))
        self._tx_lock.acquire()
        self._tx_queue.put(message)
        self._tx_lock.release()

    # Stop the communication
    def stop(self):
        time.sleep(0.5)
        self._thread_stop_flag = 1
        print('Python SDK for DBD Ant Stopped.')

    def setPowerOn(self, id):
        self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_CONTROL_WORD, self._SUBINDEX_WRITE, 1)

    def setPowerOff(self, id):
        self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_CONTROL_WORD, self._SUBINDEX_WRITE, 0)

    def setTargetVelocity(self, id, value):
        # Note: Unit: pulse/ms (50000 pulse per round), this unit nearly equals to RPM
        #       and for stepper motors, 0 to 300 is reasonable, higher speed will lose steps
        self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_TARGET_VELOCITY, self._SUBINDEX_WRITE, value)

    def setTargetPosition(self, id, value):
        # Note: Unit pulse, with 50000 pulse per round, and the value in should in range from -2^31 to 2^31
        self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_TARGET_POSITION, self._SUBINDEX_WRITE, value)

    def setVelocityMode(self, id):
        self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_OPERATION_MODE, self._SUBINDEX_WRITE,
                          self._OPERATION_MODE_PROFILE_VELOCITY)

    def setPositionMode(self, id):
        self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_OPERATION_MODE, self._SUBINDEX_WRITE,
                          self._OPERATION_MODE_PROFILE_POSITION)

    def setHomingMode(self, id):
        self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_OPERATION_MODE, self._SUBINDEX_WRITE,
                          self._OPERATION_MODE_HOMING)

    def setHomingDirection(self, id, value):
        if value == 1:
            self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_HOMING_DIRECTION, self._SUBINDEX_WRITE,
                              1)
        elif value == -1:
            self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_HOMING_DIRECTION, self._SUBINDEX_WRITE,
                              -1)
        else:
            print("wrong value, please try 1 or -1.")

    def setHomingLevel(self, id, value):
        if value == 1:
            self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_HOMING_LEVEL, self._SUBINDEX_WRITE,
                              1)
        elif value == 0:
            self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_HOMING_LEVEL, self._SUBINDEX_WRITE,
                              0)
        else:
            print("wrong value, please try 1 or 0.")

    def setRunningCurrent(self, id, value):
        self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_RUNNING_CURRENT, self._SUBINDEX_WRITE, value)

    def setKeepingCurrent(self, id, value):
        self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_KEEPING_CURRENT, self._SUBINDEX_WRITE, value)

    def setAccTime(self, id, value):
        # Note: acc time is a parameter for accelation and deaccelation progress, unit is ms, normally 200ms to 1000ms is reasonable
        self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_ACC_TIME, self._SUBINDEX_WRITE, value)

    def setOutputIO(self, id, value):
        self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_IO_OUT, self._SUBINDEX_WRITE, value)

    def getInputIO(self, id):
        self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_IO_INPUT, self._SUBINDEX_READ, 0)
        time.sleep(0.05)
        return self._motors[id][self._INDEX_IO_INPUT]

    def getActualVelocity(self, id):
        self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_ACTUAL_VELOCITY, self._SUBINDEX_READ, 0)
        time.sleep(0.05)
        return self._motors[id][self._INDEX_ACTUAL_VELOCITY]

    def getActualPosition(self, id):
        self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_ACTUAL_POSITION, self._SUBINDEX_READ, 0)
        time.sleep(0.05)
        return self._motors[id][self._INDEX_ACTUAL_POSITION]

    def getTargetVelocity(self, id):
        self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_TARGET_VELOCITY, self._SUBINDEX_READ, 0)
        time.sleep(0.05)
        return self._motors[id][self._INDEX_TARGET_VELOCITY]

    def getTargetPosition(self, id):
        self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_TARGET_POSITION, self._SUBINDEX_READ, 0)
        time.sleep(0.05)
        return self._motors[id][self._INDEX_TARGET_POSITION]

    def getRunningCurrent(self, id):
        self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_RUNNING_CURRENT, self._SUBINDEX_READ, 0)
        time.sleep(0.05)
        return self._motors[id][self._INDEX_RUNNING_CURRENT]

    def getKeepingCurrent(self, id):
        self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_KEEPING_CURRENT, self._SUBINDEX_READ, 0)
        time.sleep(0.05)
        return self._motors[id][self._INDEX_KEEPING_CURRENT]

    def getAccTime(self, id):
        self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_ACC_TIME, self._SUBINDEX_READ, 0)
        time.sleep(0.05)
        return self._motors[id][self._INDEX_ACC_TIME]

    def getHomingDirection(self, id):
        self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_HOMING_DIRECTION, self._SUBINDEX_READ, 0)
        time.sleep(0.05)
        return self._motors[id][self._INDEX_HOMING_DIRECTION]

    def getHomingLevel(self, id):
        self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_HOMING_LEVEL, self._SUBINDEX_READ, 0)
        time.sleep(0.05)
        return self._motors[id][self._INDEX_HOMING_LEVEL]

    def waitHomingDone(self, id):
        condition = 1
        while condition:
            vel = self.getActualVelocity(id)
            if (vel == 0):
                condition = 0

    def waitTargetPositionReached(self, id):
        condition = 1
        while condition:
            vel = self.getActualVelocity(id)
            if (vel == 0):
                condition = 0

    def getDeviceID(self, id):
        self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_DEVICE_ID, self._SUBINDEX_READ, 0)
        time.sleep(0.05)
        return self._motors[id][self._INDEX_DEVICE_ID]

    def scanDevices(self):
        online = []
        print('Searching Online Devices...')
        for i in range(0, 121):
            self._motors[i][self._INDEX_DEVICE_ID] = 0
        for i in range(1, 121):
            if i == self.getDeviceID(i):
                online.append(i)
        print('Online Devices:')
        print(online)

    def saveParameters(self, id):
        self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_MEMORY, self._SUBINDEX_WRITE, 1)

    def changeID(self, id, value):
        self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_DEVICE_ID, self._SUBINDEX_WRITE, value)
        time.sleep(0.05)
        self.saveParameters(value)

    # Init Process
    def __init__(self, portName):
        # Variables
        self._connection = 0
        # Motors consist of 128 motor cells, each motor has 1024 parameters
        # self._motors = numpy.zeros((128, 1024), dtype=numpy.int32)
        array = ((ctypes.c_int32 * 128) * 1024)
        self._motors = array()
        self._thread_stop_flag = 0
        self._tx_queue = queue.Queue()
        self._tx_lock = threading.Lock()
        self._rx_lock = threading.Lock()
        self._portName = portName
        self._connect()
        self._thread1 = threading.Thread(target=self._linkProcess)
        self._thread1.start()
        print('Pyhton SDK for DBD Ant Started')

    def benchSDO(self, mid, count):
        c = count
        message = ctypes.create_string_buffer(10)

        struct.pack_into('h', message, 0, *((0x580 + mid),))
        struct.pack_into('h', message, 2, *(self._INDEX_CONTROL_WORD,))
        struct.pack_into('h', message, 4, *(self._SUBINDEX_READ,))
        struct.pack_into('i', message, 6, *(0,))
        print(repr(message.raw))
        time_start = time.time()
        while count > 0:
            self._connection.write(message)
            self._analysis(self._connection.read(11))
            count = count - 1
        time_end = time.time()
        time_c = time_end - time_start
        print('time cost', time_c, 's')
        print('average time', time_c / c * 1000, 'ms')

    def benchSDO32(self, mid, count):
        c = count
        message = ctypes.create_string_buffer(32)

        struct.pack_into('h', message, 0, *((0x580 + mid),))
        struct.pack_into('h', message, 2, *(self._INDEX_CONTROL_WORD,))
        struct.pack_into('h', message, 4, *(self._SUBINDEX_READ,))
        struct.pack_into('i', message, 6, *(0,))
        print(repr(message.raw))
        time_start = time.time()
        while count > 0:
            self._connection.write(message)
            self._analysis(self._connection.read(32))
            count = count - 1
        time_end = time.time()
        time_c = time_end - time_start
        print('time cost', time_c, 's')
        print('average time', time_c / c * 1000, 'ms')


m = Ant('/dev/ttyUSB0')

m.getAccTime(1)
time.sleep(1)
m.stop()
