import serial
import time
import numpy as np
from binascii import unhexlify
import serial
import time
from crcmod import mkCrcFun

# import keyboard

class RS485Control:
    def __init__(self, 
        suction_read_port='/dev/ttyUSB0', suction_read_baud=9600,
        # suction_write_port='/dev/ttyUSB0', suction_write_baud=9600,
        threshold_true_vacuum_kpa = -55.0,
        threshold_not_activated_kpa = -1.0):
        try:
            self.ser = serial.Serial(
                suction_read_port, 
                suction_read_baud, 
                timeout=0.03,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                xonxoff = False,
                rtscts = False,
                dsrdtr=False,
                ) 
            # self.ser_read.open()

            # self.ser_write = serial.Serial(
            #     suction_write_port, 
            #     suction_write_baud, 
            #     timeout=0.01,
            #     bytesize=serial.EIGHTBITS,
            #     parity=serial.PARITY_NONE,
            #     stopbits=serial.STOPBITS_ONE,
            #     xonxoff = False,
            #     rtscts = False,
            #     dsrdtr=False,
            #     ) 
            # self.ser_read.open()
        except Exception as ex:
            print("Open serial port error " + str(ex))
            exit()
        
        self.threshold_true_vacuum_kpa = threshold_true_vacuum_kpa # -55
        self.threshold_not_activated_kpa = threshold_not_activated_kpa # -1
        self.activation_state = False
        

    def get_pressure(self):
        # Returned data 
        # Commands to send, including crc
        # [12][04][00][01][00][01][62][A9]
        # [13][04][00][01][00][01][63][78]
        # [14][04][00][01][00][01][62][CF]
        # [15][04][00][01][00][01][63][1E]

        commands=[
            [0x12, 0x04, 0x00, 0x01, 0x00, 0x01, 0x62, 0xA9],
            [0x13, 0x04, 0x00, 0x01, 0x00, 0x01, 0x63, 0x78],
            [0x14, 0x04, 0x00, 0x01, 0x00, 0x01, 0x62, 0xCF],
            [0x15, 0x04, 0x00, 0x01, 0x00, 0x01, 0x63, 0x1E],
        ]

        pressures = []
        for command in commands:
            response = self.ser_send(self.ser, command)
            kpa = int.from_bytes(response[3:5], 'big', signed=True)/10
            pressures.append(kpa)
        return np.array(pressures)
        
    def ser_send(self, port, data):
        if port.isOpen():
            try:
                port.flushInput()  # flush input buffer
                port.flushOutput()  # flush output buffer
                print('sending:', data)
                port.write(serial.to_bytes(data))
                time.sleep(port.timeout)  # wait 0.5s
                # read 8 byte data
                response = port.read(8)
                # print("read 8 byte data: ", response)
                # self.ser_read.close()
                return response
            except Exception as e1:
                print("communicating error " + str(e1))
        else:
            print("open serial port error")

    def activate(self):
        commands=[
            [0x01, 0x05, 0x00, 0x00, 0xFF, 0x00, 0x8C, 0x3A], # open first solenoid valve
            [0x01, 0x05, 0x00, 0x01, 0xFF, 0x00, 0xDD, 0xFA], # open second solenoid valve
            [0x01, 0x05, 0x00, 0x02, 0xFF, 0x00, 0x2D, 0xFA], # open third solenoid valve
            [0x01, 0x05, 0x00, 0x03, 0xFF, 0x00, 0x7C, 0x3A], # open fourth solenoid valve
        ]
        for command in commands:
            self.ser_send(self.ser, command)

        self.activation_state = True
        print("Activate suction cup")
    
    def deactivate(self):
        commands=[
            [0x01, 0x05, 0x00, 0x00, 0x00, 0x00, 0xCD, 0xCA],
            [0x01, 0x05, 0x00, 0x01, 0x00, 0x00, 0x9C, 0x0A],
            [0x01, 0x05, 0x00, 0x02, 0x00, 0x00, 0x6C, 0x0A],
            [0x01, 0x05, 0x00, 0x03, 0x00, 0x00, 0x3D, 0xCA],
        ]
        for command in commands:
            self.ser_send(self.ser, command)

        self.activation_state = False
        print("Deactivate suction cup")
    
    def toggle(self):
        if self.activation_state == False:
            self.activate()
        else:
            self.deactivate()

    def read_sucker_state(self):
        pressures = self.get_pressure()
        success = (pressures < self.threshold_true_vacuum_kpa) # if smaller than true vacuum threshold then should be grip success
        not_activated = (pressures > self.threshold_not_activated_kpa) # if larger than not activated threshold then should be not activated
        # anything in between should be failed grip / partial grip etc
        
        print("Pressures:", pressures, " | Successes: ", success, "| Not activated: ", not_activated)
        return success 

  
    def int_hex2(self, int):
        return '{0:0{1}X}'.format(int, 2)

    def int_hex4(self, int):
        return '{0:0{1}X}'.format(int, 4)

    def int_hex8(self, int):        
        return '{0:0{1}X}'.format(int, 8)

    def int_0x2(self, int):
        return '0x{0:0{1}X}'.format(int, 2)

    def int_0x4(self, int):
        return '0x{0:0{1}X}'.format(int, 4)

    def int_0x8(self, int):
        return '0x{0:0{1}X}'.format(int, 8)

    def crc16_modbus(self, s):
        crc16 = mkCrcFun(0x18005, rev=True, initCrc=0xFFFF, xorOut=0x0000)
        return self.get_crc_value(s, crc16)

    def str_to_hex(self, hex_string):
        # hex_string = "0xFF"
        an_integer = int(hex_string, 16)
        hex_value = hex(an_integer)
        return an_integer

    def get_crc_value(self, s, crc16):
        data = s.replace(' ', '')
        crc_out = hex(crc16(unhexlify(data))).upper()
        str_list = list(crc_out)
        if len(str_list) == 5:
            str_list.insert(2, '0')  # 位数不足补0
        if len(str_list) == 4:
            str_list.insert(2, '00')  # 位数不足补0
        crc_data = ''.join(str_list[2:])
        # return crc_data[:2] + ' ' + crc_data[2:]
        # return crc_data[2:] + crc_data[:2]
        return self.str_to_hex("0x" + crc_data[2:]), self.str_to_hex("0x" + crc_data[:2])


    def forward_n_step(self, addr, n):
        # addr = 2
        func_code = 16
        reg_addr = 2004  # forward
        num_regs = 2
        data_len = 4
        reg_value = n

        data_to_crc = self.int_hex2(addr) + self.int_hex2(func_code) + self.int_hex4(reg_addr) + self.int_hex4(
            num_regs) + self.int_hex2(data_len) + self.int_hex8(reg_value)

        p1, p2 = self.crc16_modbus(data_to_crc)

        data_with_crc = data_to_crc + self.int_hex2(p1) + self.int_hex2(p2)

        mylist = [0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff]
        for i in range(len(mylist)):
            mylist[i] = self.str_to_hex('0x' + data_with_crc[i * 2:(i + 1) * 2])

        self.ser_send(self.ser, mylist)


    def backward_n_step(self, addr, n):
        # addr = 2
        func_code = 16
        reg_addr = 2006  # backward
        num_regs = 2
        data_len = 4
        reg_value = n

        data_to_crc = self.int_hex2(addr) + self.int_hex2(func_code) + self.int_hex4(reg_addr) + self.int_hex4(
            num_regs) + self.int_hex2(data_len) + self.int_hex8(reg_value)

        p1, p2 = self.crc16_modbus(data_to_crc)

        data_with_crc = data_to_crc + self.int_hex2(p1) + self.int_hex2(p2)

        mylist = [0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff]
        for i in range(len(mylist)):
            mylist[i] = self.str_to_hex('0x' + data_with_crc[i * 2:(i + 1) * 2])

        self.ser_send(self.ser, mylist)

    def go_position(self, position):
        addr = 1
        func_code = 16
        reg_addr = 2002  # position mode
        num_regs = 2
        data_len = 4
        reg_value = position

        data_to_crc = self.int_hex2(addr) + self.int_hex2(func_code) + self.int_hex4(reg_addr) + self.int_hex4(
            num_regs) + self.int_hex2(data_len) + self.int_hex8(reg_value)
        # print(data_to_crc)
        p1, p2 = self.crc16_modbus(data_to_crc)

        data_with_crc = data_to_crc + self.int_hex2(p1) + self.int_hex2(p2)
        # print(data_with_crc)
        mylist = [0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff]
        for i in range(len(mylist)):
            mylist[i] = self.str_to_hex('0x' + data_with_crc[i * 2:(i + 1) * 2])

        self.ser_send(self.ser, mylist)

    def move_angle(self, angle):
        steps_to_take = round(self.full_revolution_steps/360*angle)
        if angle < 0:
            self.backward_n_step(abs(steps_to_take))
        if angle > 0:
            self.forward_n_step(abs(steps_to_take))

    def reset(self):
        mylist = [0x01, 0x06, 0x07, 0xD0, 0x00, 0x01, 0x48, 0x87]
        self.ser_send(self.ser, mylist)
        


if __name__ == "__main__":
    pneumatics = Pneumatics()
    while 1:
        pneumatics.read_sucker_state()
        time.sleep(5)
        pneumatics.toggle()
        pneumatics.backward_n_step(2, 50000)
        time.sleep(5)
        pneumatics.toggle()
        pneumatics.forward_n_step(2, 50000)
