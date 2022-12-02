from binascii import unhexlify

import serial
import time
from crcmod import mkCrcFun
# import keyboard

class MotorController:
    def __init__(self, port):
        self.port = port
        pos_col_one = 0
        pos_col_two = 70000
        pos_col_three = 140000
        pos_col_four = 210000
        self.box_pos = [pos_col_one, pos_col_two, pos_col_three, pos_col_four]

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
        crc_data = ''.join(str_list[2:])
        # return crc_data[:2] + ' ' + crc_data[2:]
        # return crc_data[2:] + crc_data[:2]
        return self.str_to_hex("0x" + crc_data[2:]), self.str_to_hex("0x" + crc_data[:2])


    def forward_n_step(self, n):
        addr = 1
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

        self.ser_send(mylist)


    def backward_n_step(self, n):
        addr = 1
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

        self.ser_send(mylist)


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

        self.ser_send(mylist)


    def reset(self):
        mylist = [0x01, 0x06, 0x07, 0xD0, 0x00, 0x01, 0x48, 0x87]
        self.ser_send(mylist)
        
    def ser_send(self, data):
        ser = serial.Serial()
        ser.port = self.port
        # 9600,N,8,1
        ser.baudrate = 9600
        ser.bytesize = serial.EIGHTBITS  # number of bits per bytes
        ser.parity = serial.PARITY_NONE  # set parity check
        ser.stopbits = serial.STOPBITS_ONE  # number of stop bits
        ser.timeout = 0.5  # non-block read 0.5s
        ser.writeTimeout = 0.5  # timeout for write 0.5s
        ser.xonxoff = False  # disable software flow control
        ser.rtscts = False  # disable hardware (RTS/CTS) flow control
        ser.dsrdtr = False  # disable hardware (DSR/DTR) flow control
        try:
            ser.open()
        except Exception as ex:
            print("open serial port error " + str(ex))
            exit()
        if ser.isOpen():
            try:
                ser.flushInput()  # flush input buffer
                ser.flushOutput()  # flush output buffer
                print('sending:', data)
                ser.write(serial.to_bytes(data))
                time.sleep(0.5)  # wait 0.5s
                # read 8 byte data
                response = ser.read(8)
                print("read 8 byte data:")
                print(response)
                ser.close()
            except Exception as e1:
                print("communicating error " + str(e1))
        else:
            print("open serial port error")


    def move_to_all_box(self, box_pos):
    # test for moving along each box
        for i in range(4):
            print("Moving to col", i)
            if i == 0:
                mc.go_position(box_pos[i])
                time.sleep(9)
            else:    
                mc.go_position(box_pos[i])
                time.sleep(6)

    def find_box(self, pill_id):
    # return box 
        if pill_id == 'A':
            return self.box_pos[0]
        elif pill_id == 'B':
            return self.box_pos[1]
        elif pill_id == 'C':
            return self.box_pos[2]
        elif pill_id == 'D':
            return self.box_pos[3]
        elif pill_id == 'E':
            return self.box_pos[3]
        elif pill_id == 'F':
            return self.box_pos[2]
        elif pill_id == 'G':
            return self.box_pos[1]
        elif pill_id == 'H':
            return self.box_pos[0]
        else:
            print('Pill ID error!')
            return self.box_pos[0]


if __name__ == "__main__":

    mc = MotorController('/dev/ttyUSB0')

    # print("forward")
    mc.forward_n_step(20000)
    # mc.reset()

    # print("backward")
    # mc.backward_n_step(2000)
    # dmesg | grep tty


    

    # mc.move_to_all_box(box_pos)
    # print("Go to position")
    # mc.go_position(mc.box_pos[3])
    # mc.go_position(mc.find_box('A'))
    

