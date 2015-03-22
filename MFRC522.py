#!/usr/bin/env python
# -*- coding: utf8 -*-

import RPi.GPIO as GPIO
import spi
import signal
import time


class MFRC522:
    NRSTPD = 22

    MAX_LEN = 16

    PCD_IDLE = 0x00
    PCD_AUTHENT = 0x0E
    PCD_RECEIVE = 0x08
    PCD_TRANSMIT = 0x04
    PCD_TRANSCEIVE = 0x0C
    PCD_RESETPHASE = 0x0F
    PCD_CALCCRC = 0x03

    PICC_REQIDL = 0x26
    PICC_REQALL = 0x52
    PICC_ANTICOLL = 0x93
    PICC_SElECTTAG = 0x93
    PICC_AUTHENT1A = 0x60
    PICC_AUTHENT1B = 0x61
    PICC_READ = 0x30
    PICC_WRITE = 0xA0
    PICC_DECREMENT = 0xC0
    PICC_INCREMENT = 0xC1
    PICC_RESTORE = 0xC2
    PICC_TRANSFER = 0xB0
    PICC_HALT = 0x50

    MI_OK = 0
    MI_NOTAGERR = 1
    MI_ERR = 2

    Reserved00 = 0x00
    CommandReg = 0x01
    CommIEnReg = 0x02
    DivlEnReg = 0x03
    CommIrqReg = 0x04
    DivIrqReg = 0x05
    ErrorReg = 0x06
    Status1Reg = 0x07
    Status2Reg = 0x08
    FIFODataReg = 0x09
    FIFOLevelReg = 0x0A
    WaterLevelReg = 0x0B
    ControlReg = 0x0C
    BitFramingReg = 0x0D
    CollReg = 0x0E
    Reserved01 = 0x0F

    Reserved10 = 0x10
    ModeReg = 0x11
    TxModeReg = 0x12
    RxModeReg = 0x13
    TxControlReg = 0x14
    TxAutoReg = 0x15
    TxSelReg = 0x16
    RxSelReg = 0x17
    RxThresholdReg = 0x18
    DemodReg = 0x19
    Reserved11 = 0x1A
    Reserved12 = 0x1B
    MifareReg = 0x1C
    Reserved13 = 0x1D
    Reserved14 = 0x1E
    SerialSpeedReg = 0x1F

    Reserved20 = 0x20
    CRCResultRegM = 0x21
    CRCResultRegL = 0x22
    Reserved21 = 0x23
    ModWidthReg = 0x24
    Reserved22 = 0x25
    RFCfgReg = 0x26
    GsNReg = 0x27
    CWGsPReg = 0x28
    ModGsPReg = 0x29
    TModeReg = 0x2A
    TPrescalerReg = 0x2B
    TReloadRegH = 0x2C
    TReloadRegL = 0x2D
    TCounterValueRegH = 0x2E
    TCounterValueRegL = 0x2F

    Reserved30 = 0x30
    TestSel1Reg = 0x31
    TestSel2Reg = 0x32
    TestPinEnReg = 0x33
    TestPinValueReg = 0x34
    TestBusReg = 0x35
    AutoTestReg = 0x36
    VersionReg = 0x37
    AnalogTestReg = 0x38
    TestDAC1Reg = 0x39
    TestDAC2Reg = 0x3A
    TestADCReg = 0x3B
    Reserved31 = 0x3C
    Reserved32 = 0x3D
    Reserved33 = 0x3E
    Reserved34 = 0x3F

    serNum = []

    def __init__(self, dev='/dev/spidev0.0', spd=1000000):
        spi.openSPI(device=dev, speed=spd)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(22, GPIO.OUT)
        GPIO.output(self.NRSTPD, 1)

        GPIO.output(self.NRSTPD, 1)

        self.reset()

        self.write(self.TModeReg, 0x8D)
        self.write(self.TPrescalerReg, 0x3E)
        self.write(self.TReloadRegL, 30)
        self.write(self.TReloadRegH, 0)

        self.write(self.TxAutoReg, 0x40)
        self.write(self.ModeReg, 0x3D)
        self.antenna_on()

    def reset(self):
        self.write(self.CommandReg, self.PCD_RESETPHASE)

    @staticmethod
    def write(address, val):
        spi.transfer(((address << 1) & 0x7E, val))

    @staticmethod
    def read(address):
        val = spi.transfer((((address << 1) & 0x7E) | 0x80, 0))
        return val[1]

    def set_bit_mask(self, reg, mask):
        tmp = self.read(reg)
        self.write(reg, tmp | mask)

    def clear_bit_mask(self, reg, mask):
        tmp = self.read(reg)
        self.write(reg, tmp & (~mask))

    def antenna_on(self):
        temp = self.read(self.TxControlReg)
        if ~(temp & 0x03):
            self.set_bit_mask(self.TxControlReg, 0x03)

    def antenna_off(self):
        self.clear_bit_mask(self.TxControlReg, 0x03)

    def to_card(self, command, send_data):
        backData = []
        backLen = 0
        status = self.MI_ERR
        irqEn = 0x00
        waitIRq = 0x00
        lastBits = None
        n = 0
        i = 0

        if command == self.PCD_AUTHENT:
            irqEn = 0x12
            waitIRq = 0x10

        if command == self.PCD_TRANSCEIVE:
            irqEn = 0x77
            waitIRq = 0x30

        self.write(self.CommIEnReg, irqEn | 0x80)
        self.clear_bit_mask(self.CommIrqReg, 0x80)
        self.set_bit_mask(self.FIFOLevelReg, 0x80)

        self.write(self.CommandReg, self.PCD_IDLE);

        while i < len(send_data):
            self.write(self.FIFODataReg, send_data[i])
            i += 1

        self.write(self.CommandReg, command)

        if command == self.PCD_TRANSCEIVE:
            self.set_bit_mask(self.BitFramingReg, 0x80)

        i = 2000
        while True:
            n = self.read(self.CommIrqReg)
            i -= 1
            if ~((i != 0) and ~(n & 0x01) and ~(n & waitIRq)):
                break

        self.clear_bit_mask(self.BitFramingReg, 0x80)

        if i != 0:
            if (self.read(self.ErrorReg) & 0x1B) == 0x00:
                status = self.MI_OK

                if n & irqEn & 0x01:
                    status = self.MI_NOTAGERR

                if command == self.PCD_TRANSCEIVE:
                    n = self.read(self.FIFOLevelReg)
                    lastBits = self.read(self.ControlReg) & 0x07
                    if lastBits != 0:
                        backLen = (n - 1) * 8 + lastBits
                    else:
                        backLen = n * 8

                    if n == 0:
                        n = 1
                    if n > self.MAX_LEN:
                        n = self.MAX_LEN

                    i = 0
                    while i < n:
                        backData.append(self.read(self.FIFODataReg))
                        i += 1
            else:
                status = self.MI_ERR

        return status, backData, backLen

    def request(self, reqMode):
        status = None
        backBits = None
        TagType = []

        self.write(self.BitFramingReg, 0x07)

        TagType.append(reqMode)
        (status, backData, backBits) = self.to_card(self.PCD_TRANSCEIVE, TagType)

        if ((status != self.MI_OK) | (backBits != 0x10)):
            status = self.MI_ERR

        return status, backBits

    def anti_coll(self):

        serial_num_check = 0
        serial_number = [self.PICC_ANTICOLL, 0x20]

        self.write(self.BitFramingReg, 0x00)

        (status, backData, backBits) = self.to_card(self.PCD_TRANSCEIVE, serial_number)

        if status == self.MI_OK:

            if len(backData) == 5:

                i = 0
                while i < 4:
                    serial_num_check = serial_num_check ^ backData[i]
                    i += 1

                if serial_num_check != backData[i]:
                    status = self.MI_ERR
            else:
                status = self.MI_ERR

        return status, backData

    def calculate_crc(self, pIndata):

        self.clear_bit_mask(self.DivIrqReg, 0x04)
        self.set_bit_mask(self.FIFOLevelReg, 0x80)

        i = 0
        while i < len(pIndata):
            self.write(self.FIFODataReg, pIndata[i])
            i += 1

        self.write(self.CommandReg, self.PCD_CALCCRC)

        i = 0xFF
        while True:
            n = self.read(self.DivIrqReg)
            i -= 1
            if not ((i != 0) and not (n & 0x04)):
                break

        p_out_data = [self.read(self.CRCResultRegL), self.read(self.CRCResultRegM)]
        return p_out_data

    def select_tag(self, serial_number):

        buf = [self.PICC_SElECTTAG, 0x70]

        i = 0
        while i < 5:
            buf.append(serial_number[i])
            i += 1

        p_out = self.calculate_crc(buf)
        buf.append(p_out[0])
        buf.append(p_out[1])

        (status, back_data, back_len) = self.to_card(self.PCD_TRANSCEIVE, buf)

        if (status == self.MI_OK) and (back_len == 0x18):
            print "Size: " + str(back_data[0])
            return back_data[0]
        else:
            return 0

    def auth(self, auth_mode, block_address, sector_key, serial_number):

        # First byte should be the authMode (A or B)
        # Second byte is the trailerBlock (usually 7)
        buff = [auth_mode, block_address]

        # Now we need to append the authKey which usually is 6 bytes of 0xFF
        i = 0
        while i < len(sector_key):
            buff.append(sector_key[i])
            i += 1

        # Next we append the first 4 bytes of the UID
        i = 0
        while i < 4:
            buff.append(serial_number[i])
            i += 1

        # Now we start the authentication itself
        (status, back_data, back_len) = self.to_card(self.PCD_AUTHENT, buff)

        # Check if an error occurred
        if not (status == self.MI_OK):
            print "AUTH ERROR!!"
        if not (self.read(self.Status2Reg) & 0x08) != 0:
            print "AUTH ERROR(status2reg & 0x08) != 0"

        # Return the status
        return status

    def stop_crypto1(self):
        self.clear_bit_mask(self.Status2Reg, 0x08)

    def read_data(self, block_address):

        """
        Read Block data
        :param block_address: The block address to read
        :rtype : (int status, array:backData, int:backLen)
        """

        received_data = [self.PICC_READ, block_address]

        pOut = self.calculate_crc(received_data)

        received_data.append(pOut[0])
        received_data.append(pOut[1])

        return self.to_card(self.PCD_TRANSCEIVE, received_data)

    def read_string(self, block_address):
        (status, data, count) = self.read_data(block_address)

        if status == self.MI_OK and len(data) == 16:
            value = ""
            for i in range(0, 16):
                if data[i] == 0:
                    break
                value += chr(data[i])
            return value

        return False

    def read_integer(self, block_address):
        data = self.read_string(block_address)
        try:
            return int(data)
        except:
            return 0

    def dump_block(self, block_address):
        (status, data, count) = self.read_data(block_address)
        if status == self.MI_OK and len(data) == 16:
            print (str(data))
            return True
        return False

    def dump_classic_1k(self, key, uid):
        i = 0
        while i < 64:
            status = self.auth(self.PICC_AUTHENT1A, i, key, uid)
            # Check if authenticated
            if status == self.MI_OK:
                self.dump_block(i)
            else:
                print "Authentication error"
            i += 1

    def write_data(self, block_address, write_data):
        
        buff = [self.PICC_WRITE, block_address]
        crc = self.calculate_crc(buff)
        buff.append(crc[0])
        buff.append(crc[1])
        (status, data, count) = self.to_card(self.PCD_TRANSCEIVE, buff)
        if not (status == self.MI_OK) or not (count == 4) or not ((data[0] & 0x0F) == 0x0A):
            status = self.MI_ERR

        if status == self.MI_OK:
            i = 0
            buf = []
            while i < 16:
                buf.append(write_data[i])
                i += 1
            crc = self.calculate_crc(buf)
            buf.append(crc[0])
            buf.append(crc[1])
            (status, data, count) = self.to_card(self.PCD_TRANSCEIVE, buf)
            if not (status == self.MI_OK) or not (count == 4) or not ((data[0] & 0x0F) == 0x0A):
                return False
            if status == self.MI_OK:
                return True

    def write_string(self, block_address, write_string):
        
        data = []
        for x in range(0, 16):
            data.append(0x00)

        for x in range(0, len(write_string)):
            data[x] = ord(write_string[x])

        return self.write_data(block_address, data)

    def write_integer(self, block_address, write_integer):

        return self.write_string(block_address, str(write_integer))
