# MIT License

# Copyright (c) 2025 Robert Hammelrath

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from machine import Pin
import time
import array
import micropython

class ADS1256:
    # Registers
    REG_STATUS = const(0x00)
    REG_MUX = const(0x01)
    REG_ADCON = const(0x02)
    REG_DRATE = const(0x03)
    REG_IO = const(0x04)
    REG_OFC0 = const(0x05)
    REG_OFC1 = const(0x06)
    REG_OFC2 = const(0x07)
    REG_FSC0 = const(0x08)
    REG_FSC1 = const(0x09)
    REG_FSC2 = const(0x0A)

    #Commands
    CMD_WAKEUP = const(0x00)
    CMD_RDATA = const(0x01)
    CMD_RDATAC = const(0x03)
    CMD_SDATAC = const(0x0F)
    CMD_RREG = const(0x10)
    CMD_WREG = const(0x50)
    CMD_SELFCAL = const(0xF0)
    CMD_SELFOCAL = const(0xF1)
    CMD_SELFGCAL = const(0xF2)
    CMD_SYSOCAL = const(0xF3)
    CMD_SYSGCAL = const(0xF4)
    CMD_SYNC = const(0xFC)
    CMD_STANDBY = const(0xFD)
    CMD_RESET = const(0xFE)

    _rate = {
         30000: 0b11110000,
         15000: 0b11100000,
         7500: 0b11010000,
         3750: 0b11000000,
         2000: 0b10110000,
         1000: 0b10100001,
         500: 0b10010010,
         100: 0b10000010,
         60: 0b01110010,
         50: 0b01100011,
         30: 0b01010011,
         25: 0b01000011,
         15: 0b00110011,
         10: 0b00100011,
         5: 0b00010011,
         2.5: 0b00000011,
    }

    _gain = {
        1 : 0b0000,
        2 : 0b0001,
        4 : 0b0010,
        8 : 0b0011,
        16: 0b0100,
        32: 0b0101,
        64: 0b0110,
    }

    AINCOM = const(8)
    NUM_INPUTS = 8
    SELFCAL_GAIN = const(1)
    SELFCAL_OFFSET = const(2)

    def __init__(self, spi, cs, drdy, gain=1, rate=1000):
        self.spi = spi
        self.cs = cs
        self.drdy = drdy
        self.cs.init(mode=Pin.OUT, value=1)
        self.drdy.init(mode=Pin.IN)

        self.buffer_1 = bytearray(1)
        self.buffer_2 = bytearray(2)
        self.buffer_3 = bytearray(3)

        # The channel table is a list of logical channels with it's
        # configuration of inputs, gain and rate. If needed,
        # the device will be reconfigured when the respective
        # channel is used. A channel definition is created or modified
        # with the channel() method.

        self.reset()
        self.channel_table = { }
        # create a single default entry.
        self.channel(0, 0, AINCOM, gain, rate)
        # Wait a moment after reset
        time.sleep_ms(1)
        self.channel_setup(0)

    def __call__(self):
        return self.read(self.previous_channel)

    def __repr__(self):
        return [self.channel(key) for key in self.channel_table]

    # Write to the registers. data is either an object with buffer
    # protocol or for convenience a single number.
    def write_reg(self, reg, data):
        if reg > REG_FSC2:
            raise ValueError("Invalid register")
        if type(data) is int:
            self.buffer_3[0] = CMD_WREG + reg
            self.buffer_3[1] = 0
            self.buffer_3[2] = data
            self.cs(0)
            self.spi.write(self.buffer_3)
            self.cs(1)
        else:
            self.buffer_2[0] = CMD_WREG + reg
            self.buffer_2[1] = len(data) - 1
            self.cs(0)
            self.spi.write(self.buffer_2)
            self.spi.write(data)
            self.cs(1)

    # Read number bytes from register reg.
    def read_reg(self, reg, number=1):
        if reg > REG_FSC2:
            raise ValueError("Invalid register")
        self.buffer_2[0] = CMD_RREG + reg
        self.buffer_2[1] = number - 1
        self.cs(0)
        self.spi.write(self.buffer_2)
        time.sleep_us(6)
        result = self.spi.read(number)
        self.cs(1)
        return result

    def write_cmd(self, cmd):
        self.buffer_1[0] = cmd
        self.cs(0)
        self.spi.write(self.buffer_1)
        self.cs(1)

    def _drdy_cb(self, drdy):
        self.drdy.irq(handler=None)
        self.__drdy = True

    def _wait_for_drdy(self):
        # Set up the trigger for conversion enable.
        self.__drdy = False
        self.drdy.irq(trigger=Pin.IRQ_FALLING, handler=self._drdy_cb)
        # Wait about 5 seconds for the DRDY event
        for _ in range(1000000):
            if self.__drdy is True:
                break
            time.sleep_us(50)
        else:
            self.__drdy = False
            self.drdy.irq(handler=None)
            raise OSError("Sensor does not respond")

    @micropython.native
    def align_buffer(self, buffer):
        for i in range(len(buffer)):
            if buffer[i] > 0x7FFFFF:
                buffer[i] -= 0x1000000
        self.data_acquired = True

    @micropython.native
    def __buffer_cb(self, drdy):
        # Check the sign later when it's time to do so
        if self.buffer_index < self.buffer_size:
            self.cs(0)
            self.spi.readinto(self.buffer_3)
            self.cs(1)
            self.buffer[self.buffer_index] = self.buffer_3[0] << 16 | self.buffer_3[1] << 8 | self.buffer_3[2]
            self.buffer_index += 1
        elif self.buffer_index == self.buffer_size:
            self.write_cmd(CMD_SDATAC)
            self.buffer_index += 1
        else:
            self.drdy.irq(handler=None)
            try:
                micropython.schedule(self.align_buffer, self.buffer)
            except:
                self.align_buffer(self.buffer)

    # read a single value or a set of values from a channel, which
    # has to be defined using the channel() method.
    def read(self, channel, buffer=None):
        if channel != self.previous_channel:
            self.channel_setup(channel)

        if buffer is None:
            self._wait_for_drdy()
            # send the command and get the data
            self.buffer_1[0] = CMD_RDATA
            self.cs(0)
            self.spi.write(self.buffer_1)
            # time.sleep_us(6)
            self.spi.readinto(self.buffer_3)
            self.cs(1)
            result = self.buffer_3[0] << 16 | self.buffer_3[1] << 8 | self.buffer_3[2]
            if result > 0x7FFFFF:
                result -= 0x1000000
            return result
        else:
            # Send the command and get one data item, which
            # is discarded
            self._wait_for_drdy()
            self.buffer_1[0] = CMD_RDATAC
            self.cs(0)
            self.spi.write(self.buffer_1)
            # time.sleep_us(6)
            self.spi.readinto(self.buffer_3)
            self.cs(1)

            self.data_acquired = False
            self.buffer = buffer
            self.buffer_size = len(buffer)
            self.buffer_index = 0
            self.drdy.irq(trigger=Pin.IRQ_FALLING, handler=self.__buffer_cb)
            return len(buffer)

    # configure the channel
    def channel_setup(self, channel):
        if channel not in self.channel_table.keys():
            raise ValueError("channel not defined")
        # set the mux, gain and rate in one transfer
        config_new = self.channel_table[channel]
        self.write_reg(REG_MUX, config_new)
        # Check the need for calibration
        if self.previous_channel is not None:
            config_old = self.channel_table[self.previous_channel]
            cal_mode = (config_new[1] != config_old[1]) | ((config_new[2] != config_old[2]) << 1)
        else:
            # The set-up after reset may not match the first used configuration,
            # so do a full calibration.
            cal_mode = SELFCAL_GAIN | SELFCAL_OFFSET
        self.calibration(cal_mode)
        self.previous_channel = channel
        # wait for the actual cycle to finish
        self._wait_for_drdy()

    # define or redefine a logical channel
    def channel(self, channel, ainp=None, ainn=AINCOM, gain=None, rate=None):
        if ainp is not None:
            # check the argument values
            if not (ainp == AINCOM or 0 <= ainp < self.NUM_INPUTS):
                raise ValueError("invalid input ainp")
            if not (ainn == AINCOM or 0 <= ainn < self.NUM_INPUTS):
                raise ValueError("invalid input ainn")
            if not (gain is None or gain in self._gain.keys()):
                raise ValueError("invalid value for gain")
            if not (rate is None or rate in self._rate.keys()):
                raise ValueError("invalid value for rate")

            if channel in self.channel_table.keys():
                # redefine a channel
                config = self.channel_table[channel]
                if gain is not None:
                    config[1] = self._gain[gain]
                if rate is not None:
                    config[2] = self._rate[rate]
            else:
                # define a new channel.
                config = bytearray(3)
                config[1] = self._gain[1] if gain is None else self._gain[gain]
                config[2] = self._rate[1000] if rate is None else self._rate[rate]
            # the input numbers are always set and must not be None
            config[0] = (ainp << 4) | ainn
            self.channel_table[channel] = config
            # force re-configuration if the current channel is (re-)configured,
            if channel == self.previous_channel:
                self.previous_channel = None
        else:
            if channel in self.channel_table.keys(): 
                config = self.channel_table[channel]
                return "channel({}, ainp={}, ainn={}, gain={}, rate={})".format(
                    channel, config[0] >> 4, config[0] & 0x0f,
                    {value: key for key, value in self._gain.items()}[config[1]],
                    {value: key for key, value in self._rate.items()}[config[2]])
            else:
                raise ValueError("channel not defined")

    def standby(self):
        self.write_cmd(CMD_STANDBY)

    def wakeup(self):
        self.write_cmd(CMD_WAKEUP)

    def reset(self):
        self.write_cmd(CMD_RESET)
        self.previous_channel = None

    def calibration(self, mode=SELFCAL_GAIN | SELFCAL_OFFSET):
        if mode == SELFCAL_GAIN | SELFCAL_OFFSET:
            self.buffer_1[0] = CMD_SELFCAL
        elif mode == SELFCAL_GAIN:
            self.buffer_1[0] = CMD_SELFGCAL
        elif mode == SELFCAL_OFFSET:
            self.buffer_1[0] = CMD_SELFOCAL
        else:
            return
        self.cs(0)
        self.spi.write(self.buffer_1)
        self.cs(1)
        # Wait for the calibration to finish
        self._wait_for_drdy()


class ADS1255(ADS1256):

    NUM_INPUTS = 2
