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

from machine import Pin, idle
import time
import micropython
import rp2

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

    def __init__(self, sck, din, dout, cs, drdy, statemachine=0, gain=1, rate=1000):

        self.timeout = 5000
        self.buffer_1 = bytearray(1)
        self.buffer_2 = bytearray(2)
        self.buffer_3 = bytearray(3)

        # The channel table is a list of logical channels with it's
        # configuration of inputs, gain and rate. If needed,
        # the device will be reconfigured when the respective
        # channel is used. A channel definition is created or modified
        # with the channel() method.

        self.channel_table = { }
        # create a single default entry.
        self.statemachine = statemachine & 0b0110   # force to 0, 2, 4, 6

        # set up DMA
        self.pio_dma = rp2.DMA()
        self.pio_ctrl = self.pio_dma.pack_ctrl(
            size=2,
            inc_read=False,
            inc_write=True,
            ring_size = 0,  # no wrapping
            treq_sel = self.statemachine * 2 + 4 + 1,  # 4-7 or 12-15
            irq_quiet = False, # generate an IRQ
            bswap = False   # Do not swap bytes (?)
        )

        self.ads1256_sm_cmd = rp2.StateMachine(self.statemachine, self.ads1256_asm_cmd,
            freq=4_000_000, set_base=drdy, in_base=din, out_base=dout, sideset_base=cs, jmp_pin=drdy)

        self.ads1256_sm_data = rp2.StateMachine(self.statemachine + 1, self.ads1256_asm_data,
            freq=3_500_000, set_base=drdy, in_base=din, out_base=dout, sideset_base=cs, jmp_pin=drdy)

        self.reset()
        self.channel(0, 0, AINCOM, gain, rate)
        self.channel_setup(0)

    def __call__(self):
        return self.read(self.previous_channel)


    @staticmethod
    @rp2.asm_pio(
        in_shiftdir=rp2.PIO.SHIFT_LEFT,
        out_shiftdir=rp2.PIO.SHIFT_LEFT,
        autopull=True,
        autopush=False,
        pull_thresh=8,
        out_init=(rp2.PIO.OUT_LOW),
        set_init=(rp2.PIO.IN_LOW, rp2.PIO.IN_LOW),
        sideset_init=(rp2.PIO.OUT_HIGH,rp2.PIO.OUT_LOW) # cs=1, sck=0
    )
    def ads1256_asm_cmd():
        # The mode is controlled by the fist two arguments, which must be pre-
        # loaded before start
        # First argument: Bits to write
        # Second argument: Bits to read
        out(x, 8)             .side(0b00)      # put it into x
        out(y, 8)             .side(0b00)      # put it into y
# Wait for a high level = start of the DRDY pulse
# Use jmp for wait to get an independent DRDY pin.
        label("wait_high")
        jmp(pin, "wait_low")    .side(0b00)
        jmp("wait_high")        .side(0b00)
# Wait for a low level = DRDY signal
        label("wait_low")
        jmp(pin, "wait_low")    .side(0b00)

# Now write the data
        jmp(not_x, "read_din")  .side(0b00)      # test for data to write
        jmp(x_dec, "write_bit") .side(0b00)      # Just decrement
        label("write_bit")
        out(pins, 1)            .side(0b10)[1]
        jmp(x_dec, "write_bit") .side(0b00)[1]

# Now read the data
        label("read_din")
        jmp(not_y, "end")       .side(0b00)[7]   # test for data to read
                                                 # and wait 8 clock cycles
        nop()                   .side(0b00)[7]   # Wait more clock cycles before read
        jmp("read_loop_dec")    .side(0b00)[7]   # Wait more clock cycles before read
                                                 # Total wait time must be 6µs at least
                                                 # Is now 6.76 µs
        label("read_bit")
        nop()                   .side(0b10)[1]   # Just set clock high
        in_(pins, 1)            .side(0b00)      # shift in one bit
        label("read_loop_dec")
        jmp(y_dec, "read_bit")  .side(0b00)      # and go for another bit, which
# done
        label("end")
        push()                  .side(0b01)[7]   # always a single push at the end
                                                 # which sets also CS inactive

    @staticmethod
    @rp2.asm_pio(
        in_shiftdir=rp2.PIO.SHIFT_LEFT,
        out_shiftdir=rp2.PIO.SHIFT_LEFT,
        autopull=False,
        autopush=True,
        push_thresh=24,
        out_init=(rp2.PIO.OUT_LOW),
        set_init=(rp2.PIO.IN_LOW, rp2.PIO.IN_LOW),
        sideset_init=(rp2.PIO.OUT_HIGH,rp2.PIO.OUT_LOW) # cs=1, sck=0
    )
    def ads1256_asm_data():
# Wait for a high level = start of the DRDY pulse
# Use jmp for wait to get an independent DRDY pin.
        pull()                   .side(0b00)    # get the # of samples
        mov(y, osr)              .side(0b00)
        label("wait_high")
        jmp(pin, "wait_low")    .side(0b00)
        jmp("wait_high")        .side(0b00)
# Wait for a low level = DRDY signal
        label("wait_low")
        jmp(pin, "wait_low")    .side(0b00)
        jmp(not_y, "end")
# now read the data, 24 bit
        set(x, 22)              .side(0b10)     # Set clock
        label("read_bit")
        in_(pins, 1)            .side(0b00)     # shift in one bit
        jmp(x_dec, "read_bit")  .side(0b10)     # and go for another bit, which
        in_(pins, 1)            .side(0b00)     # get the last bit
                                                # will be pushed automatically
        jmp(y_dec,"wait_high")  .side(0b00)     
        label ("end")
        pull()                  .side(0b00)
        set(x, 7)               .side(0b00)     # 8 bits to be sent
        label("write_bit")
        out(pins, 1)            .side(0b10)[1]  # Write SDATAC
        jmp(x_dec, "write_bit") .side(0b00)[1]
        push()                  .side(0b01)     # Tell it's finished, and pull CS high
                                                     
    def transfer_cmd(self, out_data, in_bytes, result_type=0):
        self.ads1256_sm_cmd.restart()
        # set the arguments for data sizes
        self.ads1256_sm_cmd.put(0 if out_data is None else (len(out_data) * 8) << 24)
        self.ads1256_sm_cmd.put((in_bytes * 8) << 24)
        self.ads1256_sm_cmd.active(1)       # start the transfer
        if out_data is not None:
            for value in out_data:
               self.ads1256_sm_cmd.put(value << 24)
        start = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), start) < self.timeout:
            if self.ads1256_sm_cmd.rx_fifo() > 0:
                break
        else:
            # Drain the RX fifo and set CS to 1.
            for _ in range(4):
                self.ads1256_sm_cmd.exec("pull(noblock).side(0b01)")
                self.ads1256_sm_cmd.active(0)
            raise OSError("sensor timeout")
        self.ads1256_sm_cmd.active(0)
        result = self.ads1256_sm_cmd.get()  # wait for a result, may be discarded
        if in_bytes > 0:
            if result_type == 1:  # return as number?
                return result
            else:
                data = bytearray(in_bytes)
                for i in range(in_bytes - 1, -1, -1):
                    data[i] = result & 0xff
                    result >>= 8
                return data
        else:
            return None

    # Write to the registers. data is either an object with buffer
    # protocol or for convenience a single number.
    def write_reg(self, reg, data):
        if reg > REG_FSC2:
            raise ValueError("Invalid register")
        if type(data) is int:
            self.buffer_3[0] = CMD_WREG + reg
            self.buffer_3[1] = 0
            self.buffer_3[2] = data
            self.transfer_cmd(self.buffer_3, 0)
        else:
            self.buffer_2[0] = CMD_WREG + reg
            self.buffer_2[1] = len(data) - 1
            self.transfer_cmd(self.buffer_2 + data, 0)

    # Read number bytes from register reg.
    def read_reg(self, reg, number=1):
        if reg > REG_FSC2:
            raise ValueError("Invalid register")
        self.buffer_2[0] = CMD_RREG + reg
        self.buffer_2[1] = number - 1
        result = self.transfer_cmd(self.buffer_2, number)
        return result

    def write_cmd(self, cmd):
        self.buffer_1[0] = cmd
        self.transfer_cmd(self.buffer_1, 0)

    # read a single value or a set of values from a channel, which
    # has to be defined using the channel() method.
    def read(self, channel, buffer=None):
        if channel != self.previous_channel:
            self.channel_setup(channel)

        if buffer is None:
            # send the command and get the data
            self.buffer_1[0] = CMD_RDATA
            result = self.transfer_cmd(self.buffer_1, 3, 1)
            if result > 0x7FFFFF:
                result -= 0x1000000
            return result
        else:
            # Send the command and get one data item, which
            # is discarded
            self.buffer_1[0] = CMD_RDATAC
            result = self.transfer_cmd(self.buffer_1, 3, 1)
            # now get the bulk of data
            self.buffer = buffer
            self.ads1256_sm_data.restart()
            self.ads1256_sm_data.put(len(buffer))
            self.ads1256_sm_data.put(CMD_SDATAC << 24)
            self.data_acquired = False
            self.pio_dma.config(read=self.ads1256_sm_data, write=buffer, count=len(buffer), ctrl=self.pio_ctrl)
            self.pio_dma.irq(handler=self.__irq_dma_finished, hard=False)
            self.pio_dma.active(True)
            self.ads1256_sm_data.active(1)  # An go off
            return len(buffer)

    def __irq_dma_finished(self, sm):
        # Shift and sign check later when it's time to do so
        self.pio_dma.irq(handler=None)
        buffer = self.buffer
        for i in range(len(buffer)):
            if buffer[i] > 0x7FFFFF:
                buffer[i] -= 0x1000000
        # wait for the DATA state machine to stop continous read
        while self.ads1256_sm_data.rx_fifo() == 0:
            idle()
        self.ads1256_sm_data.get()
        self.ads1256_sm_data.active(0)
        self.data_acquired = True

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
        # Use the wakeup command for that purpose
        self.write_cmd(CMD_WAKEUP)

    # define or redefine a logical channel
    def channel(self, channel, ainp, ainn=AINCOM, gain=None, rate=None):
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

    def standby(self):
        self.write_cmd(CMD_STANDBY)

    def wakeup(self):
        self.write_cmd(CMD_WAKEUP)

    def reset(self):
        self.write_cmd(CMD_RESET)
        self.previous_channel = None

    def calibration(self, mode=SELFCAL_GAIN | SELFCAL_OFFSET):
        if mode == SELFCAL_GAIN | SELFCAL_OFFSET:
            cmd = CMD_SELFCAL
        elif mode == SELFCAL_GAIN:
            cmd = CMD_SELFGCAL
        elif mode == SELFCAL_OFFSET:
            cmd = CMD_SELFOCAL
        else:
            return
        self.write_cmd(cmd)
        # Wait for the calibration to finish
        # Use the wakeup command for that purpose
        self.write_cmd(CMD_WAKEUP)


class ADS1255(ADS1256):

    NUM_INPUTS = 2
