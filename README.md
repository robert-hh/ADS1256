# WARNING: This driver is in a draft state and not yet tested.

It's published for discussion purposes. Tests will probably be done until
July 4, 2025.

# ADS1256: MicroPython classes for the ADS1256 and ADS1255 ADC

This is a short and simple class for the ADS1256 and ADS1255 ADC. It supports reading
the ADC value, reading the temperature and configuring the various device
modes. In the following document, ADS1256 or ads1256 is used for both ADS1256 and ADS1255, unless
otherwise stated.

Tested with MicroPython ports for RP2040 ~~, STM32, SAMD, i.MX RT (e.g. Teensy),  
ESP32, ESP8266, NRF52840 and W600~~. Approximate times for reading an ADC value:
- RP2040 at 125 MHz: 200 µs single read, max rate 2000 for burst mode.
- RP2350 at 150 MHz: 200 µs single read, max rate 3750 for burst mode.
- RP2340 at 125 MHZ using the PIO: 50µs single read, max rate 30000 for bust mode.
- ~~PYBD SF6 at 192 MHz: 250 µs~~
- ~~Teensy 4.1 at 600 MHz: 100 µs~~
- ~~SAMD51 at 120 MHz: 450 µs~~
- ~~SAMD21 at 48 MHz: 1.2 ms~~
- ~~ESP32 at 160 MHz: 900 µs~~
- ~~ESP8266 at 80 MHz: 1.2 ms~~
- ~~NRF52840: 800µs.~~
- ~~Renesas RA6M2 at 120 MHz: 600µs~~
- ~~W600 at 80 MHz: 900 µs~~ 

~~The nrf port has the problem that the device cannot be configured when using the IRQ based driver.
Otherwise the ADS1256 operates at it's default mode, which is gain=1, rate=1000, channel=0.
pin.irq() seems not to work at the Renesas port, at least not with the tested EV-RA6M2 board.
It can be configured, but refuses to work.~~


## Constructor

### ads1256 = ADS1256(spi, cs, drdy)
### ads1255 = ADS1255(spi, cs, drdy)

This is the interface constructor. spi must be a SPI object configured for phase=1 and polarity=0.
cs and drdy are the pin objects of the GPIO pins used for the respective signals.
When calling the constructor, a default channel 0 will be defined with AIN0 and AINCOM as inputs,
a gain of 1 and a rate of 1000.


### ads1256 = ADS1256(sck, din, dout, cs, drdy[, statemachine=0])
### ads1255 = ADS1255(sck, din, dout, cs, drdy[, statemachine=0])

This is the constructor to be used for the RP2 PIO implementation. The first five
arguments have to be Pin objects. sck is the clock pin, din the input data to the RP2, to be
connected with DOUT of the ADS1256, dout is the output data pin of the RP2 board, to be
connected to DIN of the ADS1256. 
cs is chip select and drdy the pin for the conversion pulse. Any Pin can be used,
it does not have to be SPI pins. Only cs and sck must be at consecutive numbers,
with cs being the lower number, like GPIO13 for cs and GPIO14 for sck

statemachine tells the number of the first statmachine. The driver uses two statemachines with consecutive ids. They fit both in a single PIO.


## Methods

### ads1256.channel(id, ainp, ainn=8, gain=1, rate=1000)

Define ro change a logical channel used to read the data. This channel is not identical to
the ADC input pins. It defines the configuration of input pins, gain and sampling rate.

- id: A token used to identify the channel. It can be anything that can be used as key to a dictionary, e.g. a number or a string.
- ainp: The positive input of the channel. Valid values are 0-7 for ADS1256 and 0-1 for ADS1255 or 8
for the AINCOM signal.
- ainn: The negative input of the channel. Valid values are 0-7 for ADS1256 and 0-1 for ADS1255 or 8
for the AINCOM signal.
- gain: The gain set for the channel.
- rate: The sampling rate of the channel.

### ads1256.read(channel [, buffer])

Read and ADC value with the configuration set for the channel. If buffer is supplied,
it will be filled with data in the read continuous mode until the buffer is filled.
The buffer must be an array of array.array() type 'i' (4 byte quantities). The return value
is the number of sampled values.
If buffer is not supplied, a single read is performed and the value is returned.

If the channel used by read is different from the previous channel, the device
is reconfigured before reading and re-calibrated, if needed.

### ads1256.calibration(mode)

Request a self-calibration of the device. Argument values:

1: calibrate the gain
2: calibrate the offset
3: calibrate gain and offset

Other values are ignored.

### ads1256.standby()

Send the standby command.

### ads1256.wakeup()

Send the wakeup command.

### ads1256.reset()

Send the reset command and calibrate the chip.

### ads1256.write_cmd(cmd)

Send the command cmd to the device.

### ads1256.write_reg(reg, data)

Write data to the register(s) starting at reg. data must be either a single integer or
an object with buffer protocol.

### ads1256.read_reg(reg, number=1)

Return the content of the register(s) starting at reg. The number of registers
is provided by number.

### ads1256()

Perform a single read using the previously used channel.

Other methods exist but are preferred for internal use.

## Constants

When constants are to be used outside the class, they must be prefixed
with the respective class name ADS1256 or ADS1255 or instance name.

### Registers

    REG_STATUS = 0x00
    REG_MUX = 0x01
    REG_ADCON = 0x02
    REG_DRATE = 0x03
    REG_IO = 0x04
    REG_OFC0 = 0x05
    REG_OFC1 = 0x06
    REG_OFC2 = 0x07
    REG_FSC0 = 0x08
    REG_FSC1 = 0x09
    REG_FSC2 = 0x0A

### Commands

    CMD_WAKEUP = 0x00
    CMD_RDATA = 0x01
    CMD_RDATAC = 0x03
    CMD_SDATAC = 0x0F
    CMD_RREG = 0x10
    CMD_WREG = 0x50
    CMD_SELFCAL = 0xF0
    CMD_SELFOCAL = 0xF1
    CMD_SELFGCAL = 0xF2
    CMD_SYSOCAL = 0xF3
    CMD_SYSGCAL = 0xF4
    CMD_SYNC = 0xFC
    CMD_STANDBY = 0xFD
    CMD_RESET = 0xFE

### Others

    AINCOM = 8
    NUM_INPUTS = 8 /ADS1256) or 2 (ADS1255)
    SELFCAL_GAIN = 1
    SELFCAL_OFFSET = 2

## Example

    # Example for using the driver with a RP2 device
    # SPI at GPIO12 - GPIO15, DRDY at GPIO11
    #
    from machine import SPI, Pin
    from array import array
    from ads1256 import ADS1256

    # Set up the interface. By default, channel 0 is created
    spi = SPI(1, sck=14, mosi=15, miso=12, phase=1, polarity=0)
    cs = Pin(13, Pin.OUT, value=1)
    drdy = Pin(11, Pin.IN)
    ads1256 = ADS1256(spi, cs, drdy)
    data  = array("i", bytearray(256*4))

    # read 256 values from the device
    num_values = ads1256.read(0, data)

    # Create a second channel with differential inputs AIN1 <-> AIN2,
    # gain=2, rate=100
    ads1256.channel(2, 1, 2, gain=2, rate=100)

    # Read a single value
    value = read(2)

Example for using the driver with a RP2 device using the PIO state machine:

    from machine import Pin, idle
    from array import array
    from ads1256_pio import ADS1256

    drdy = Pin(11, Pin.IN)
    din = Pin(12, Pin.IN)
    cs = Pin(13, Pin.OUT, value=1)
    sck = Pin(14, Pin.OUT, value=0)
    dout = Pin(15, Pin.OUT, value=0)
    ads=ADS1256(sck, din, dout, cs, drdy)

    ads.read(0)

    # Read 256 values into a buffer

    data  = array("i", bytearray(256*4))
    ads.channel(0, 0, 8, rate=15000)
    ads.read(0, data)
    while ads.data_acquired is False:
        idle()
    print(data)

