# ADS1256: MicroPython classes for the ADS1256 and ADS1255 ADC

This is a short and simple class for the ADS1256 and ADS1255 ADC. It supports reading
the ADC value, reading the temperature and configuring the various device
modes. In the following document, ADS1256 or ads1256 is used for both ADS1256 and ADS1255, unless
otherwise stated.

Tested with MicroPython ports for RP2040, RP2350, SAMD, i.MX RT (e.g. Teensy), STM32,  
ESP32, ESP8266, NRF52840, and W600. Approximate times for reading an ADC value:
- RP2040 at 125 MHz: 300 µs single read, max rate 7500 for burst mode.
- RP2350 at 150 MHz: 78 µs single read, max rate 7500 for burst mode.
- RP2040/2350 at 125/150 MHZ using the PIO: 40µs single read, max rate 30000 for burst mode.
- PYBD SF6 at 192 MHz: 110 µs single read, max rate 15000 for burst mode, SPI baudrate=1.5MHz.
- Teensy 4.1 at 600 MHz: 120 µs single read, max rate 15000 for burst mode, SPI baudrate=1.8MHz
- SAMD51 at 120 MHz: 100 µs single read, max rate 7500 for burst mode, SPI baudrate=1.8MHz
- SAMD21 at 48 MHz: 300µs single read, max rate 3750 for burst mode, SPI baudrate=1.8MHz
- ESP32 at 160 MHz: 200 µs single read, max rate 3750 for burst mode, SPI baudrate=1.8MHz
- ESP8266 at 80 MHz: 450 µs single read, max rate 1000 for burst mode, SPI baudrate=1.5MHz
- NRF52840: 240 µs single read, max rate 3750 for burst mode, SPI baudrate=1MHz
- W600 at 80 MHz: 400 µs single read, max rate 2000 for burst mode, SPI baudrate=1.5MHz


## Constructor with ads1256.py

### ads1256 = ADS1256(spi, cs, drdy[, gain=1, rate=1000])
### ads1255 = ADS1255(spi, cs, drdy[, gain=1, rate=1000])

This is the interface constructor. `spi` must be a SPI object configured for phase=1 and polarity=0.
mosi must be conncted to the ADS1256 DIN pin, miso to the ADS1256 DOUT pin and sck to the ADS1256 SCK
pin. The SPI baudrate must be lower than ADS1256_clock / 4. The typical ADS1256 clock frequency
is 7.68MHz, resulting a maximum SPI baudrate of 1.9 Mhz. 
`cs` and `drdy` are the pin objects of the GPIO pins used for the respective ADS1256 signals.  
When calling the constructor, a default channel 0 will be defined with AIN0 and AINCOM as inputs and
the gain and rate set as optional parameters. The ADS1256 will be configured with these settings.


## Constructor with ads1256_pio.py

### ads1256 = ADS1256(sck, din, dout, cs, drdy[, statemachine=0, gain=1, rate=1000)
### ads1255 = ADS1255(sck, din, dout, cs, drdy[, statemachine=0, gain=1, rate=1000])

This is the constructor to be used for the RP2 PIO implementation. The first five
arguments have to be Pin objects. `sck` is the clock pin, `din` the input data to the RP2, to be
connected with DOUT of the ADS1256, `dout` is the output data pin of the RP2 board, to be
connected to DIN of the ADS1256. 
`cs` is chip select and `drdy` the pin for the conversion pulse. Any Pin can be used,
it does not have to be SPI pins. Only cs and sck must be at consecutive numbers,
with cs being the lower number, like GPIO13 for cs and GPIO14 for sck

statemachine tells the number of the first statmachine. The supplied number is masked
to an even value. The driver uses two statemachines with consecutive id numbers.
Both fit into a single PIO.  

When calling the constructor, a default channel 0 will be defined with AIN0 and
AINCOM as inputs and the gain and rate set as optional parameters.
The ADS1256 will be configured with these settings.


## Methods

### ads1256.channel(id [, ainp=None, ainn=8, gain=1, rate=1000, buffered=False])

Define or change a logical channel used to read the data. This channel is not identical to
the ADC input pins. It defines the configuration of input pins, gain and sampling rate.

- id: A token used to identify the channel. It can be anything that can be used as
  key to a dictionary, e.g. a number or a string.
- ainp: The positive input of the channel. Valid values are 0-7 for ADS1256 and
0-1 for ADS1255 or 8 for the AINCOM signal.
- ainn: The negative input of the channel. Valid values are 0-7 for ADS1256 and
0-1 for ADS1255 or 8 for the AINCOM signal.
- gain: The gain set for the channel.
- rate: The sampling rate of the channel.
- buffered: Enable or disable input buffering, which increases the input
  impedance to 10 - 80 MOhm depending on the sample rate at the cost of a
  reduced input voltage range and increased power consumption.

If just the id is supplied as argument, a string with the channel settings is returned.

### ads1256.read(channel [, buffer])

Read and ADC value with the configuration set for the channel. If a buffer is supplied,
it will be filled with data in the read continuous mode until the buffer is filled.
The buffer must be an array of array.array() type 'i' (4 byte signed quantities).
The return value is the number of sampled values. The call will
return immediately, while the data is collected. One can use the
flag `data_acquired` of the ads1256 object to test, whether the data
acquisition is finished. The data in the buffer is **NOT** sign
corrected and in the correct range until `data_acquired` is True.  

If buffer is not supplied, a single read is performed and the value is returned.

If the channel used by read is different from the previous channel, the device
is reconfigured before reading and re-calibrated, if needed.

### ads1256.calibration([mode=3])

Request a self-calibration of the device. Argument values are:

1: Calibrate the gain.  
2: Calibrate the offset.  
3: Calibrate gain and offset.  

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
is provided by the argument `number`. For number == 1, a single int is returned as
register value, otherwise a bytearray.

### ads1256.sensor_detect(mode=0)

Set the sensor detect current according to the data sheet. Values for mode are:

0: Sensor Detect OFF  
1: Sensor Detect Current = 0.5μA  
2: Sensor Detect Current = 2μA  
3: Sensor Detect Current = 10μA  

### ads1256()

Perform a single read using the previously used channel.

### dig_pin = ads1256.gpio(pin, mode=IN | OUT | CLK [, div = 1 | 2 | 4])

Create a GPIO object which represent one of the four digital outputs. 
`pin` is the digital output number in the range 0-3.
`mode` defines the pin mode, which either 0 for output, 1 for input and 2 for board clock
output. The default is 1 for input. The CLK mode applies to pin 0 only.
'div' sets the divider for the clock, which is either 1, 2 or 4.
The default is 1. The div setting is applied only to pin 0 and ignored
for the other pin.

### dig_pin()

Return the value of a digital pin.

### dig_pin(value)

Set a digital output pin to 0 or 1, as given by `value`.


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
    NUM_INPUTS = 8 (ADS1256) or 2 (ADS1255)
    NUM_GPIO = 4 (ADS1256) or 2 (ADS1255)
    SELFCAL_GAIN = 1
    SELFCAL_OFFSET = 2
    OUT = 0
    IN = 1
    CLK = 2

## Notes

### Input Impedance

The ADS1256 is set up not to use the ADC input buffer. Then, the differential input impedance is
pretty low like about 150/gain kOhm at gain 1-32 and about 4,7kOhm at a gain of 64. In addition
with series resistors at a board that may lead to read() returning lower values that expected.
Enabling the ADC input buffer increases the input impedance to values between 10MOhm and 80MOhm,
depending on the data rate, but at the cost of reduced input voltage range. The ADC input buffer
can be enabled by setting bit 1 of the status register. Since auto-config is enabled,
it has to be a non-destructive write, like

    ads1256.write_reg(ads1256.REG_STATUS, ads1256.read_reg(ads1256.REG_STATUS) | 0x02)


### Comparison ADS1256 vs. HX711 vs. CS1237

Due to the supported gain of 64 the ADS1256 could be used in differential mode
as load cell digitizer, supporting up to 4 load cells at the same time.
The sensibility is a little bit lower, but the noise figures are way better. The
table below show the deviation from average for different settings. The
deviation slots are exclusive, e.g. the line with 0.0010% does *not* include
the numbers for 0.0003%. At rate 10, the ADS1256 performs much better.
At rate 1000 the ADS1256 numbers are similar to the HX711/CS1237 at a rate of 10.
The HX711 and CS1237 have similar noise figures.

                    CS1237      HX711     ADS1256    ADS1256    ADS1256  
                    Gain=64    Gain=64    Gain=64    Gain=64    Gain=64  
                    Rate=10    Rate=10    Rate=10   Rate=1000 Rate=30000 
    Deviation from
       Average                                            
       0,0003 %      0,085      0,105      0,664      0,123      0,039   
       0,0010 %      0,242      0,203      0,333      0,242      0,084   
       0,0030 %      0,485      0,421      0,003      0,512      0,199   
       0,0100 %      0,188      0,27         0        0,123      0,527   
       0,0300 %        0        0,001        0          0        0,151   
       0,1000 %        0          0          0          0          0     
        Beyond         0          0          0          0          0     



## Examples

    # Example for using the driver with a RP2 device
    # SPI at GPIO12 - GPIO15, DRDY at GPIO11
    #
    from machine import SPI, Pin, idle
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
    # Wait for the data to get acquired. Instead of waitint, other
    # tasks can be performed.
    while ads1256.data_acquired is False:
        idle()

    # Create a second channel with differential inputs AIN1 <-> AIN2,
    # gain=2, rate=100
    ads1256.channel(1, 1, 2, gain=2, rate=100)

    # Read a single value
    value = read(1)

## Example for using the driver with a RP2 device using the PIO state machine:

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

## Example for using the GPIO pins

    # use GPIO pin 2 as input
    p2 = ads1256.gpio(2, mode=ADS1256.IN)
    value = p2()

    # use GPIO pin 3 as output
    p3 = ads1256.gpio(3, mode=ADS1256.OUT)
    p3(1)

    # enable the board clock/4 output at GPIO pin 0
    p0 = ads1256.gpio(0, mode=ADS1256.CLK, div=4)

    # Print the gpio object properties:
    print(p2)
