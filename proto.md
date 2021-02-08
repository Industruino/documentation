# Industruino PROTO product documentation

*Current version: HW: 2.2 FW: 3.0*

1. Setting up the Arduino [IDE](indio.md#setting-up-the-arduino-ide)
2. [D21G](indio.md#d21g-board-specific-features) specific features
3. PROTO digital and analog [GPIO features](#proto-gpio) 
4. [IDC](indio.md#idc-expansion-port-pinout) expansion port pinout
5. Libraries for use with PROTO:
    * [UC1701](indio.md#uc1701-library) - LCD display default option
    * [U8G and U8G2](indio.md#u8g-and-u8g2-libraries) - LCD display advanced option
6. [RTC](indio.md#rtc)
7. [EEPROM](indio.md#eeprom)
8. [WDT](indio.md#watchdog)
9. [TFTP](indio.md#tftp) - upload sketch over Ethernet
10. [Security](indio.md#security-bit) - protect your code


For datasheets, user manuals, pinout maps, see [industruino.com](https://industruino.com/page/techcentre)

![PROTO](https://industruino.com/website/image/product.template/8_10e4d35/image)


# PROTO GPIO

The Industruino PROTO D21G is designed to work with 5V GPIO, while the MCU is running on 3.3V. The level shifters are bidirectional and support analog signals. Note that the output is at 3.3V by default. 

The pins are connected directly to the MCU so we can use the standard Arduino functions.

Note that the PROTO casing may have legacy labels for analog inputs, Please make sure to check the D21G pinout map for available analog input pins.


## Digital output
Default HIGH signal is 3.3V. To get a digital output on the PROTO D21G at 5V level you should add a pull-up resistor to 5V (10K resistor should be sufficient).


## Digital input
The `INPUT_PULLUP` mode defaults to 3.3V, we can use a pull-up resistor of 10K if we need 5V.

External interrupts work on all pins except 11 and 17, with the standard `attachInterrupt(pin, ISR, mode);` instruction. More details at [Arduino reference](https://www.arduino.cc/en/Reference/AttachInterrupt).


## Analog output
The PROTO D21G has one 10-bit DAC available on pin D18; to use it we need to refer to it as `DAC0`. The range is 0 to 3.3V. Default resolution is 8-bit. These lines will set the output to 1.65V (50% of 3.3V) at maximum resolution of 12-bit.
```
analogWriteResolution(12);
analogWrite(DAC0, 2048);
```

PWM output is available on a range of pins (default frequency is 730Hz), see the PROTO D21G pinout map.
More details at [Arduino reference](https://www.arduino.cc/en/Reference/AnalogWriteResolution).


## Analog input
Please note that the pin numbers for analog input follow the digital pin numbers, i.e. A4 is on D4, A5 on D5 etc, as mentioned in the PROTO D21G pinout map. Your PROTO casing may have different labels, based on the legacy boards 32u4 and 1286. Please make sure to check the D21G pinout map for available analog input pins.

Analog read resolution default is 10-bit, returning values 0-1023.
We can change this to 12-bit with `analogReadResolution(12);` returning values 0-4095.

More details at [Arduino reference](https://www.arduino.cc/en/Reference/AnalogReadResolution).

The analog reference voltage is 3.3V by default, and can be changed to these options:
```
  analogReference(AR_DEFAULT);     // 3V3 - may not be very stable
  analogReference(AR_INTERNAL);    // 2V23 - may not be very stable
  analogReference(AR_EXTERNAL);    // put on AREF, max 3V3
  analogReference(AR_INTERNAL1V0);  // recommended on PROTO D21G
  analogReference(AR_INTERNAL1V65); // recommended on PROTO D21G
  analogReference(AR_INTERNAL2V23); // may not be very stable
```
