# Industruino GSM/GPRS product documentation

*Current version: HW REV: 3.0*

1. [Configuration](#configuration)
2. [LED and buttons](#led-and-buttons)
3. [GSM/GPRS library options](#gsm/gprs-library-options)
4. [SD card](#sd-card)
5. [RS232](#rs232)


For datasheets, user manuals, pinout maps, see [industruino.com](https://industruino.com/page/techcentre)

![GSM/GPRS](https://industruino.com/website/image/product.template/65_eb07355/image)


# Configuration

See the module's user manual for configuration options of the serial selector switches. 
* One serial port is needed for communication with the SIM800 module 
* Another serial port can be used for the RS232 interface

The 2 hardware serial ports of the D21G can be used for these purposes, or a software serial can be used.
This module can also be used without IND.I/O or PROTO controller 'external mode', with external power, and control over the RS232 port.

If you are using an IND.I/O unit, please be aware that by default, the 'Serial' port (D0/D1) is connected to the IND.I/O's RS485 port. The current IND.I/O baseboards have a hardware switch to connect/disconnect the RS485 port to this Serial port (D0/D1). To use the 'Serial' port for RS232 or GSM, this switch should be in the upward position (away from the RS485 terminals).

# LED and buttons

A note on the LED on the GSM/GPRS module: this indicates the network status of the SIM800:
* fast blink (1s): searching for network
* slow blink (3s): registered on network
* very fast blink: GPRS enabled


Next to the LED there are 2 small buttons:
* left button is the RESET of the SIM800
* right button is the PWR button to switch on/off (connected to D6)


# GSM/GPRS library options

The Industruino GSM/GPRS module is based on the SIM800 and can be used with a variety of libraries. We recommend the [TinyGSM](https://github.com/vshymanskyy/TinyGSM) library, which comes with an Industruino example (in the 'more' folder). You can find other examples modified for Industruino [here](https://github.com/Industruino/democode). 

If you want to use the Adafruit FONA library, be aware that the Industruino GSM/GPRS module uses D6 as power on/off pin (it needs 1s HIGH), which is not the same as the Reset pin of the FONA library (pulled LOW for 0.1s in the library).

# SD card

The standard SD library included in the Arduino IDE works with the Ethernet module with minor modifications (CS is D4 as standard): for the D21G replace *Serial* by *SerialUSB*.

# RS232

The standard RS232 port can be used according to which serial port of the Industruino D21G it is connected to.
It can also be used to control the SIM800 module, in external mode.

