# Industruino GSM/GPRS product documentation

*Current version: HW REV: 3.0*

1. [Configuration](#configuration)
2. [Library options](#gsm/gprs-library-options)


For datasheets, user manuals, pinout maps, see [industruino.com](https://industruino.com/page/techcentre)

![GSM/GPRS](https://industruino.com/website/image/product.template/65_eb07355/image)


# Configuration

See the module's user manual for configuration options of the serial selector switches. If you are using an IND.I/O unit, please be aware that by default, the 'Serial' port (D0/D1) is connected to the IND.I/O's RS485 port. The current IND.I/O baseboards have a hardware switch to connect/disconnect the RS485 port to this Serial port (D0/D1). To use the 'Serial' port for RS232 or GSM, this switch should be in the upward position (away from the RS485 terminals).


# GSM/GPRS library options

The Industruino GSM/GPRS module is based on the SIM800 and can be used with a variety of libraries. We recommend the [TinyGSM](https://github.com/vshymanskyy/TinyGSM) library, and you can find several code examples modified for Industruino [here](https://github.com/Industruino/democode). 

If you want to use the Adafruit FONA library, be aware that the Industruino GSM/GPRS module uses D6 as power on/off pin (it needs 1s HIGH), which is not the same as the Reset pin of the FONA library (pulled LOW for 0.1s in the library).

