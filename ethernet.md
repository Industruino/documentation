# Industruino ETHERNET product documentation

*Current version: HW REV: 5.1*

The Industruino Ethernet module has several functions:
1. [Ethernet](#ethernet2-library) interface
2. [FRAM](#fram) memory
3. [SD card](#sd-card) interface


For datasheets, user manuals, pinout maps, see [industruino.com](https://industruino.com/page/techcentre)

![ETHERNET](https://industruino.com/website/image/product.template/10_1e844ce/image)


# Ethernet2 library

If you have the D21G Topboard please use Industruino version of the [Ethernet2 library](https://github.com/Industruino/Ethernet2). This is a fork of the original Ethernet2, with one important change: the SPI speed is set to 4MHz in this [file, line 25](https://github.com/Industruino/Ethernet2/blob/master/src/utility/w5500.cpp).

The Ethernet module is connected over SPI, so we also need the SPI library.

For D21G
```
#include <SPI.h>
#include <Ethernet2.h>
```

### MAC address

For a unique MAC-address, you can use the EUI-64 number stored in the D21G's RTC (MCP79402), for an example see this [sketch](https://github.com/Industruino/democode/blob/master/MACfromRTC/MACfromRTC.ino).


If you are using the Industruino Ethernet module with legacy 32u4 or 1286 topboard, use our [EthernetIndustruino library](https://github.com/Industruino/EthernetIndustruino).  For 32u4 / 1286:
```
#include <SPI.h>
#include <EthernetIndustruino.h>
```

# FRAM

The Ethernet module also includes FRAM: see this [demo sketch](https://github.com/Industruino/democode/tree/master/fram_D21G). 


For legacy 32u4/1286 the example is in the EthernetIndustruino library. If you want to use the FRAM together with the Ethernet, there is no need to include the SPI settings as in the FRAM example, because this is taken care of in the Ethernet library. So you can just include the 2 libraries with the above 2 lines; DO NOT include the SPI settings as in the FRAM example:
```
//Setting up the SPI bus -- NO NEED when using the EthernetIndustruino library
SPI.begin();
SPI.setDataMode(SPI_MODE0); 
SPI.setBitOrder(MSBFIRST);
SPI.setClockDivider(SPI_CLOCK_DIV2);
```

# SD card

The standard SD library included in the Arduino IDE works with the Ethernet module with minor modifications (CS is D4 as standard): for the D21G replace *Serial* by *SerialUSB*. And you may have to add these lines to ensure the CS of Ethernet, FRAM, and SD are `HIGH`:
```
pinMode(10, OUTPUT); // Ethernet CS
pinMode(6, OUTPUT);  // FRAM CS
pinMode(4, OUTPUT);  // SD card CS
digitalWrite(10, HIGH);
digitalWrite(6, HIGH);
digitalWrite(4, HIGH);
```

