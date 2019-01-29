# Industruino 4-20mAker product documentation

*Current version: HW REV: ??, FW REV: 3??*

1. Setting up the Arduino [IDE](#setting-up-the-arduino-ide)
2. [wiring](#wiring-example) example
3. [SAML](#saml-board-specific-features) specific features
4. [GPIO](#gpio) pinout
5. [WDT](#wdt)

For datasheets, user manuals, pinout maps, see [industruino.com](https://industruino.com/page/techcentre)

![420maker](https://industruino.com/website/image/product.template/2_cd53b8d/image)

# Setting up the Arduino IDE

D21G is compatible with IDE from 1.6.12 with automatic install via board manager: 
* In *File > Preferences > Additional Boards Manager URLs:* add https://static.industruino.com/downloads/code/IndustruinoCores/IndustruinoSAMD/pkgdef/package_industruino_samd_index.json 
* Enter the Board Manager via *Tools > Board* and search for 'industruino'
* Install the Industruino SAML package
* *Industruino D21G* will show up in the Boards list in *Tools > Board* in section *Industruino SAMD*
* Windows driver (if needed): https://static.industruino.com/downloads/drivers/drivers-industruino-windows-0.0.1.zip


# Wiring example

When using the 4-20mAker with the IND.I/O analog input channels, this is the default wiring for the current loop, using the same power supply for both devices:
* IND.I/O V+ (12/24V) <-> 4-20mAker V+
* 4-20mAker V- <-> IND.I/O analog input channel
* IND.I/O V- (GND) <-> IND.I/O analog GND

If using a separate power supply for each device:
* analog PSU V+ <-> 4-20mAker V+
* 4-20mAker V- <-> IND.I/O analog input channel
* IND.I/O analog GND <-> analog PSU V-

# SAML board specific features

* Serial ports on SAML:
  * SerialUSB for USB (Serial Monitor)
  * Serial for hardware serial on D0/D1

# GPIO


# WDT

We can use the WDT functions available in the SAML core.
To initialise, configure, and enable the WDT:
```
  WDT->CTRLA.reg = 0;                       // init
  WDT->CONFIG.reg = WDT_CONFIG_PER_CYC2048; // set timeout period based on 1.024kHz clock, from 8ms to 16sec, CYC = 1ms
  // options: WDT_CONFIG_PER_CYC8 WDT_CONFIG_PER_CYC16 WDT_CONFIG_PER_CYC32 WDT_CONFIG_PER_CYC64 WDT_CONFIG_PER_CYC128
  // WDT_CONFIG_PER_CYC256 WDT_CONFIG_PER_CYC512 WDT_CONFIG_PER_CYC1024 WDT_CONFIG_PER_CYC2048 WDT_CONFIG_PER_CYC4096
  // WDT_CONFIG_PER_CYC8192 WDT_CONFIG_PER_CYC16384

  WDT->CTRLA.reg = WDT_CTRLA_ENABLE;        // enable
```
To clear the WDT:
```
  WDT->CLEAR.reg = WDT_CLEAR_CLEAR_KEY;   // reset the WDT: without this line, the WDT would cause a reset after 2 seconds
```


* `RESET` button is located behind the green screw terminal; double tap resets the MCU
