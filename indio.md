# Industruino IND.I/O product documentation

Current version:
* HW REV: 5.2
* FW REV: 3.0

* [Setting up the Arduino IDE](setting-up-the-arduino-ide)
* [D21G specific features](d21g-board-specific-features)
* Libraries for use with IND.I/O:
  * [UC1701](#uc1701) - LCD display
  * [U8G and U8G2](#u8g-and-u8g2) - LCD display

# Setting up the Arduino IDE

D21G is compatible with IDE from 1.6.12 with automatic install via board manager: 
* In *File > Preferences > Additional Boards Manager URLs:* add https://static.industruino.com/downloads/code/IndustruinoCores/IndustruinoSAMD/pkgdef/package_industruino_samd_index.json 
* Enter the Board Manager via *Tools > Board* and search for 'industruino'
* Install the Industruino package
* *Industruino D21G* will show up in the Boards list in *Tools > Board* in section *Industruino SAMD*
* Windows driver (if needed): https://static.industruino.com/downloads/drivers/drivers-industruino-windows-0.0.1.zip


# D21G board specific features

* RESET button on D21G: when your computer does not recognise the USB port anymore, reset the D21G by pushing the reset button on the bottom of the topboard TWICE (you will have to remove the casing for this). The LCD backlight will start fading in&out, indicating the bootloader is active and ready for a new upload.
* Serial ports on D21G: 
  * `SerialUSB` for USB (Serial Monitor)
  * `Serial` for hardware serial on D0/D1
  * `Serial1` for hardware serial on D10/D5
* LCD backlight on D26 
* Hardware Timers of the Industruino D21G are similar to the Arduino Zero, see this [blog post](https://industruino.com/blog/our-news-1/post/d21g-timer-library-33).
* [D21G PROTO kit](#proto): notes on digital and analog input/output 


The pre-loaded demo code is available at https://github.com/Industruino/democode


*Legacy boards 32u4 and 1286 specific features:*
* 32u4 is compatible with all IDE versions (board type: Leonardo)
* 1286 is compatible with IDE up to 1.6.5 (manual install of board definitions: follow instructions in the support file package)
* Serial ports on 32u4 and 1286: `Serial` for USB and `Serial1` for hardware serial on D0/D1
* LCD backlight is on D13 for 32u4 and D26 for 1286


# Industruino libraries

Arduino libraries to use with Industruino products:
* [UC1701](#uc1701) - LCD display
* [U8G and U8G2](#u8g-and-u8g2) - LCD display
* [Ethernet](#ethernet) - Ethernet module
  * [FRAM](#fram)
  * [SD card](#sd-card)
  * [IDC pinout](#idc-pinout)
* [Indio](#indio) - IND.I/O kit only
  * [digital I/O](#digital-io) 
  * [analog input](#analog-input)
  * [analog output](#analog-output)
  * [RS485](#rs485)
  * [interrupts](#interrupts)
  * [calibration](#calibration)
* [RTC](#rtc) - D21G only
* [EEPROM](#eeprom)
* [WDT](#watchdog)
* [GSM/GPRS](#gsmgprs) - GSM/GPRS module
* [Modbus](#modbus) - RTU and TCP

# UC1701
You can download the library from within the Arduino libraries manager or from [this repository](https://github.com/Industruino/UC1701).
The Industruino LCD is connected over SPI to the pins D19,20,21,22 (and the backlight to D13 on 32u4 boards and D26 on 1286 boards). We suggest you use either of these 2 libraries:
* our customised UC1701 library (available in this repository): easy to use, relatively small (it is also used in the Industruino pre-installed demo sketches). it is largely compatible with the popular PCD8544 (Nokia screens). familiar syntax:  
```
lcd.begin();  
lcd.clear();  
lcd.setCursor(1, 1);  
lcd.print("hello Industruino!");
```

# U8G and U8G2
[U8G](https://github.com/olikraus/u8glib) is a popular display library with many fonts and graphics, consuming more memory than the basic UC1701 above. Use this constructor:
```
U8GLIB_MINI12864 u8g(21, 20, 19, 22);	// SPI Com: SCK = 21, MOSI = 20, CS = 19, A0 = 22
```

[U8G2](https://github.com/olikraus/u8g2) is the new improved version of the above U8G library, largely compatible, with 3 [buffer](https://github.com/olikraus/u8g2/wiki/u8g2setupcpp#buffer-size) options (speed vs memory); the page_buffer examples work. 
Use this constructor for software SPI:
```
U8G2_UC1701_MINI12864_1_4W_SW_SPI u8g2(U8G2_R2, 21, 20, 19, 22);   // rotation, clock, data, cs, dc
```
Hardware SPI: may also work on D21G Topboard: Change SPI interface #define from "SPI" to "SPI1" in src/U8x8lib.cpp line #49).
Use this constructor for hardware SPI:
```
U8G2_UC1701_MINI12864_F_4W_HW_SPI u8g2(U8G2_R2, /* cs=*/ 19, /* dc=*/ 22);
```

# Ethernet
If you have the D21G Topboard please use Industruino version of the [Ethernet2 library](https://github.com/Industruino/Ethernet2). This is a fork of the original Ethernet2, with one important change: the SPI speed is set to 4MHz in this [file, line 25](https://github.com/Industruino/Ethernet2/blob/master/src/utility/w5500.cpp).


For a unique MAC-address, you can use the EUI-64 number stored in the D21G's RTC (MCP79402), for an example see this [sketch](https://github.com/Industruino/democode/blob/master/MACfromRTC/MACfromRTC.ino).


If you are using the Industruino Ethernet module with 32u4 or 1286 Topboard use our [EthernetIndustruino library](https://github.com/Industruino/EthernetIndustruino). 
Both libraries are based on the standard Arduino Ethernet library and support all same commands. The Ethernet module is connected over SPI, so we also need the SPI library.

For D21G
```
#include <SPI.h>
#include <Ethernet2.h>
```
For 32u4 / 1286
```
#include <SPI.h>
#include <EthernetIndustruino.h>
```

### FRAM

The Ethernet module also includes FRAM; for D21G see this [demo sketch](https://github.com/Industruino/democode/tree/master/fram_D21G). 
For 32u4/1286 the example is in the EthernetIndustruino library. If you want to use the FRAM together with the Ethernet, there is no need to include the SPI settings as in the FRAM example, because this is taken care of in the Ethernet library. So you can just include the 2 libraries with the above 2 lines; DO NOT include the SPI settings as in the FRAM example:
```
//Setting up the SPI bus -- NO NEED when using the EthernetIndustruino library
SPI.begin();
SPI.setDataMode(SPI_MODE0); 
SPI.setBitOrder(MSBFIRST);
SPI.setClockDivider(SPI_CLOCK_DIV2);
```

### SD card

The standard SD library included in the Arduino IDE works with the Ethernet module with minor modifications (CS is D4 as standard): for the D21G replace *Serial* by *SerialUSB*. And you may have to add these lines to ensure the CS of Ethernet, FRAM, and SD are HIGH:
```
pinMode(10, OUTPUT); // Ethernet CS
pinMode(6, OUTPUT);  // FRAM CS
pinMode(4, OUTPUT);  // SD card CS
digitalWrite(10, HIGH);
digitalWrite(6, HIGH);
digitalWrite(4, HIGH);
```

### IDC pinout

When using the Ethernet module with the Industruino PROTO, it is important to be aware of the I/O pins it is using, and which should not be used for other I/O functions; see below table. The IDC pins can also be used to connect other 5V devices to the PROTO and IND.I/O; they are accessible with standard `pinMode`, `digitalRead` and `digitalWrite` commands (different from the IND.I/O digital I/O channels, see below). Please note the D21G works at 3.3V: if you need 5V output, use a 10K pull-up resistor. For more details, look [here](https://github.com/Industruino/libraries#proto).

| IDC pin number	| Module function	| Arduino pin	| Default connected	| Required for standard functions |
| --- | --- | --- | --- | --- |
| 1	|MISO	| D14	| yes	| yes |
| 2	| +5V	| +5V	| yes	| yes |
| 3	| SCLK	| D15/SCLK	| yes	| yes |
| 4	| MOSI	| D16/MOSI	| yes	| yes |
| 5	| Ethernet CS	| D10/RX1	(Serial1) | yes	| yes |
| 6	| GND	| GND	| yes	| yes |
| 7	| SD CS	| D4	| yes	| yes |
| 8	| Ethernet ext reset	| D5/TX1	(Serial1) | yes	| no |
| 9	| Ethernet IRQ	| D7	| yes	| no |
| 10	| FRAM CS	| D6	| yes	| yes |
| 11	| /	| D0/RX	(Serial) | PROTO: no <br> IND.I/O: depends on switch position | PROTO: no <br> IND.I/O: RS485/IDC|
| 12	| /	| D1/TX	(Serial) | PROTO: no	<br> IND.I/O: depends on switch position | PROTO: no <br> IND.I/O: RS485/IDC|
| 13	| /	| D2/SDA	| PROTO: no	<br> IND.I/O: yes | used for I2C RTC|
| 14	| / | D3/SCL |	PROTO: no	<br> IND.I/O: yes | used for I2C RTC|


# Indio

##### Important notes:    
##### The digital and analog I/O will only work when Vin power (6.5-32V) is supplied to the Indio baseboard via the green screw connectors. When only USB power is connected, none of the digital or analog channels, nor the RS485, will work.
##### It is important to power down all systems (Industruino, sensors/actuators) before making connections to the Industruino.

You can download the library from within the Arduino libraries manager or from [this repository](https://github.com/Industruino/Indio).

If you are using the Industruino IND.I/O product, you will need this library to access the I/O channels. The pins on the IDC expansion connector, the backlight pin, and the membrane panel buttons pin(s) should still be accessed in the usual way, not using the Indio library; the Indio library is only for the external I/O channels available on the green screw connectors.

The Indio board uses an I2C expander for the I/O channels so we also need the Wire library.
```
#include <Indio.h>
#include <Wire.h>
```

### DIGITAL IO 

Configuration:
```
Indio.digitalMode(1,INPUT);       // Set CH1 as an input
Indio.digitalMode(7,OUTPUT);      // Set CH7 as an output
```
Read/write:
```
Indio.digitalRead(1);             // Read CH1
Indio.digitalWrite(7,LOW);        // Set CH7 to low (0V)
```
Notes: 
* INPUT logic HIGH voltage is >11V and logic LOW is <3V
* INPUT maximum trigger frequency is 10kHz (see below for interrupts)
* OUTPUT maximum switching frequency is 400Hz

### ANALOG INPUT

##### Important note:    
##### The analog I/O section is galvanically isolated from the digital I/O section and the microcontroller section, to allow a separate power supply in the analog section for optimal accuracy. In case your analog sensors/actuators are on the same power supply as the digital section (Vin 12/24V) you have to connect the analog GND to the digital GND.

Configuration of resolution (to be done BEFORE configuration of the input mode):
```
Indio.setADCResolution(16);       // Set the ADC resolution
                                  // Choices are 12bit@240SPS, 14bit@60SPS, 16bit@15SPS and 18bit@3.75SPS.
```

Configuration of input mode:
```
Indio.analogReadMode(1, V10);     // Set Analog-In CH1 to 10V mode (0-10V).
Indio.analogReadMode(1, V10_p);   // Set Analog-In CH1 to % 10V mode (0-10V -> 0-100%).
Indio.analogReadMode(1, V5);      // Set Analog-In CH1 to 5V mode (2x gain enabled on ADC).
Indio.analogReadMode(1, V5_p);    // Set Analog-In CH1 to 5V mode (0-5V -> 0-100%).
Indio.analogReadMode(1, V10_raw); // Set Analog-In CH1 to 10V mode and read raw ADC value (0-10V -> 0-4096).

Indio.analogReadMode(1, mA);      // Set Analog-In CH1 to mA mode (0-20mA).
Indio.analogReadMode(1, mA_p);    // Set Analog-In CH1 to % mA mode (4-20mA -> 0-100%)
Indio.analogReadMode(1, mA_raw);  // Set Analog-In CH1 to mA mode and read raw ADC value (0-20mA -> 0-4096).
```  

Note: if you want to change the resolution in your program, you have to repeat the above input mode configuration after changing the resolution.


Read:
```
Indio.analogRead(1);              //Read Analog-In CH1 (output depending on selected mode as above)
```
Please note that the output of the Indio.analogRead() in RAW mode is not of the type INTEGER, but FLOAT. The output range is fixed from 0 to 4096 for all resolutions, but only in 12-bit mode this returns integers; for higher resolution the measurements are floating point numbers.

### ANALOG OUTPUT

##### Important note:   
#####The analog I/O section is galvanically isolated from the digital I/O section and the microcontroller section, to allow a separate power supply in the analog section for optimal accuracy. In case your analog sensors/actuators are on the same power supply as the digital section (Vin 12/24V) you have to connect the analog GND to the digital GND.

The DAC resolution is 12 bits.

Configuration of output mode:
```
Indio.analogWriteMode(1, V10);      // Set Analog-Out CH1 to 10V mode (0-10V).
Indio.analogWriteMode(1, V10_p);    // Set Analog-Out CH1 to % 10V mode ( 0-100% -> 0-10V).
Indio.analogWriteMode(1, V10_raw);  // Set Analog-Out CH1 to 10V mode and take raw DAC value (0-4096 -> 0-10V).

Indio.analogWriteMode(1, mA);       // Set Analog-Out CH1 to mA mode (0-20mA).
Indio.analogWriteMode(1, mA_p);     // Set Analog-Out CH1 to % mA mode (0-100% -> 4-20mA).
Indio.analogWriteMode(1, mA_raw);   // Set Analog-Out CH1 to mA mode and take raw DAC value (0-4096 -> 0-20mA).   
```
Write (examples corresponding to above configuration):
```
Indio.analogWrite(1, 2.67, true);   //Set CH1 to 2.67V ("true" will write value to EEPROM of DAC, restoring it after power cycling).
Indio.analogWrite(1, 33.5, true);   //Set CH1 to 33.5% (approx 3.685V)
Indio.analogWrite(1, 1000, true);   //Set CH1 DAC to integer value 1000 (approx 2.685V).

Indio.analogWrite(1, 10.50, false); //Set CH1 to 10.5mA ("false" will not write value to EEPROM of DAC).
Indio.analogWrite(1, 75, true);     //Set CH1 to 75% (approx 16mA).
Indio.analogWrite(1, 2048, true);   //Set CH1 DAC to integer value 2048 (approx 10.5mA).
```

### INTERRUPTS


#### INTERRUPTS on the D21G topboard

The interrupt pin of the expander on the 12/24V digital side is connected to D8 (=INT8) pin of the D21G topboard. This pin will trigger when a change on any of the 8 input or output channels occurs. By default all input channels have this interrupt enabled. However, this can be configured per channel:
```
Indio.digitalMode(1, INPUT);         // default behaviour: interrupt enabled
Indio.digitalMode(2, INPUT_MASKED);  // pin change will not trigger the general interrupt
```

If more than 1 channel needs to be detected by the interrupt, a flag can be set inside the interrupt service routine, and then any pin change can be checked inside the main loop, as discussed in this forum post https://industruino.com/forum/help-1/question/multiple-channels-interrupts-on-32u4-topboard-205

This code example (for D21G topboard) shows a counter on the LCD for each edge on CH1 (without debounce), and prints to the Serial Monitor when a membrane button is pressed. Please note it is not good practice to use Serial prints in ISRs - this is for demo only.
```
#include <Indio.h>
#include <Wire.h>

#include <UC1701.h>
static UC1701 lcd;

volatile int counter = 0;

void setup() {

  SerialUSB.begin(9600);
  lcd.begin();

  Indio.digitalMode(1, OUTPUT); //  Clear CH1 to LOW
  Indio.digitalWrite(1, LOW);  // Clear CH1 to LOW
  Indio.digitalMode(1, INPUT); // Set CH1 as an input

  attachInterrupt(8, count, FALLING);       // D8 attached to the interrupt of the expander

  attachInterrupt(24, enter, FALLING);      // D24 is the Enter button (pull-up)
  attachInterrupt(25, up, FALLING);      // D25 is the Up button (pull-up)
  attachInterrupt(23, down, FALLING);      // D23 is the Down button (pull-up)

}

void count() {
  SerialUSB.println("trigger");
  counter++;
}

void enter() {
  SerialUSB.println("enter");
}

void up() {
  SerialUSB.println("up");
}

void down() {
  SerialUSB.println("down");
}

void loop() {
  lcd.setCursor(1, 3);
  lcd.print(counter);
  delay(100);
}
```

#### INTERRUPTS on the legacy 32u4 topboard

The interrupt pin of the expander on the 12/24V digital side is connected to pin D8 (PCINT4) of the 32u4 topboard. This pin will trigger when a change on any of the 8 input or output channels occurs. If more than 1 channel needs to be detected by the interrupt, a flag can be set inside the interrupt service routine, and then any pin change can be checked inside the main loop, as discussed in this forum post https://industruino.com/forum/help-1/question/multiple-channels-interrupts-on-32u4-topboard-205

This code example (for 32u4 topboard) shows a counter on the LCD for each rising edge on CH1 (without debounce).
```
#include <Indio.h>
#include <Wire.h>

#include <UC1701.h>
static UC1701 lcd;

volatile int risingEdge = 1;
volatile int counter = 0;

void setup() {

  Serial.begin(9600);
  lcd.begin();
  Indio.digitalMode(1, OUTPUT); //  Clear CH1 to LOW
  Indio.digitalWrite(1, LOW); 
  Indio.digitalMode(1, INPUT); // Set CH1 as an input

  // Enable Pin Change Interrupt
  PCMSK0 = (1 << PCINT4);
  PCICR = (1 << PCIE0);

  // Global Interrupt Enable
  sei();
}

ISR (PCINT0_vect)
{
  if (risingEdge == 1) {
    risingEdge = 0;
    Serial.println("trigger");
    counter++;
  }

  else {
    risingEdge = 1;
  }
}

void loop() {
  lcd.setCursor(1,3);
  lcd.print(counter);
  delay(100);
}
```

#### INTERRUPTS on the legacy 1286 topboard

The interrupt pin of the expander on the 12/24V digital side is connected to the INT7 pin of the 1286 topboard. This pin will trigger when a change on any of the 8 input or output channels occurs, and we can specify `CHANGE`, `RISING`, `FALLING`, `LOW` (note this pin is inverted: a change from LOW to HIGH on the digital channel triggers `FALLING`). If more than 1 channel needs to be detected by the interrupt, a flag can be set inside the interrupt service routine, and then any pin change can be checked inside the main loop, as discussed in this forum post https://industruino.com/forum/help-1/question/multiple-channels-interrupts-on-32u4-topboard-205

This code example (for 1286 topboard) shows a counter on the LCD for each rising edge on CH1 (without debounce).
```
#include <Indio.h>
#include <Wire.h>

#include <UC1701.h>
static UC1701 lcd;

volatile int counter = 0;

void setup() {

  Serial.begin(9600);
  lcd.begin();
  Indio.digitalMode(1, OUTPUT); //  Clear CH1 to LOW
  Indio.digitalWrite(1, LOW);  // 
  Indio.digitalMode(1, INPUT); // Set CH1 as an input

  attachInterrupt(7, count, CHANGE);       // INT7 attached to the interrupt of the expander
                                           // this is not D7
}

void count() {
  Serial.println("trigger");
  counter++;
}

void loop() {
  lcd.setCursor(1, 3);
  lcd.print(counter);
  delay(100);
}
```

The 1286 topboard also allows us to attach an interrupt to the membrane panel buttons: its button inputs are connected to pin change interrupts PCINT 4, 5, and 6 for buttons Down, Enter, and Up respectively.

Below demo sketch will show "waiting" on the LCD; when you press the "Enter" button an interrupt will be triggered and "Enter pressed" will show on the LCD for one second. To attach the interrupt to the "Up" or "Down" button change "PCINT5" in line `PCMSK0 = (1 << PCINT6);` to PCINT4 or PCINT6.

```
#include <UC1701.h>
static UC1701 lcd;

volatile int modeFlag = 0;

void setup() {
  
    lcd.begin(); //enable LCD
    // Enable Pin Change Interrupt 5 = Enter button
    PCMSK0 = (1 << PCINT5);
    PCICR = (1 << PCIE0);
 
    // Global Interrupt Enable
   sei();    
}

ISR (PCINT0_vect)
{    
    modeFlag = 1;    
}

void loop() { 
  
  lcd.setCursor(0, 0);
  lcd.print("waiting       ");
  
  if (modeFlag == 1) {
    lcd.setCursor(0, 0);
    lcd.print("Enter pressed");
    delay(1000); 
    modeFlag = 0;
  }
}
```

### RS485

RS485 is a popular industrial network standard and the INDIO features a half duplex RS485 transceiver. It is often used with the Modbus protocol, so-called Modbus RTU (as opposed to Modbus TCP which uses Ethernet). The Master unit sends out periodic requests over the network, and Slaves receive and reply.

Hardware specifics for RS485 on the INDIO:
* Serial connection = Serial (on the current D21G, Serial1 on old 32u4 and 1286 topboards)
* TxEnablePin = D9

Note: the current IND.I/O baseboards have a hardware switch to connect/disconnect the RS485 port and the Serial port (D0/D1). To use the RS485, this switch should be in the bottom position (towards the RS485 terminals).

Note: the INDIO board has 3 jumpers for RS485 termination resistors, for details see [here](https://industruino.com/blog/our-news-1/post/modbus-rtu-master-and-slave-14).

We can use the [SimpleModbusMaster and SimpleModbusSlave libraries](https://drive.google.com/folderview?id=0B0B286tJkafVYnBhNGo4N3poQ2c&usp=drive_web&tid=0B0B286tJkafVSENVcU1RQVBfSzg#list) (versions V2rev2 and V10 respectively) to establish communication over RS485 between 2 or more INDIOs, with one acting as the Master and the other one(s) as the Slave(s). This is one way of expanding the Industruino's number of I/O pins.

Basic configuration of the above Modbus RTU libraries:
```
#define baud       9600   // use 9600 on D21G, higher rates may not work
#define timeout    1000
#define polling    20    // the scan rate
#define retry_count 10
// used to toggle the receive/transmit pin on the driver
#define TxEnablePin 9                                                           // INDUSTRUINO RS485

modbus_configure(&Serial, baud, SERIAL_8N2, timeout, polling, retry_count, TxEnablePin, packets, TOTAL_NO_OF_PACKETS, regs);
```
For more information see the examples on our blog, e.g. [Modbus RTU between 2 INDIOs](https://industruino.com/blog/our-news-1/post/modbus-rtu-master-and-slave-14).

### CALIBRATION

Please find the calibration data array inside the Indio.cpp library file (as below), together with this explanation on how to perform the calibration. The library is preloaded with calibration data but characteristics are board specific thus reading with standard calibration data might be off. You are advised to update the calibration arrays in your own Indio.cpp file. We provide a sketch to assist with this [here](https://github.com/Industruino/Indio/tree/master/examples/Indio_AnalogCalibration).

Analog Calibration method:
* Analog IN, 0-10V: In your sketch set the analogReadMode to V10_raw. Feed a known voltage between 0-10V into 1 of the 4 input channels. Read the corresponding raw ADC value from the serial terminal and write it down. Do this at two voltage levels, for example 2.5V and 7.5V. Enter the recorded raw ADC value and corresponding voltage (in mV) into the calibration array below. Repeat this for all 4 channels.
* Analog IN, 4-20mA: In your sketch set the analogReadMode to mA_raw. Feed a known current between 0-20mA into 1 of the 4 input channels. Read the corresponding raw ADC value from the serial terminal and write it down. Do this at two current levels, for example 10000uA and 20000uA. Enter the recorded raw ADC value and corresponding voltage (in uA) into the calibration array below. Repeat this for all 4 channels. As a current source you can for example use a 500 Ohm resistor in series with a 0-20V voltage source and multimeter in series.
* Analog OUT, 0-10V: In your sketch set the analogWriteMode to V10_raw. Set the analogWrite value to anything between 0-4096. Read the corresponding output voltage on a multimeter and write it down. Do this at two DAC output values, for example 1000 and 3600. Enter the recorded raw DAC value and corresponding voltage (in mV) into the calibration array below. Repeat this for all 2 channels.
* Analog OUT, 4-20mA: In your sketch set the analogWriteMode to mA_raw. Set the analogWrite value to anything between 0-4096. Read the corresponding output current on a multimeter (connect multimeter between output and ground, best with a 100-500 Ohm resistor in series) and write it down. Do this at two DAC output values, for example 1000 and 3600. Enter the recorded raw DAC value and corresponding current (in uA) into the calibration array below. Repeat this for all 2 channels.

```
//Calibration data array for ADC, 0-10V mode

const int ADC_voltage_low_raw[5] = {0,384,384,382,386};//raw ADC value for low reference calibration point. Ignore first 0, subsequent is CH1-CH4 from left to right.
const int ADC_voltage_low_mV[5] = {0,1006,1006,1006,1006}; //corresponding mV for low reference calibration point. Ignore first 0, subsequent is CH1-CH4 from left to right.

const int ADC_voltage_high_raw[5] = {0,2939,2946,2952,2946}; //raw ADC value for high reference calibration point. Ignore first 0, subsequent is CH1-CH4 from left to right.
const int ADC_voltage_high_mV[5] = {0,7985,7985,7985,7985}; //corresponding mV for high reference calibration point. Ignore first 0, subsequent is CH1-CH4 from left to right.


//Calibration data array for ADC, 4-20mA mode

const int ADC_current_low_raw[5] = {0,1863,1863,1863,1863}; //raw ADC value for low reference calibration point. Ignore first 0, subsequent is CH1-CH4 from left to right.
const int ADC_current_low_uA[5] = {0,10000,10000,10000,10000}; //corresponding uA for low reference calibration point. Ignore first 0, subsequent is CH1-CH4 from left to right.

const int ADC_current_high_raw[5] = {0,3692,3692,3692,3692}; //raw ADC value for high reference calibration point. Ignore first 0, subsequent is CH1-CH4 from left to right.
const int ADC_current_high_uA[5] = {0,20000,20000,20000,20000}; //corresponding uA for high reference calibration point. Ignore first 0, subsequent is CH1-CH4 from left to right.


//Calibration data array for DAC, 0-10V mode

const int DAC_voltage_low_raw[3] = {0,1000,1000};//raw DAC value for low reference calibration point. Ignore first 0, subsequent is CH1-CH2 from left to right.
const int DAC_voltage_low_mV[3] = {0,2641,2630}; //corresponding mV for low reference calibration point. Ignore first 0, subsequent is CH1-CH2 from left to right.

const int DAC_voltage_high_raw[3] = {0,2500,2500}; //raw DAC value for high reference calibration point. Ignore first 0, subsequent is CH1-CH2 from left to right.
const int DAC_voltage_high_mV[3] = {0,6823,6805}; //corresponding mV for high reference calibration point. Ignore first 0, subsequent is CH1-CH2 from left to right.


//Calibration data array for DAC, 4-20mA mode

const int DAC_current_low_raw[3] = {0,1000,1000}; //raw DAC value for low reference calibration point. Ignore first 0, subsequent is CH1-CH2 from left to right.
const int DAC_current_low_uA[3] = {0,5162,5162}; //corresponding uA for low reference calibration point. Ignore first 0, subsequent is CH1-CH2 from left to right.

const int DAC_current_high_raw[3] = {0,3600,3600}; //raw DAC value for high reference calibration point. Ignore first 0, subsequent is CH1-CH2 from left to right.
const int DAC_current_high_uA[3] = {0,19530,19530}; //corresponding uA for high reference calibration point. Ignore first 0, subsequent is CH1-CH2 from left to right.
```


# RTC
Industruinos with the D21G topboard have a built-in RTC (MCP79402 with I2C 0x57 and 0x6F) that can be used with this [library](https://github.com/Industruino/MCP7940-RTC-Library). A simple example that displays the date and time on the LCD can be found [here](https://github.com/Industruino/democode/blob/master/rtc_D21G/rtc_D21G.ino). The RTC also contains a unique 8-byte number (EUI-64) that can be used as a MAC address, see above in the Ethernet section.


# EEPROM
Industruinos with the D21G topboard (from version 1.7) have 1kB EEPROM (non-volatile memory, Microchip AT24CS08) available over I2C (0x50, 0x51, 0x52, 0x53) that can be used with this [library](https://github.com/RobTillaart/Arduino/tree/master/libraries/I2C_EEPROM). It can also be used directly with the Wire library to read/write bytes.


Each of the 4 I2C addresses have 255 bytes available; in total 1kByte.


Check out this [example sketch](https://github.com/Industruino/democode/blob/master/i2c_eeprom_D21G/i2c_eeprom_D21G.ino) that uses the above library to write/read `byte`, `long`, and `float` types (including the conversions).


The 32u4 and 1286 topboards can use the standard Arduino EEPROM library.


# Watchdog
The Adafruit SleepyDog library works on the D21G as in this [example](https://github.com/Industruino/democode/blob/master/watchdog_D21G/watchdog_D21G.ino). This library also has a SLEEP function, which gives good results on the Industruino as documented [here](https://industruino.com/blog/our-news-1/post/d21g-features-30).


For AVR watchdogs on the 1286 and 32u4, see [here](https://industruino.com/page/wdt).


# GSM/GPRS
The Industruino GSM/GPRS module is based on the SIM800 and can be used with a variety of libraries. We recommend the [TinyGSM](https://github.com/vshymanskyy/TinyGSM) library, and you can find several code examples modified for Industruino [here](https://github.com/Industruino/democode). See the module's user manual for configuration options of the serial selector switches. If you are using an IND.I/O unit, please be aware that by default, the 'Serial' port (D0/D1) is connected to the IND.I/O's RS485 port. The current IND.I/O baseboards have a hardware switch to connect/disconnect the RS485 port to this Serial port (D0/D1). To use the 'Serial' port for RS232 or GSM, this switch should be in the upward position (away from the RS485 terminals).


If you want to use the Adafruit FONA library, be aware that the Industruino GSM/GPRS module uses D6 as power on/off pin (it needs 1s HIGH), which is not the same as the Reset pin of the FONA library (pulled LOW for 0.1s in the library).


# Modbus
Modbus is a serial communications protocol popular in industry. It uses a Master/Slave(s) configuration, and comes in 2 types:
* Modbus RTU: using an RS485 port, available on the Industruino IND.I/O
  * suggested library: [SimpleModbus](https://code.google.com/archive/p/simple-modbus/)
  * example: [master: wind speed sensor](https://industruino.com/blog/our-news-1/post/modbus-rtu-on-industruino-ind-i-o-11)
  * example: [master+slave: I/O expansion](https://industruino.com/blog/our-news-1/post/modbus-rtu-master-and-slave-14)
* Modbus TCP: using Ethernet, with the Industruino Ethernet module
  * suggested library: [MgsModbus](http://myarduinoprojects.com/modbus.html)
  * example: [slave: switching application](https://industruino.com/blog/our-news-1/post/ind-i-o-switching-application-with-modbus-tcp-12)

Modbus uses 16-bit registers, we so often need to convert these from/to 32-bit `float` and `long` types; you can use the functions described [here](https://industruino.com/blog/our-news-1/post/modbus-tips-for-industruino-26).


# PROTO
*Notes on digital and analog input/output for the Industruino D21G PROTO kit*

The Industruino PROTO D21G is designed to work with 5V GPIO, while the MCU is running on 3.3V. The level shifters are bidirectional and support analog signals. Note that the output is at 3.3V by default. 

The pins are connected directly to the MCU so we can use the standard Arduino functions.


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
  analogReference(AR_DEFAULT);     // 3V3
  analogReference(AR_INTERNAL);    // 2V23
  analogReference(AR_EXTERNAL);    // put on AREF, max 3V3
  analogReference(AR_INTERNAL1V0);  
  analogReference(AR_INTERNAL1V65);
  analogReference(AR_INTERNAL2V23);
```

