# Industruino IND.I/O product documentation

*Current version: HW REV: 5.2, FW REV: 3.0*

1. Setting up the Arduino [IDE](#setting-up-the-arduino-ide)
2. [D21G](#d21g-board-specific-features) specific features
3. [IDC](#idc-expansion-port-pinout) expansion port pinout
4. Libraries for use with IND.I/O:
    * [UC1701](#uc1701-library) - LCD display default option
    * [U8G and U8G2](#u8g-and-u8g2-libraries) - LCD display advanced option
    * [Indio](#indio-library) - industrial I/O channels
      * [digital I/O](#digital-io) 
      * [interrupts](#interrupts)      
      * [analog input](#analog-input)
      * [analog output](#analog-output)
      * [analog calibration](#analog-calibration)
5. [RS485 port](#rs485)
6. [RTC](#rtc)
7. [EEPROM](#eeprom)
8. [WDT](#watchdog)
9. [Modbus](#modbus) - RTU and TCP
10. [TFTP](#tftp) - upload sketch over Ethernet
11. [Security bit](#security-bit) - protect your code


For datasheets, user manuals, pinout maps, see [industruino.com](https://industruino.com/page/techcentre)

![IND.I/O](https://industruino.com/website/image/product.template/2_cd53b8d/image)

# Setting up the Arduino IDE

D21G is compatible with IDE from 1.6.12 with automatic install via board manager: 
* In *File > Preferences > Additional Boards Manager URLs:* add https://static.industruino.com/downloads/code/IndustruinoCores/IndustruinoSAMD/pkgdef/package_industruino_samd_index.json 
* Enter the Board Manager via *Tools > Board* and search for 'industruino'
* Install the Industruino package
* *Industruino D21G* will show up in the Boards list in *Tools > Board* in section *Industruino SAMD*
* Windows driver (if needed): https://static.industruino.com/downloads/drivers/drivers-industruino-windows-0.0.1.zip


# D21G board specific features

* `RESET` button on D21G: when your computer does not recognise the USB port anymore, reset the D21G by pushing the reset button on the bottom of the topboard TWICE (you will have to remove the casing for this). The LCD backlight will start fading in&out, indicating the bootloader is active and ready for a new upload.
* Serial ports on D21G: 
  * `SerialUSB` for USB (Serial Monitor)
  * `Serial` for hardware serial on D0/D1
  * `Serial1` for hardware serial on D10/D5
* The LCD is connected over SPI to pins D19,D20,D21,D22
* LCD backlight is on D26
* The membrane buttons are on D23,D24,D25
* Hardware Timers of the Industruino D21G are similar to the Arduino Zero, see this [blog post](https://industruino.com/blog/our-news-1/post/d21g-timer-library-33).
* The pre-loaded demo code is available at https://github.com/Industruino/democode


*Legacy boards 32u4 and 1286 specific features:*
* 32u4 is compatible with all IDE versions (board type: Leonardo)
* 1286 is compatible with IDE up to 1.6.5 (manual install of board definitions: follow instructions in the support file package)
* Serial ports on 32u4 and 1286: `Serial` for USB and `Serial1` for hardware serial on D0/D1
* LCD backlight is on D13 for 32u4 and D26 for 1286


# IDC expansion port pinout

Unlike the IND.I/O's digital and analog I/O channels, the IDC expansion port provides direct access to the MCU's GPIO pins. This port is intended for use with Industruino Ethernet module or Industruino GSM/GPRS module, but in other cases, the IDC pins can also be used to connect other devices to the IND.I/O. These pins are accessible with standard `pinMode`, `digitalRead` and `digitalWrite` commands (different from the IND.I/O digital I/O channels which are controlled by the Indio library). Please note:
* the D21G works at 3.3V: if you need 5V output, use a 10K pull-up resistor. 
* the Industruino's DC/DC converter (V+/V- to 5V) is 2W, so can supply 400mA max on the 5V line. The topboard needs around 50mA, depending on the LCD backlight, so 350mA is available on the IDC 5V.


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


# UC1701 library

You can download the library from within the Arduino libraries manager or from [this repository](https://github.com/Industruino/UC1701). This library is easy to use, relatively small (it is also used in the Industruino pre-installed demo sketches). It is largely compatible with the popular PCD8544 (Nokia screens), with familiar syntax:  
```
lcd.begin();  
lcd.clear();  
lcd.setCursor(1, 1);  
lcd.print("hello Industruino!");
```


# U8G and U8G2 libraries

[U8G](https://github.com/olikraus/u8glib) is a popular display library with many fonts and graphics, consuming more memory than the basic UC1701 above. Use this constructor:
```
U8GLIB_MINI12864 u8g(21, 20, 19, 22);	// SPI Com: SCK = 21, MOSI = 20, CS = 19, A0 = 22
```

[U8G2](https://github.com/olikraus/u8g2) is the new improved version of the above U8G library, largely compatible, with 3 [buffer](https://github.com/olikraus/u8g2/wiki/u8g2setupcpp#buffer-size) options (speed vs memory); using the second hardware SPI also increases speed. Try the GraphicsTest examples included in the library, with these respective constructors.
For the fastest performance, use this constructor for hardware SPI with full_buffer:
```
U8G2_UC1701_MINI12864_F_2ND_4W_HW_SPI u8g2(U8G2_R2, /* cs=*/ 19, /* dc=*/ 22);
```
And hardware SPI with page_buffer:
```
U8G2_UC1701_MINI12864_1_2ND_4W_HW_SPI u8g2(U8G2_R2, /* cs=*/ 19, /* dc=*/ 22);
```
Use this constructor for software SPI and page_buffer:
```
U8G2_UC1701_MINI12864_1_4W_SW_SPI u8g2(U8G2_R2, 21, 20, 19, 22);   // rotation, clock, data, cs, dc
```
And software SPI with full_buffer:
```
U8G2_UC1701_MINI12864_F_4W_SW_SPI u8g2(U8G2_R2, 21, 20, 19, 22);   // rotation, clock, data, cs, dc
```


# Indio library

Important notes:    
1. The digital and analog I/O will only work when Vin power (6.5-32V) is supplied to the Indio baseboard via the green screw connectors. When only USB power is connected, none of the digital or analog channels, nor the RS485, will work.  
2. It is important to power down all systems (Industruino, sensors/actuators) before making connections to the Industruino.

You can download the library from within the Arduino libraries manager or from [this repository](https://github.com/Industruino/Indio).

You will need this library to access the I/O channels. The pins on the IDC expansion connector, the backlight pin, and the membrane panel buttons should still be accessed in the usual way, not using the Indio library; the Indio library is only for the external I/O channels available on the green screw connectors.

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
* example of how to connect [NPN and PNP](https://industruino.com/blog/our-news-1/post/industrial-proximity-sensors-for-rpm-40) type sensors


### INTERRUPTS

Pin changes on any of the 8 digital input channels can trigger a general interrupt to the MCU. The interrupt pin of the expander on the 12/24V digital side is connected to D8 (=INT8) pin of the D21G. This pin will trigger when a change on any of the 8 input or output channels occurs. By default all input channels have this interrupt enabled. However, this can be configured per channel:
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


### ANALOG INPUT

Important notes:    
1. The analog I/O section is galvanically isolated from the digital I/O section and the microcontroller section, to allow a separate power supply in the analog section for optimal accuracy. In case your analog sensors/actuators are on the same power supply as the digital section (Vin 12/24V) you have to connect the analog GND to the digital GND.
2. The accuracy of the input and output can be improved by calibration (see below).

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

Important notes:   
1. The analog I/O section is galvanically isolated from the digital I/O section and the microcontroller section, to allow a separate power supply in the analog section for optimal accuracy. In case your analog sensors/actuators are on the same power supply as the digital section (Vin 12/24V) you have to connect the analog GND to the digital GND.
2. The accuracy of the input and output can be improved by calibration (see below).

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


### ANALOG CALIBRATION

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


# RS485

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

# TFTP

The Industruino D21G topboard offers the possiblity to upload a sketch over Ethernet instead of the usual USB. This is accomplished with a special Ethernet/USB hybrid bootloader. This is an overview of the procedure, with more details [below]:
1. Hardware switch on the topboard: make sure the switch 'Eth Boot' is set to the ON position
2. Network settings in FRAM: store your network configuration in non-volatile memory
3. Upload a sketch over USB that includes the ```ResetServer``` elements
4. Prepare the new sketch to be uploaded over Ethernet with TFTP:
   * Include the ```ResetServer``` elements
   * Export the sketch as binary file .bin
   * Process the binary with the ```prepareFile``` utility
5. Upload the sketch over Ethernet
   * Use TFTP to start the upload
   * Reset the Industruino via your browser to allow the upload to be received
   

Here are more details about each step. Original blog post [here](https://industruino.com/blog/our-news-1/post/remote-sketch-upload-via-ethernet-27).


### Hardware switch 'Eth boot'

You will have to remove the casing of your Industruino to reach the bottom of the topboard. Next to the battery holder you will find the 'Eth boot' switch; slide to the ON position (towards the battery holder). You can still upload sketches over USB, but now the bootloader will also get the network settings from FRAM memory to listen for an Ethernet upload request.


### Network setting in FRAM

We recommend to use the unique MAC address from the RTC EEPROM. Use this [sketch](https://github.com/Industruino/democode/blob/master/MACfromRTC/MACfromRTC.ino) to retrieve this 6-byte MAC address and note it down.

FRAM is non-volatile memory, included in the Industruino Ethernet module. Install the FRAM library from [here](https://github.com/Industruino/FRAM) and open the NetConfigDataMgr sketch. This sketch can be used to READ, WRITE, ERASE the network settings. 
* Connect your Industruino with the Ethernet module and power up with external PSU
* Uncomment the WRITE line (24), and comment out the READ line (23)
* Edit the 4 network settings MAC (as above), IP, MASK, GATEWAY
* You can pick a password (default: 'password') but make sure to use 8 characters; this password is the key that will accept incoming sketch upload requests over TFTP
* Upload the sketch and check output on the Serial Monitor to make sure your settings are correct
* Verify the settings are correctly stored by activating the READ line, uploading the sketch, and check Serial Monitor


### ResetServer

In your Arduino IDE, open *File > Examples* and under 'Examples for Industruino D21G' find the ```EthernetReset``` folder with the ```ResetServer``` sketch. Alternatively, use the [demo sketch](https://github.com/Industruino/democode/blob/master/ResetServerTFTP_D21G/ResetServerTFTP_D21G.ino) with feedback on the LCD.

This sketch shows the elements that are necessary to reset your Industruino over Ethernet. You have to include these elements in the original sketch and in the updated sketch. Modify your network settings same as before.

Upload the sketch over USB. You can now test it by going to this URL in your browser: 
```http://192.168.1.199:8080/password/reset```   
This should reply 'Rebooting...' and reset your Industruino.    
Note the password is the same as you set in the sketch.


### TFTP upload

This involves a few steps to prepare the binary file to be sent over TFTP.

* Make sure to include the ResetServer elements in the new sketch you want to upload if you want to upload over Ethernet again.
* Use the 'Export compiled Binary' tool in the Arduino IDE to get the .bin in your sketch folder.
* Download the [prepareFile](https://static.industruino.com/downloads/code/prepareFile.zip) utility and use the command line to navigate to your platform's folder (win, mac, linux32/64)
* Find the executable 'prepareFile' and use this syntax to obtain the binary outputFile:
```./prepareFile password inputFile outputFile```
* Start the TFTP session:
```
tftp 192.168.1.199
> mode binary
> trace
> verbose
> put outputFile
```
* TFTP will attempt to send the file but it will not be able to establish the connection until the Industruino is reset, so use your browser to reset, before the TFTP reaches timeout.
* The process can be automated with a simple Linux [shell script](https://github.com/Industruino/democode/blob/master/ResetServerTFTP_D21G/industruino_tftp.sh): this example resets the board by URL first and then immediately starts the TFTP put. Timing is critical, as the reset only activates the bootloader for a couple of seconds. If that window is too short for your application, you can make use of the longer TFTP timeout (around 25 seconds on standard Linux), and first initiate the TFTP put, and then reset by URL.


### Troubleshooting

* Make sure your FRAM network settings are identical to settings in the sketch that is running (incl MAC address)
* Make sure your computer and network allow TFTP traffic. TFTP uses UDP on port 69 and then random ports so it is possible your firewall blocks the upload, resulting in a TFTP timeout. If you suspect a network issue, you can test with the simplest possible network: an Ethernet cable from your laptop to your Industruino, with the laptop disconnect from wifi/LAN. Your laptop should be configured with a fixed IP address, e.g. 192.168.1.1, also used as gateway, and mask 255.255.255.0. Just make sure to use a compatible fixed IP address on the Industruino, e.g. 192.168.1.199.


# Security bit

In order to protect the intellectual property of your code, it is possible to set a security bit in the SAMD21G microcontroller, which effectively blocks any attempt to download the compiled code from the microcontroller's internal FLASH memory by a third party. 

* A demo sketch which sets this bit can be found [here](https://github.com/Industruino/democode/tree/master/SAMD21_SecurityBit). You can integrate the functions seen in this demo  sketch into your own application sketch. 

* Running the ```if (!setSecurityBit()){};``` command in your Setup() routine will enable the security bit. 

* The sketch also enables the brown-out protection at 3.3V (to protect Voltage Fault Injection Attacks).

WARNING: After enabling the security bit you will still be able to upload new code (the old protected code gets erased), but you can not read-back the FLASH contents of the MCU. To disable the security you will need to use an in-circuit debugger such as the Atmel ICE. 
