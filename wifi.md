# Industruino WIFI product documentation

*Current version: HW REV: 1.0 (tbc)*

The Industruino WiFi module has several functions:  
 1. [WiFi](#wifi) interface  
 2. [FRAM](#fram) non-volatile memory  
 3. [SD card](#sd-card) interface  
 4. WiFi module [firmware update](#wifi-module-firmware-update)  
 5. [SSL certificates](#ssl-certificates) 

For datasheets, user manuals, pinout maps, see [industruino.com](https://industruino.com/page/techcentre)  
For a plug&play test sketch, see this [WIFIwebclient_demo](https://github.com/Industruino/democode/blob/master/WIFIwebclient_demo/WIFIwebclient_demo.ino) sketch. You only need to insert your WiFi network credentials.

![WIFI MODULE PHOTO](https://industruino.com/website/image/product.template/10_1e844ce/imag)


**Important note:**  
**The Industruino IND.I/O has to be powered by 8-28V to make the WiFi module work; it does not work with USB power only.**


# WiFi

### WiFiNINA library

We will be using the Adafruit fork of Arduino’s WiFiNINA library because that allows us to select the communication pins for our WIFI module (see [below](#library-minimal-example)).

   1. Download the .zip of the Adafruit version of the WiFiNINA library from github [here](https://github.com/adafruit/WiFiNINA) 
   2. Remove the standard WiFiNINA library that comes with the Arduino IDE (the default library folder is at sketchbook/libraries - remove the WiFiNINA folder)
   3. Install the new WiFiNINA library by extracting the .zip file in the sketchbook/libraries folder, OR by using the Arduino IDE menu: Sketch >  Include Library > Add .ZIP Library…
   4. It is necessary to change one setting in the library: the SPI speed needs to be reduced to 4MHz (down from [default 8MHz](https://github.com/adafruit/WiFiNINA/blob/9f4644ebf75b2ae4501a98e42b4714d73e3107a2/src/utility/spi_drv.cpp#LL138C1-L138C83)). Open file sketchbook/libraries/WiFiNINA-master/src/utility/spi_drv.cpp and edit this line:
      
`WIFININA_SPIWIFI->beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));`

   5. Open the [demo sketch](https://github.com/Industruino/democode/blob/master/WIFIwebclient_demo/WIFIwebclient_demo.ino), insert your WiFi credentials, compile and upload to test your system. Open the Serial Monitor for information, or watch the LCD screen. If any errors occur during  compilation, check that you have only one version of the WiFiNINA library installed in your Arduino IDE. You will also need the standard Industruino UC1710 library for the LCD display. 

### Library minimal example
The Adafruit version of the WiFiNINA library allows us to define the pins used by the Industruino MCU to communicate with the ESP32. These pins need to be defined in each sketch:

```
// Industruino WIFI module
#include <SPI.h>
#include <WiFiNINA.h>
#define SPIWIFI_SS      	10
#define SPIWIFI_ACK      	7
#define ESP32_RESETN 		5
#define ESP32_GPIO0  		-1  // not connected
#define SPIWIFI          	SPI
```
Then the WiFi module is initialised with:
```
WiFi.setPins(SPIWIFI_SS, SPIWIFI_ACK, ESP32_RESETN, ESP32_GPIO0, &SPIWIFI);   
```
A client can be initialised as:
```
WiFiClient client;        // for default client, can use connect() or connectSSL()
```
or:
```
WiFiSSLClient client;   // for client that always uses SSL
```
### Module RED LED
The module has one red LED that can be controlled by the sketch, using the wifi_drv utility, and the LED is connected to the ESP32 pin 26.
```
#include <utility/wifi_drv.h>   // for module RED LED
#define ESP32_RGB_RED 26

WiFiDrv::pinMode(ESP32_RGB_RED, OUTPUT);
WiFiDrv::digitalWrite(ESP32_RGB_RED, HIGH);  // on
```

### Other useful library functions:
```
WiFi.status();
WiFi.macAddress(mac)
WiFi.firmwareVersion()
WiFi.begin(ssid, pass)
WiFi.localIP()
WiFi.RSSI()
WiFi.getTime()
```
Full documentation at [Arduino](https://www.arduino.cc/reference/en/libraries/wifinina/).

### MAC address

The WiFi module is using an ESP32 to connect to the network, and it has a hard-coded MAC address, shown in the demo sketch.

# FRAM

The WiFi module also includes FRAM non-volatile memory: see this simle [FRAM demo sketch](https://github.com/Industruino/democode/tree/master/fram_D21G) or the complete [WiFi demo sketch](https://github.com/Industruino/democode/blob/master/WIFIwebclient_demo/WIFIwebclient_demo.ino). 

# SD card

The standard SD library included in the Arduino IDE works with the WiFi module, see [demo sketch](https://github.com/Industruino/democode/blob/master/WIFIwebclient_demo/WIFIwebclient_demo.ino). If you want to run the SD library examples, they need minor modifications (Chip Select pin is D4 as standard): replace *Serial* by *SerialUSB*. And you may have to add these lines to ensure the CS of WiFi, FRAM, and SD are `HIGH`:
```
pinMode(10, OUTPUT); // WiFi CS
pinMode(6, OUTPUT);  // FRAM CS
pinMode(4, OUTPUT);  // SD card CS
digitalWrite(10, HIGH);
digitalWrite(6, HIGH);
digitalWrite(4, HIGH);
```

# WiFi module firmware update

The WIFI module comes with NINA firmware version 1.7.x available from Adafruit [here](https://github.com/adafruit/nina-fw/releases). The modules were designed and tested with version 1.7.4 but later versions should also work.

If for some reason the firmware needs to be updated, follow this procedure:

### Download the new firmware

Download the .bin file to your laptop, e.g. `NINA_W102-1.7.4.bin`

### Find or install `esptool`

We will use Espressif's `esptool` to flash the `.bin` file over USB to the WIFI module.

  * If you have the ESP32 core installed in your Arduino IDE, you can find the esptool at `.arduino15/packages/esp32/tools/esptool_py/2.6.1` or similar  
  * If you do not have the esptool yet, you will need to have Python installed on your laptop and then you can use pip to install [esptool](https://docs.espressif.com/projects/esptool/en/latest/esp32/installation.html)
    * On Windows, open a Command Prompt window
```
> Python –version
```
This will show you the version if you have Python installed; any version is fine. If you do not have Python, install a stable release from [here](https://python.org/downloads)
```
C:\Users\Laptop\AppData\Local\Programs\Python\Python311
> pip install esptool
```
If you do not have `pip`, install like [this](https://phoenixnap.com/kb/install-pip-windows). Pip is located in `C:\Users\Laptop\AppData\Local\Programs\Python\Python311\Scripts`  
This will install the necessary packages for `esptool`, in the same `Scripts` folder

### Use `esptool` to flash the new firmware

After downloading the `.bin` file, and locating or installing `esptool` you are ready to flash the new firmware to the WIFI module: 

   1. Keep the Industruino device powered off
   2. Keep the WIFI module disconnected from the Industruino device
   3. Open the casing of the WIFI module to reach the PCB
   4. Connect the WIFI module to your laptop using the microUSB port as in below picture, the green LED will switch on
   5. Check the name of the USB port on your laptop e.g. `/dev/ttyUSB0` (on Linux and macOS) or `COM1` 

Windows: use the Device Manager and look for Ports (COM&LPT), there should be an entry called ‘Silicon Labs CP210x’, note the port name, e.g. `COM5`) - if necessary install the [driver](https://www.pololu.com/docs/0J7/all#2) for ‘CP2102 USB to UART’  

Linux: use the `sudo dmesg` command to see the name of the port 

   6. At the command line, navigate to your Downloads folder where you have the `.bin`
   7. Flash the relevant NINA firmware `.bin` file:

Windows: find the python folder, then Scripts  
```
> esptool --port COM5 --baud 115200 write_flash 0 C:\Users\Laptop\Desktop\NINA_W102-1.7.4.bin
```  
Linux:   
```
$ esptool.py --port /dev/ttyUSB0 --baud 115200 write_flash 0 NINA_W102-1.7.4.bin
```
This will take about 1 minute.

Now your module has the updated firmware, and you can check it with `WiFi.firmwareVersion()` in your sketch.

# SSL certificates

One reason to update the firmware is to include specific SSL certificates. The standard firmware includes a long list of certificates for common websites, but not all. The Arduino IDE has a tool to upload additional certificates to a WiFiNINA module, but unfortunately this does not work with the Industruino WIFI module. The only option is to compile a **customised version of the NINA firmware**, starting from the Adafruit version. This ‘building’ involves installing the ESP32 toolchain and the ESP-IDF as explained [here](https://github.com/adafruit/nina-fw#building).

The file we want to modify is [roots.pem](https://github.com/adafruit/nina-fw/blob/master/data/roots.pem). This file contains all the SSL certificates, and we can add our certificate to it (.pem format).
```
$ make -B firmware
```
This creates a new .bin file including the added SSL certificates; upload to the module with `esptool` as described above.

You are welcome to contact Industruino at connect@industruino.com to request to include your SSL certificate in a customised firmware .bin.