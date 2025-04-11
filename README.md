# Digital Timetable Display System

## Overview
This project implements a digital timetable display system using an ESP32 microcontroller, an SSD1306 OLED display, a DS3231 RTC module, and a buzzer. The system shows the current or upcoming class based on the time and day, with notifications before classes begin.


## Features
- Real-time display of current class information
- Notification system with buzzer alarm 5 minutes before class starts
- Automatic time synchronization via NTP (Network Time Protocol)
- Persistent storage of timetable data using SPIFFS filesystem
- Day and time awareness with RTC (Real-Time Clock) module
- HTTP Web server interface for updating timetable remotely
- Low power consumption for extended battery life
- Attractive AMU logo display during non-class hours

## Hardware Requirements
- ESP32 development board
- SSD1306 128x64 OLED display
- DS3231 RTC module
- Buzzer
- Connecting wires
- Power supply

## Pin Configuration
(These are hard coded into the code due to the issue with using two different i2c drivers.So, if you want to change them look into the code.)
| Component | ESP32 GPIO Pin |
|-----------|----------------|
| OLED SDA  | GPIO 21        |
| OLED SCL  | GPIO 22        |
| OLED RST  | Configured in menuconfig |
| RTC SDA   | GPIO 5         |
| RTC SCL   | GPIO 2         |
| Buzzer    | GPIO 23        |

## Timetable Format
The timetable is stored as a CSV file in the SPIFFS filesystem at `/storage/timetable.csv`. The format is:

```
DAY, 8:00, 8:50, 9:40, 10:30, 11:20, 12:10, 13:00, 14:00
MON, Math, Physics, Chemistry, Break, English, History, Geography, x
TUE, Physics, Chemistry, Math, Break, Geography, English, History, x
...
```

Where:
- First row contains time slots
- First column contains days of the week (MON, TUE, WED, THU, FRI, SAT, SUN)
- "x" represents no class
- Empty cells or "x" are treated as free periods

## Setup and Installation

### Prerequisites
- ESP-IDF (version 4.4 or later)

### Building and Flashing
1. Clone this repository:
   ```
    https://github.com/Shakirzzzzz/Automated-Classroom-Alert-System-with-Smart-Reminders-and-Display-Capabilities.git
   ```

2. Configure the project:
   ```
   idf.py menuconfig
   ```
   - Set Wi-Fi credentials under "Example Connection Configuration"(and also in the code)
       #### To setup up the http server i originally hard coded the ssid and password into to wifi.c file. Then when i chose to use ntp server to update the RTC,the example code came with the example connect so i didnt bother to change that. So update the ssid and password in the wifi.c file and also in the menuconfig under example connect    
   - Configure display options in the code because the config files of the RTC and SSD1306 interfere with each other.
       #### The variables for these are on top in the TimeTableCode.c file.
   - Set only the timezone in the config file of DSD3231.

3. Build the project:
   ```
   idf.py build
   ```

4. Flash the firmware:
   ```
   idf.py -p PORT flash
   ```
   Replace `PORT` with your ESP32's serial port (e.g., `/dev/ttyUSB0` or `COM3`).

5. Monitor the output:
   ```
   idf.py -p PORT monitor
   ```

6. Upload the timetable file:
   - Connect to the ESP32's Wi-Fi access point (default SSID: "Test")
   - Open a web browser and navigate to http://172.20.10.3/
   - Upload your timetable CSV file(By default there is my personal timetable uploaded)

## Usage
Once powered on, the system:
1. Connects to Wi-Fi and synchronizes time with an NTP server (on first boot)
2. Retrieves the current time from the RTC module
3. Displays the current class based on the day and time
4. Shows the AMU logo during non-class hours(You can create your own bitmap logo and replace it in the code)
5. Provides a buzzer notification 5 minutes before each class and when the class is about to start.



## Acknowledgements
- ESP-IDF framework by Espressif Systems
- SSD1306 library for ESP-IDF
- DS3231 RTC library
- https://github.com/nopnop2002/esp-idf-ds3231.git
- https://github.com/nopnop2002/esp-idf-ssd1306.git
- https://github.com/espressif/esp-idf/tree/master/examples
