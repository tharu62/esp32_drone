| Supported Targets | ESP32-S3-WROOM-1 |

# DRONE configuration and description (WORK IN PROGRESS)

## Overview

This project desribes the configuration and setup of a quadcopter drone.
The drone is runnig with an ESP32 microcontroller, a MPU6050 sensor and 4 brushless motors controlled by compatible ESCs. It is controlled by remote with a custom made joystick that
uses ESP-NOW communication. 

## How to use example

/...

### Hardware Required

To run this project, you should at least have an Espressif development board based on a chip
 listed in supported targets as well as a MPU6050. The latter is a inertial measurement 
 unit, which contains an accelerometer, a gyroscope as well as a temperature sensor. 
 For more information you can read the added documentation. You can use whatever motor you can 
 find but it has to allow pwm controll on 3V3, as the esp32-s3 chip run on 3V3 logic level.
 Detailed list of components:
 - ESP32-S3-WROOM-1 chip
 - MPU6050
 - 4 BRUSHLESS MOTORS 1503
 - 4 MINI ESC COMPATIBLE WITH THE MOTORS
 - PCB + ELETTRICAL COMPONENT (search BOP, CPL and GERBER in the doc folder)

#### Pin Assignment

**Note:** The following pin assignments are used by default, you can change these in the code.


|                  | SDA            | SCL            |
| ---------------- | -------------- | -------------- |
| ESP32 I2C Master | GPIOXX         | GPIOXX         |
| MPU6050 Sensor   | SDA            | SCL            |


|  ESC SIGNAL PIN | MOTOR A  | MOTOR B | MOTOR C | MOTOR D | 
| --------------- | -------- | ------- | ------- | ------- |
| ESP32 MOTOR PWM | GPIOXX   | GPIOXX  | GPIOXX  | GPIOXX  |



**Note:** There's no need to add an external pull-up resistors for any pin, because the driver will enable the internal pull-up resistors.

### Build and Flash

Enter `idf.py -p PORT flash monitor` to build, flash and monitor the project.

(To exit the serial monitor, type ``Ctrl-]``.)

See the [Getting Started Guide](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html) for full steps to configure and use ESP-IDF to build projects.

## Example Output

```bash
I (328) example: I2C initialized successfully
I (338) example: WHO_AM_I = 71
I (338) example: I2C de-initialized successfully
```
```bash
  _   _                        __ ___                 _           _   _           
 | | | |                      / /|__ \               | |         | | (_)          
 | |_| |__   __ _ _ __ _   _ / /_   ) |     _ __ ___ | |__   ___ | |_ _  ___ ___  
 | __| '_ \ / _` | '__| | | | '_ \ / /     | '__/ _ \| '_ \ / _ \| __| |/ __/ __| 
 | |_| | | | (_| | |  | |_| | (_) / /_     | | | (_) | |_) | (_) | |_| | (__\__ \ 
  \__|_| |_|\__,_|_|   \__,_|\___/____|    |_|  \___/|_.__/ \___/ \__|_|\___|___/ 
```