# ESP32 C++ Code for Controlling Robot Arm

We need the following items.

- Arduino IDE
- Node32MCU ESP32 Board



# Environment Settings

### Windows Driver for ESP32

1. Download the [CP210x Universal Windows Driver](https://www.silabs.com/documents/public/software/CP210x_Universal_Windows_Driver.zip).
2. Unzip the compressed file.
3. Right-click the `silabser.inf` and select "install".



### Arduino IDE

1. Install [Arduino IDE](https://support.arduino.cc/hc/en-us/articles/360019833020-Download-and-install-Arduino-IDE).

2. Open File > Preferences > Settings > Additional boards manager URLs, and then enter the following URL:

   > https://dl.espressif.com/dl/package_esp32_index.json

   Finally, click OK.

3. Open Tools > Board > Boards Manager, and then install `esp32` made by Espressif Systems.

4. Open Tools > Manage Libraries, and then install the following libraries.

   - `PID` made by Brett Beauregard
   - `ArduinoJson` made by Benoit Blanchon
   - `ESP32Servo` `0.13.0` made by Kevin Harrington, John K. Bennett
   - `ESP32Encoder` `0.9.1` made by Kevin Harrington
   - `EspSoftwareSerial` `8.0.1` made by Dirk Kaar, Peter Lerup
   - `Adafruit-PWM-Servo-Driver-Library` made by Adafruit



## Code Definition and Features

### The Maximum and Minimum Servo Angles

We've defined the max. and min. angles for servos at the following code.

```c++
const uint8_t servoMinAngles[] = {0, 0, 0, 0, 0};
const uint8_t servoMaxAngles[] = {180, 120, 160, 180, 70};
```



### Initial Pose

The initial pose is defined via the following code.

```c++
void setup() {
    // ...
    // Create the arm control task
    for (uint8_t i = 0; i < NUM_OF_SERVOS; i++) {
        currentAngles[i] = (float) servoMinAngles[i];
        switch (i) {
            case 0:
                targetAngles[0] = 90;
                break;
            case 1:
                targetAngles[1] = 0;
                break;
            case 2:
                targetAngles[2] = 160;
                break;
            case 3:
                targetAngles[3] = 50;
                break;
            case 4:
                targetAngles[4] = 10;
                break;
            default:
                break;
        }
    }
    // ...
}
```

