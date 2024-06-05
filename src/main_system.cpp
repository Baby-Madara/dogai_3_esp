#include <system.h>


void setup()    {    Serial.begin(115200);    taskInitializers();    }
void loop()     {}



/*

[env:nodemcu-32s]
platform = espressif32
board = nodemcu-32s
framework = arduino
monitor_speed = 115200
upload_protocol = espota
upload_port = 192.168.1.9

; lib_deps = 
; 	adafruit/Adafruit MPU6050@^2.2.4
; 	arduino-libraries/Servo@^1.2.1
; 	bblanchon/ArduinoJson@^7.0.4
; 	adafruit/Adafruit PWM Servo Driver Library@^3.0.2
; 	links2004/WebSockets@^2.4.1
; 	hideakitai/ArduinoEigen@^0.3.2

*/