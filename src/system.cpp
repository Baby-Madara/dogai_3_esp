#include <system.h>
#include <robotics_math.h>



TwoWire servoDriverWire = TwoWire(1); // Use I2C bus 0
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, servoDriverWire);





/////////////////////////////////////// Webpage //////////////////////////////////////////////

// Network credentials
const char *ssid = "Fr7oo";
const char *password = "25897463";

// Static IP configuration
IPAddress local_IP(192, 168, 1, 184);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(8, 8, 8, 8);

// Web server and WebSocket server
WebServer server(80);
WebSocketsServer webSocketPhone(81);
WebSocketsServer webSocketPython(82);


// HTML page (minified for brevity)
const char htmlPage[] PROGMEM = R"rawliteral(<!DOCTYPE HTML>
    <html>
    <head>
      <title>ESP32 Control</title>
      <style>
        body { 
          font-family: Arial, sans-serif; 
          text-align: center; 
          display: grid;
          grid-template-rows: auto auto auto auto auto auto auto;
          gap: 20px;
          padding: 20px;
          user-select: none;
          -webkit-user-select: none;
        }
        .grid-container {
          display: grid;
          grid-template-columns: 1fr 2fr 1fr;
          align-items: center;
          place-items: center;
          gap: 20px;
        }
        img { 
          justify-self: center;
          align-self: center;
          margin: 0 auto; 
          grid-column: span 3;

        }
        .joystick { 
          margin: 0 auto; 
          display: inline-block; 
          width: 150px; 
          height: 150px; 
          background: #ccc; 
          position: relative; 
          border-radius: 50%; 
          touch-action: none; 
        }
        .joystick .stick { 
          width: 60px; 
          height: 60px; 
          background: #888; 
          border-radius: 50%; 
          position: absolute; 
          left: 50%; 
          top: 50%; 
          transform: translate(-50%, -50%); 
          touch-action: none; 
        }
        .slider input[type="range"] { 
          width: 100%; 
          height: 30px;
          transform: translateX(-50%);
        }
        .log { 
          margin: 0 auto; 
          border: 1px solid #ccc; 
          padding: 10px; 
          text-align: left;
          grid-column: span 3;
        }
        .log table {
          width: 100%;
          border-collapse: collapse;
        }
        .log th, .log td {
          border: 1px solid #ccc;
          padding: 5px;
          text-align: center;
          white-space: nowrap;
        }
        .title, .image, .data, .velocities, .rpy1, .rpy2, .xyz {
          grid-column: span 3;
        }
      </style>
      <script>
        var socket;
        var logData = {
          vx: 0, vy: 0, vz: 0,
          R1: 0, P1: 0, Y1: 0,
          R2: 0, P2: 0, Y2: 0,
          X: 0, Y: 0, Z: 0
        };
    
        function initWebSocket() {
          socket = new WebSocket('ws://' + window.location.hostname + ':81');
          socket.binaryType = 'arraybuffer';
    
          socket.onopen = function() {
            console.log('WebSocket connection established');
          };
    
          socket.onmessage = function(event) {
            drawImage(new Uint8Array(event.data));
          };
    
          socket.onclose = function() {
            console.log('WebSocket connection closed, attempting to reconnect in 1 second...');
            setTimeout(initWebSocket, 1000);
          };
    
          socket.onerror = function(error) {
            console.error('WebSocket error:', error);
          };
    
          setInterval(() => {
            if (socket.readyState === WebSocket.OPEN) {
              socket.send(JSON.stringify(logData));
            }
          }, 100);
        }
    
        function drawImage(byteArray) {
          var canvas = document.getElementById('photoCanvas');
          var ctx = canvas.getContext('2d');
          var imageData = ctx.createImageData(50, 50);
          for (var i = 0; i < byteArray.length; i++) {
            var value = byteArray[i];
            imageData.data[i * 4] = value;        // Red
            imageData.data[i * 4 + 1] = value;    // Green
            imageData.data[i * 4 + 2] = value;    // Blue
            imageData.data[i * 4 + 3] = 255;      // Alpha
          }
          ctx.putImageData(imageData, 0, 0);
        }
    
        function init() {
          initWebSocket();
    
          const joysticks = document.querySelectorAll('.joystick');
          joysticks.forEach((joystick, index) => {
            const stick = joystick.querySelector('.stick');
            const joystickRect = joystick.getBoundingClientRect();
            const maxRadius = joystick.offsetWidth / 2 - stick.offsetWidth / 2;
    
            function handleJoystickMove(event) {
              event.preventDefault();
              const touch = event.touches ? event.touches[0] : event;
              const x = touch.clientX - joystickRect.left - joystick.offsetWidth / 2;
              const y = touch.clientY - joystickRect.top - joystick.offsetHeight / 2;
              const distance = Math.min(Math.sqrt(x * x + y * y), maxRadius);
              const angle = Math.atan2(y, x);
    
              const offsetX = Math.cos(angle) * distance;
              const offsetY = Math.sin(angle) * distance;
    
              stick.style.transform = `translate(calc(${offsetX}px - 50%), calc(${offsetY}px - 50%))`;
    
              updateLogData(index + 1, offsetY * (-20.0 / 9.0), offsetX * (-20.0 / 9.0));
            }
    
            function resetJoystick() {
              stick.style.transform = 'translate(-50%, -50%)';
              updateLogData(index + 1, 0, 0);
            }
    
            joystick.addEventListener('mousedown', (event) => {
              document.addEventListener('mousemove', handleJoystickMove);
              document.addEventListener('mouseup', () => {
                document.removeEventListener('mousemove', handleJoystickMove);
                resetJoystick();
              });
            });
    
            joystick.addEventListener('touchstart', (event) => {
              document.addEventListener('touchmove', handleJoystickMove);
              document.addEventListener('touchend', () => {
                document.removeEventListener('touchmove', handleJoystickMove);
                resetJoystick();
              });
            });
          });
    
          const sliders = document.querySelectorAll('.slider input[type="range"]');
          sliders.forEach((slider, index) => {
            slider.addEventListener('input', () => {
              sendSliderData(index + 1, parseFloat(slider.value));
            });
          });
        }
    
        function updateLogData(joystickIndex, x, y) {
          switch (joystickIndex) {
            case 1:
              logData.vx = x;
              logData.vy = y;
              break;
            case 2:
              logData.R1 = x;
              logData.P1 = y;
              break;
            case 3:
              logData.R2 = x;
              logData.P2 = y;
              break;
            case 4:
              logData.X = x;
              logData.Y = y;
              break;
          }
          updateLogDisplay();
        }
    
        function sendSliderData(id, value) {
          switch (id) {
            case 1:
              logData.vz = value;
              break;
            case 2:
              logData.Y1 = value;
              break;
            case 3:
              logData.Y2 = value;
              break;
            case 4:
              logData.Z = value;
              break;
          }
          updateLogDisplay();
        }
    
        function updateLogDisplay() {
          document.getElementById('vx').innerText = logData.vx.toFixed(2);
          document.getElementById('vy').innerText = logData.vy.toFixed(2);
          document.getElementById('vz').innerText = logData.vz.toFixed(2);
          document.getElementById('R1').innerText = logData.R1.toFixed(2);
          document.getElementById('P1').innerText = logData.P1.toFixed(2);
          document.getElementById('Y1').innerText = logData.Y1.toFixed(2);
          document.getElementById('R2').innerText = logData.R2.toFixed(2);
          document.getElementById('P2').innerText = logData.P2.toFixed(2);
          document.getElementById('Y2').innerText = logData.Y2.toFixed(2);
          document.getElementById('X').innerText = logData.X.toFixed(2);
          document.getElementById('Y').innerText = logData.Y.toFixed(2);
          document.getElementById('Z').innerText = logData.Z.toFixed(2);
        }    
        document.addEventListener('DOMContentLoaded', init);
        </script>
</head>
<body onload="init()">
  <h1 class="title">ESP32 Control</h1>
  <canvas id="photoCanvas" width="300" height="200" class="image"></canvas>
  <div class="log data">
    <table>
      <tr>
        <td>vx: <span id="vx">0</span></td>
        <td>R1: <span id="R1">0</span></td>
        <td>R2: <span id="R2">0</span></td>
        <td>X: <span id="X">0</span></td>
      </tr>
      <tr>
        <td>vy: <span id="vy">0</span></td>
        <td>P1: <span id="P1">0</span></td>
        <td>P2: <span id="P2">0</span></td>
        <td>Y: <span id="Y">0</span></td>
      </tr>
      <tr>
        <td>vz: <span id="vz">0</span></td>
        <td>Y1: <span id="Y1">0</span></td>
        <td>Y2: <span id="Y2">0</span></td>
        <td>Z: <span id="Z">0</span></td>
      </tr>
    </table>
  </div>
  <div class="grid-container velocities">
    <div class="joystick">
      <div class="stick"></div>
    </div>
    <div>Velocities</div>
    <div class="slider" id="slider1">
      <input type="range" min="-100" max="100" value="0">
    </div>
  </div>
  <div class="grid-container rpy1">
    <div class="joystick">
      <div class="stick"></div>
    </div>
    <div>RPY1</div>
    <div class="slider" id="slider2">
      <input type="range" min="-100" max="100" value="0">
    </div>
  </div>
  <div class="grid-container rpy2">
    <div class="joystick">
      <div class="stick"></div>
    </div>
    <div>RPY2</div>
    <div class="slider" id="slider3">
      <input type="range" min="-100" max="100" value="0">
    </div>
  </div>
  <div class="grid-container xyz">
    <div class="joystick">
      <div class="stick"></div>
    </div>
    <div>XYZ</div>
    <div class="slider" id="slider4">
      <input type="range" min="-100" max="100" value="0">
    </div>
  </div>
</body>
</html>
)rawliteral";

void handleRoot()
{
    server.send(200, "text/html", htmlPage);
}

void webSocketEventPhone(uint8_t num, WStype_t type, uint8_t *payload, size_t length)
{
    if (type == WStype_TEXT)
    {
        Serial.printf("[%u] get Text: %s\n", num, payload);
        // Handle incoming text data from WebSocket
    }
}

void webSocketEventPython(uint8_t num, WStype_t type, uint8_t *payload, size_t length)
{
    if (type == WStype_TEXT)
    {
        Serial.printf("Python WebSocket [%u] get Text: %s\n", num, payload);
        // Handle incoming text data from Python WebSocket
    }
    else if (type == WStype_BIN)
    {
        Serial.printf("Python WebSocket [%u] get binary length: %u\n", num, length);
        // Handle incoming binary data from Python WebSocket
    }
}





float xyz_cmd_array[4][3];








/////////////////////////////////////////////      RTOS     /////////////////////////////////////////////

// Task handles
TaskHandle_t Task1_Handle;
TaskHandle_t Task2_Handle;
TaskHandle_t Task3_Handle;
TaskHandle_t Task4_Handle;
TaskHandle_t Task5_Handle;
TaskHandle_t Task6_Handle;

void taskInitializers()
{

    // xTaskCreatePinnedToCore(
    //      Task_function,
    //      "Name of task",
    //      Stack_size (bytes),
    //      Task input parameter  (NULL usually),
    //      Priority of the task  (0 low - 4 high...),
    //      &Task_handle,
    //      core_number
    // );

    // Create Task1 WiFi_comm
    xTaskCreatePinnedToCore(
        Task1_WiFi_comm,
        "WiFi_comm",
        16384,
        NULL,
        1,
        &Task1_Handle,
        1);

    // Create Task2 controller
    xTaskCreatePinnedToCore(
        Task2_controller,
        "controller",
        1024,
        NULL,
        2,
        &Task2_Handle,
        0);

    // Create Task3 gait_IK
    xTaskCreatePinnedToCore(
        Task3_gait_IK,
        "gait_IK",
        81920,
        NULL,
        8,
        &Task3_Handle,
        0);

    // Create Task4 observer
    xTaskCreatePinnedToCore(
        Task4_observer,
        "observer",
        16384,
        NULL,
        4,
        &Task4_Handle,
        0);

    // Create Task5 Serial
    xTaskCreatePinnedToCore(
        Task5_Serial,
        "Serial",
        16384,
        NULL,
        1,
        &Task5_Handle,
        0);

    // Create Task6 LiDAR
    xTaskCreatePinnedToCore(
        Task6_LiDAR,
        "LiDAR",
        16384,
        NULL,
        1,
        &Task6_Handle,
        1);
}

void Task1_WiFi_comm(void *pvParameters)
{

    // Configuring static IP
    if (!WiFi.config(local_IP, gateway, subnet, dns))
    {
        Serial.println("STA Failed to configure");
    }

    // Connecting to WiFi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        vTaskDelay(500);
        Serial.print(".");
    }
    Serial.println("WiFi connected.");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    // Starting the web server and WebSocket server
    server.on("/", handleRoot);
    server.begin();
    webSocketPhone.begin();
    webSocketPython.begin();

    webSocketPhone.onEvent(webSocketEventPhone);
    webSocketPython.onEvent(webSocketEventPython);

    // Start mDNS with the hostname
    if (!MDNS.begin("doggo3"))
    {
        Serial.println("Error setting up MDNS responder!");
    }
    Serial.println("mDNS responder started");

    while (1)
    {

        // Handle WebSocket communication
        server.handleClient();
        webSocketPhone.loop();
        webSocketPython.loop();
        vTaskDelay(1);
    }
}

void Task2_controller(void *pvParameters)
{
    //

    while (1)
    {

        vTaskDelay(1);
    }
}

void Task3_gait_IK(void *pvParameters)
{
    // init:
    servoDriverWire.begin(SDA_2, SCL_2);
    pwm.begin();
    pwm.setOscillatorFrequency(26600000); // it is something between 23 to 27MHz (needed to be calibrated  --> calibration 26600000Hz)
    pwm.setPWMFreq(50);                   // of the output PWM signal from channels (for each servo)
    GaitManager gaitManager;
    
    for (int servo_num = 0; servo_num < 14; servo_num++)
    {
        joints[servo_num].writeAngleDirect(jointDefault[servo_num]);
        // vTaskDelay(100);
    }

    // loop:
    while (1)
    {
        
        gaitManager.loop(xyz_cmd_array);
        cmd_angles[0] = -90;
        for (int servo_num = 0; servo_num < 14; servo_num++)
        {
            joints[servo_num].writeAngleDirect(cmd_angles[servo_num]);
            // Serial.print(joints[0].angleCmd);
            // Serial.print("\t");
            // Serial.println(joints[0].calibratedSignal);
        }
        vTaskDelay(50);

        // cmd_angles[0] = 0;
        // for (int servo_num = 0; servo_num < 14; servo_num++)
        // {
        //     joints[servo_num].writeAngleDirect(cmd_angles[servo_num]);
        //     Serial.print(joints[0].angleCmd);
        //     Serial.print("\t");
        //     Serial.println(joints[0].calibratedSignal);
        // }
        // vTaskDelay(500);
        // cmd_angles[0] = 80;
        // for (int servo_num = 0; servo_num < 14; servo_num++)
        // {
        //     joints[servo_num].writeAngleDirect(cmd_angles[servo_num]);
        //     Serial.print(joints[0].angleCmd);
        //     Serial.print("\t");
        //     Serial.println(joints[0].calibratedSignal);
        // }
        // vTaskDelay(500);
    
    }
}

void Task4_observer(void *pvParameters)
{
    // init:
    if (myMPU.begin())
    {
        myMPU.calculate_IMU_error();
    }
    else
    {
        Serial.println("MPU m4 48ala");
    }

    // loop:
    while (1)
    {
        // myMPU.updateOrientation();
        OrientationQuaternion = myMPU.getOrientationQuaternion();

        vTaskDelay(1);
    }
}

void Task5_Serial(void *pvParameters)
{
    int i = 0;
    Serial.begin(115200);

    while (1)
    {
        // Print remaining stack size
        UBaseType_t remainingStack = uxTaskGetStackHighWaterMark(NULL);
        // Serial.print("Remaining stack size: ");
        // Serial.println(remainingStack);
        // Serial.println("Hello " + String(i));

        i++;
        vTaskDelay(200); // Wait for 1 second
        // Serial.println(String(OrientationQuaternion.w) + String(" , ") + String(OrientationQuaternion.x) + String(" , ") + String(OrientationQuaternion.y) + String(" , ") + String(OrientationQuaternion.z));
        // vTaskDelay(1000); // Wait for 1 second
    }
}

void Task6_LiDAR(void *pvParameters)
{
    uint8_t image[50][50];

    while (1)
    {
        // Generate random 2D array
        for (int i = 0; i < 50; i++)
        {
            for (int j = 0; j < 50; j++)
            {
                int r = random(3);
                image[i][j] = (uint8_t)(r == 0 ? 0 : (r == 1 ? 128 : 255));
            }
        }
        // Flatten the 2D array into a 1D array
        uint8_t flatImage[50 * 50];
        for (int i = 0; i < 50; i++)
        {
            for (int j = 0; j < 50; j++)
            {
                flatImage[i * 50 + j] = image[i][j];
            }
        }

        // Send the flat image data as a binary message
        webSocketPhone.broadcastBIN(flatImage, 50 * 50);

        // Delay before generating a new image
        vTaskDelay(100 / 6);
    }
}

///////////////////////////////////////////////      HW     ///////////////////////////////////////////////

/////////////////////////////////////////////      mpu     /////////////////////////////////////////////
// in deg/sec:
// omegaX:  98.97159   ,    -0.0986408747 ,   -98.87974       |      100 ,  0,   -100
// omegaY: 100.24887   ,     1.1573488335 ,   -97.85251       |      100 ,  0,   -100
// omegaZ:  97.01528   ,    -1.8363988338 ,  -100.44398       |      100 ,  0,   -100
Data interpolDataOmegaX[3] = {
    {-98.88974 * M_PI / 180.0, -100 * M_PI / 180.0},
    {-0.0986408747 * M_PI / 180.0, 0 * M_PI / 180.0},
    {98.97759 * M_PI / 180.0, 100 * M_PI / 180.0}};
Data interpolDataOmegaY[3] = {
    {-97.85251 * M_PI / 180.0, -100 * M_PI / 180.0},
    {1.1573483350 * M_PI / 180.0, 0 * M_PI / 180.0},
    {100.26587 * M_PI / 180.0, 100 * M_PI / 180.0}};
Data interpolDataOmegaZ[3] = {
    {-100.44398 * M_PI / 180.0, -100 * M_PI / 180.0},
    {-1.8363988338 * M_PI / 180.0, 0 * M_PI / 180.0},
    {97.02428 * M_PI / 180.0, 100 * M_PI / 180.0}};

//////////////////////////////////////////////////////////////////////////////////////////
// in m/sec^2
// accX:    -9.68587   ,     0.05806     ,    10.0116064      |    -9.806 ,  0,      9.806
// accY:    -9.87989   ,    -0.05928     ,     9.8234674      |    -9.806 ,  0,      9.806
// accZ:    -9.9856136 ,    -0.05580     ,     9.78571        |    -9.806 ,  0,      9.806
Data interpolDataAccX[3] = {
    {-9.68587, -M_G},
    {0.05806, 0},
    {10.0116064, M_G}};
Data interpolDataAccY[3] = {
    {-9.87989, -M_G},
    {-0.05928, 0},
    {9.8234674, M_G}};
Data interpolDataAccZ[3] = {
    {-9.9856136, -M_G},
    {-0.05580, 0},
    {9.79634214, M_G}};

MPU6050_Custom myMPU(interpolDataOmegaX, interpolDataOmegaY, interpolDataOmegaZ, interpolDataAccX, interpolDataAccY, interpolDataAccZ);
// MPU6050_Custom myMPU(interpolDataOmegaX, interpolDataOmegaY, interpolDataOmegaZ, interpolDataAccX, interpolDataAccY, interpolDataAccZ);
// MPU6050_Custom myMPU(3, MPU6050_RANGE_250_DEG, MPU6050_RANGE_2_G,  MPU6050_BAND_44_HZ,  NULL,  NULL,  100,  0,  0.98,  3);

Quaternion_ OrientationQuaternion;
Quaternion_ OmegaQuaternion;
Quaternion_ alphaQuaternion;
Quaternion_ accelQuaternion;

double palmStates[4] = {0, 0, 0, 0};

double jointStates[14] = {
    0.001, 0.002, 0.003,
    0.004, 0.005, 0.006,
    0.007, 0.008, 0.009,
    0.010, 0.011, 0.012,
    0, 0};

double jointCmd[14] = {
    0, 0, 0,
    0, 0, 0,
    0, 0, 0,
    0, 0, 0,
    0, 0};

int servoPinList[14] = {
    0, 1, 2,
    3, 4, 5,
    6, 7, 8,
    9, 10, 11,
    12, 13};

// int servoPinList[14] =  {
//                             2,   3,   4,
//                             5,   6,   7,
//                             8,   9,  10,
//                             11, 12,  13,
//                             44, 45
//                         };

int potPinList[14] = {
    -1, -1, -1,
    -1, -1, -1,
    -1, -1, -1,
    -1, -1, -1,
    -1, -1};

// int potPinList[14] =    {
//                              4,    2,   15,
//                              0,   32,   33,
//                             25,   26,   27,
//                             14,   12,  13,
//                             -1,   -1
//                         };
// int potPinList[14] =    {
//                             A0,   A1,   A2,
//                             A3,   A4,   A5,
//                             A6,   A7,   A8,
//                             A9,   A10,  A11,
//                             -1,   -1
//                         };

#ifdef OLD_DOG

double jointOffsetList[14] = {90, 80, 55, 79, 93, 119, 73, 168, 63, 65, 55, 110, 98, -85};
double jointDirList[14] = {-1, 1, 1, -1, -1, -1, 1, 1, 1, 1, -1, -1, 1, -1};

#else

double jointDirList[14] = {
    1,
    1,
    -2 / 3.0,
    1,
    -1,
    2 / 3.0,
    -1,
    1,
    -2 / 3.0,
    -1,
    -1,
    2 / 3.0,
    1,
    -1,
};

double jointOffsetList[14] = {
    96, 90, -190,
    87, -97.2, 76,
    -81, 93, -194,
    -89, -94.9, 85,
    98, -85 //-97 --> -7
};

#endif

double jointLimitsLow[14] = {
    -90, -70, 0,
    -90, -70, 0,
    -90, -70, 0,
    -90, -70, 0,
    -90, -90};

double jointLimitsHigh[14] = {
    90, 85, 155,
    90, 85, 155,
    90, 85, 155,
    90, 85, 155,
    90, 90};

double jointDefault[14] = {
    0, -30, 60,
    0, -30, 60,
    0, -30, 60,
    0, -30, 60,
    0, 0};

JointServo joints[14] = {
    JointServo(servoPinList[frS], pwm, potPinList[frS], 0, -70, 2220, 524, jointDirList[frS], jointOffsetList[frS], jointLimitsLow[frS], jointLimitsHigh[frS], jointDefault[frS]),
    JointServo(servoPinList[frL], pwm, potPinList[frL], 0, -70, 1892, 260, jointDirList[frL], jointOffsetList[frL], jointLimitsLow[frL], jointLimitsHigh[frL], jointDefault[frL]),
    JointServo(servoPinList[frF], pwm, potPinList[frF], 0, 90, 4072, 1784, jointDirList[frF], jointOffsetList[frF], jointLimitsLow[frF], jointLimitsHigh[frF], jointDefault[frF]),

    JointServo(servoPinList[flS], pwm, potPinList[flS], 0, 70, 2312, 628, jointDirList[flS], jointOffsetList[flS], jointLimitsLow[flS], jointLimitsHigh[flS], jointDefault[flS]),
    JointServo(servoPinList[flL], pwm, potPinList[flL], 0, -70, 2168, 2748, jointDirList[flL], jointOffsetList[flL], jointLimitsLow[flL], jointLimitsHigh[flL], jointDefault[flL]),
    JointServo(servoPinList[flF], pwm, potPinList[flF], 0, 90, 4052, 2096, jointDirList[flF], jointOffsetList[flF], jointLimitsLow[flF], jointLimitsHigh[flF], jointDefault[flF]),

    JointServo(servoPinList[brS], pwm, potPinList[brS], 0, -70, 2092, 3940, jointDirList[brS], jointOffsetList[brS], jointLimitsLow[brS], jointLimitsHigh[brS], jointDefault[brS]),
    JointServo(servoPinList[brL], pwm, potPinList[brL], 0, -70, 1940, 3788, jointDirList[brL], jointOffsetList[brL], jointLimitsLow[brL], jointLimitsHigh[brL], jointDefault[brL]),
    JointServo(servoPinList[brF], pwm, potPinList[brF], 0, 90, 3680, 2108, jointDirList[brF], jointOffsetList[brF], jointLimitsLow[brF], jointLimitsHigh[brF], jointDefault[brF]),

    JointServo(servoPinList[blS], pwm, potPinList[blS], 0, 70, 2056, 3768, jointDirList[blS], jointOffsetList[blS], jointLimitsLow[blS], jointLimitsHigh[blS], jointDefault[blS]),
    JointServo(servoPinList[blL], pwm, potPinList[blL], 0, -70, 2116, 3728, jointDirList[blL], jointOffsetList[blL], jointLimitsLow[blL], jointLimitsHigh[blL], jointDefault[blL]),
    JointServo(servoPinList[blF], pwm, potPinList[blF], 0, 90, 0, 2216, jointDirList[blF], jointOffsetList[blF], jointLimitsLow[blF], jointLimitsHigh[blF], jointDefault[blF]),

    JointServo(servoPinList[12], pwm, potPinList[12], 0, 180, 0, 4065, jointDirList[12], jointOffsetList[12], jointLimitsLow[12], jointLimitsHigh[12], jointDefault[12]),
    JointServo(servoPinList[13], pwm, potPinList[13], 0, 180, 0, 4065, jointDirList[13], jointOffsetList[13], jointLimitsLow[13], jointLimitsHigh[13], jointDefault[13]),
};

palmSensor palms[4] = {
    palmSensor(36, 0, 2.14 * M_G, 0, 1780), // 2.14*M_G
    palmSensor(39, 0, 2.14 * M_G, 0, 1780), // 2.14*M_G
    palmSensor(34, 0, 2.14 * M_G, 0, 1780), // 2.14*M_G
    palmSensor(35, 0, 2.14 * M_G, 0, 1780), // 2.14*M_G
};

double cmd_angles[14] = {0};
