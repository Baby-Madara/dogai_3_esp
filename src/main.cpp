#include <system.h>


// void setup()    {    taskInitializers();    }
// void loop()     {}
// const char *ssid = "Fr7oo";
// const char *password = "25897463";










#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

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
WebSocketsServer webSocket(81);

void wifiTask(void *parameter);
void randomImageTask(void *parameter);
void handleRoot();
void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length);

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
void setup()
{
    Serial.begin(115200);
    delay(10);

    // Configuring static IP
    if (!WiFi.config(local_IP, gateway, subnet, dns))
    {
        Serial.println("STA Failed to configure");
    }

    // Connecting to WiFi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    Serial.println("WiFi connected.");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    // Starting the web server and WebSocket server
    server.on("/", handleRoot);
    server.begin();
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);

    // Starting tasks
    xTaskCreate(wifiTask, "wifiTask", 8192, NULL, 1, NULL);
    xTaskCreate(randomImageTask, "randomImageTask", 8192, NULL, 1, NULL);
}

void loop()
{}

void handleRoot()
{
    server.send(200, "text/html", htmlPage);
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length)
{
    if (type == WStype_TEXT)
    {
        Serial.printf("[%u] get Text: %s\n", num, payload);
        // Handle incoming data from WebSocket
    }
}

// Task to handle WiFi communication
void wifiTask(void *parameter)
{
    for (;;)
    {
        // Handle WebSocket communication
        server.handleClient();
        webSocket.loop();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

// Task to generate a random 2D array and send it as an image
void randomImageTask(void *parameter)
{
    uint8_t image[50][50];

    for (;;)
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
        webSocket.broadcastBIN(flatImage, 50 * 50);

        // Delay before generating a new image
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
