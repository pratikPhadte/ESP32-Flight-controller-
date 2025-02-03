#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <Wire.h>

// WiFi credentials
const char* ssid = "WiFi_SSID";
const char* password = "WiFi_Password";

// Create an AsyncWebServer object on port 80
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// Complementary filter gains
float comp_filter_gain = 0.991;

// Kalman filters for angle mode
volatile float AccX, AccY, AccZ;
volatile float AngleRoll, AnglePitch;
volatile float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2;
volatile float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2;
volatile float Kalman1DOutput[]={0,0};
volatile float DesiredAngleRoll, DesiredAnglePitch;
volatile float ErrorAngleRoll, ErrorAnglePitch;
volatile float PrevErrorAngleRoll, PrevErrorAnglePitch;
volatile float PrevItermAngleRoll, PrevItermAnglePitch;
volatile float RatePitch, RateRoll, RateYaw;
float RateCalibrationPitch, RateCalibrationRoll, RateCalibrationYaw,AccXCalibration,AccYCalibration,AccZCalibration;

uint32_t LoopTimer;
float t=0.004;      // time cycle

float complementaryAngleRoll = 0.0f;
float complementaryAnglePitch = 0.0f;

void onWebSocketMessage(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_DATA) {
        String message = String((char*)data).substring(0, len);
        
        // Parse only if the message starts with "comp_filter_gain:"
        if (message.startsWith("comp_filter_gain:")) {
            float newVal = message.substring(17).toFloat();
            
            // Ensure value is within the expected range
            if (newVal >= 0.00 && newVal <= 1.00) {
                comp_filter_gain = newVal;
                Serial.println("Updated Complementary Filter Gain: " + String(comp_filter_gain));
            } else {
                Serial.println("Invalid comp_filter_gain received: " + String(newVal));
            }
        }
    }
}


//Has debouce logic to prevent overshoot in the output of the webserver
// Updated HTML (Slider for Complementary Filter Gain)
const char webpage[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Complementary Filter Control</title>
  <style>
    body { font-family: Arial, sans-serif; text-align: center; background-color: #f4f4f4; }
    h1 { color: #333; }
    .slider-container {
      display: flex;
      align-items: center;
      justify-content: center;
      margin: 30px 0;
    }
    .slider-container label {
      font-size: 18px;
      margin: 0 15px;
      font-weight: bold;
    }
    input[type="range"] {
      width: 60%;
      height: 10px;
      -webkit-appearance: none;
      background: #ddd;
      border-radius: 5px;
      outline: none;
    }
    input[type="range"]::-webkit-slider-thumb {
      -webkit-appearance: none;
      width: 20px;
      height: 20px;
      background: #007bff;
      border-radius: 50%;
      cursor: pointer;
    }
    p#status {
      font-size: 18px;
      font-weight: bold;
      color: green;
    }
  </style>
</head>
<body>
  <h1>Complementary Filter Gain Control</h1>
  
  <div class="slider-container">
    <label>Accelerometer</label>
    <input type="range" id="comp_filter_gain" min="0.00" max="1.00" step="0.001" value="0.991">
    <label>Gyroscope</label>
  </div>
  <p>Current Value: <span id="filterValue">0.991</span></p>

  <p id="status">Not Connected</p>

  <script>
    const ws = new WebSocket(`ws://${location.hostname}/ws`);
    const filterSlider = document.getElementById("comp_filter_gain");
    const filterValue = document.getElementById("filterValue");

    ws.onopen = () => document.getElementById("status").innerText = "Connected";

    let debounceTimer; // Declare debounceTimer to handle the delay between inputs

    filterSlider.addEventListener("input", () => {
        clearTimeout(debounceTimer); // Reset the timer on new input
        filterValue.innerText = filterSlider.value; // Update the displayed value immediately

        // Send data only after 100ms of no further input (debounce)
        debounceTimer = setTimeout(() => {
            ws.send(`comp_filter_gain:${filterSlider.value}`); // Send updated value to WebSocket server
        }, 100); 
    });
  </script>
</body>
</html>

)rawliteral";

void setup() {
  Serial.begin(115200);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  Serial.println(WiFi.localIP());

  // Setup WebSocket handler
  ws.onEvent(onWebSocketMessage);
  server.addHandler(&ws);

  // Serve HTML
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", webpage);
  });

  // Start the server
  server.begin();

  int led_time = 100;
  pinMode(15, OUTPUT);
  for (int i = 0; i < 8; i++) {
    digitalWrite(15, i % 2);
    delay(led_time);
  }

  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  RateCalibrationRoll = 0.33;
  RateCalibrationPitch = -0.95;
  RateCalibrationYaw = -2.06;
  AccXCalibration = 0.05;
  AccYCalibration = 0.01;
  AccZCalibration = -0.06;

  LoopTimer = micros();
}

void loop() {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();

  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();

  RateRoll = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw = (float)GyroZ / 65.5;
  AccX = (float)AccXLSB / 4096;
  AccY = (float)AccYLSB / 4096;
  AccZ = (float)AccZLSB / 4096;

  RateRoll -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw -= RateCalibrationYaw;
  AccX -= AccXCalibration;
  AccY -= AccYCalibration;
  AccZ -= AccZCalibration;

  AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 57.29;
  AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 57.29;

  complementaryAngleRoll = comp_filter_gain * (complementaryAngleRoll + RateRoll * t) + (1 - comp_filter_gain) * AngleRoll;
  complementaryAnglePitch = comp_filter_gain * (complementaryAnglePitch + RatePitch * t) + (1 - comp_filter_gain) * AnglePitch;

  Serial.println(complementaryAngleRoll);

  while (micros() - LoopTimer < (t * 1000000));
  LoopTimer = micros();
}
