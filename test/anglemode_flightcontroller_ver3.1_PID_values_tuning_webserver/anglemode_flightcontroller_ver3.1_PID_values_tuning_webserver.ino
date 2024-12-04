
//THERE IS NO WARRANTY FOR THE SOFTWARE, TO THE EXTENT PERMITTED BY APPLICABLE LAW. EXCEPT WHEN OTHERWISE STATED IN WRITING THE COPYRIGHT HOLDERS AND/OR 
//OTHER PARTIES PROVIDE THE SOFTWARE “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
//OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. THE ENTIRE RISK AS TO THE QUALITY AND PERFORMANCE OF THE SOFTWARE IS WITH THE CUSTOMER. SHOULD THE 
//SOFTWARE PROVE DEFECTIVE, THE CUSTOMER ASSUMES THE COST OF ALL NECESSARY SERVICING, REPAIR, OR CORRECTION EXCEPT TO THE EXTENT SET OUT UNDER THE HARDWARE WARRANTY IN THESE TERMS.

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <Wire.h>
#include <ESP32Servo.h> // Change to the standard Servo library for ESP32


// REPLACE WITH YOUR NETWORK CREDENTIALS
//COnnect your PC/mobile to this wifi and open the IP address from the serial monitor in your browser.
 const char* ssid = "your_wifi_name";
 const char* password = "your_wifi_password";

float PRateRoll = 0.625; 
float IRateRoll = 2.1;
float DRateRoll = 0.008;

float PAngleRoll=2; 
float IAngleRoll=0;
float DAngleRoll=0.007; 

float PRateYaw = 4;
float IRateYaw = 3;
float DRateYaw = 0;

int ESCfreq=500;

float PRatePitch = PRateRoll;
float IRatePitch = IRateRoll;
float DRatePitch = DRateRoll;
float PAnglePitch=PAngleRoll;
 float IAnglePitch=IAngleRoll;
 float DAnglePitch=DAngleRoll;
uint32_t LoopTimer;
float t=0.004;      //time cycle


volatile float RatePitch, RateRoll, RateYaw;
volatile float RateCalibrationPitch, RateCalibrationRoll, RateCalibrationYaw,AccXCalibration,AccYCalibration,AccZCalibration;
int RateCalibrationNumber;

Servo mot1;
Servo mot2;
Servo mot3;
Servo mot4;
const int mot1_pin = 13;
const int mot2_pin = 12;
const int mot3_pin = 14; //14 for perf board
const int mot4_pin = 27;

volatile uint32_t current_time;
volatile uint32_t last_channel_1 = 0;
volatile uint32_t last_channel_2 = 0;
volatile uint32_t last_channel_3 = 0;
volatile uint32_t last_channel_4 = 0;
volatile uint32_t last_channel_5 = 0;
volatile uint32_t last_channel_6 = 0;
volatile uint32_t timer_1;
volatile uint32_t timer_2;
volatile uint32_t timer_3;
volatile uint32_t timer_4;
volatile uint32_t timer_5;
volatile uint32_t timer_6;
volatile int ReceiverValue[6]; // Increase the array size to 6 for Channel 1 to Channel 6
volatile int channel_1_pwm_prev;
volatile int channel_2_pwm_prev;
volatile int channel_3_pwm_prev;
volatile int channel_4_pwm_prev;
volatile  int channel_1_pwm;
volatile  int channel_2_pwm;
 volatile int channel_3_pwm;
 volatile int channel_4_pwm;
float b=0.7;

  unsigned long channel_1_fs = 1500; //thro
unsigned long channel_2_fs = 1500; //ail
unsigned long channel_3_fs = 1000; //elev
unsigned long channel_4_fs = 1500; //rudd
unsigned long channel_5_fs = 1000; //gear, greater than 1500 = throttle cut
unsigned long channel_6_fs = 1000; //aux1

const int channel_1_pin = 34;
const int channel_2_pin = 35;
const int channel_3_pin = 32;
const int channel_4_pin = 33;
const int channel_5_pin = 25;
const int channel_6_pin = 26;

volatile float PtermRoll;
volatile float ItermRoll;
volatile float DtermRoll;
volatile float PIDOutputRoll;
volatile float PtermPitch;
volatile float ItermPitch;
volatile float DtermPitch;
volatile float PIDOutputPitch;
volatile float PtermYaw;
volatile float ItermYaw;
volatile float DtermYaw;
volatile float PIDOutputYaw;
volatile float KalmanGainPitch;
volatile float KalmanGainRoll;

int ThrottleIdle = 1170;
int ThrottleCutOff = 1000;

volatile float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
volatile float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
volatile float InputRoll, InputThrottle, InputPitch, InputYaw;
volatile float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
volatile float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
volatile float PIDReturn[] = {0, 0, 0};

float complementaryAngleRoll = 0.0f;
float complementaryAnglePitch = 0.0f;

//Kalman filters for angle mode
volatile float AccX, AccY, AccZ;
volatile float AngleRoll, AnglePitch;
volatile float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2;
volatile float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2;
volatile float Kalman1DOutput[]={0,0};
volatile float DesiredAngleRoll, DesiredAnglePitch;
volatile float ErrorAngleRoll, ErrorAnglePitch;
volatile float PrevErrorAngleRoll, PrevErrorAnglePitch;
volatile float PrevItermAngleRoll, PrevItermAnglePitch;

volatile float MotorInput1, MotorInput2, MotorInput3, MotorInput4;

void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState=KalmanState + (t*KalmanInput);
  KalmanUncertainty=KalmanUncertainty + (t*t*4*4); //here 4 is the vairnece of IMU i.e 4 deg/s
  float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 3 * 3); //std deviation of error is 3 deg
  KalmanState=KalmanState+KalmanGain * (KalmanMeasurement-KalmanState);
  KalmanUncertainty=(1-KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0]=KalmanState; 
  Kalman1DOutput[1]=KalmanUncertainty;
}


void channelInterruptHandler()
{
current_time = micros(); if (digitalRead(channel_1_pin)) { if (last_channel_1 == 0) { last_channel_1 = 1; timer_1 = current_time; } } else if (last_channel_1 == 1) { last_channel_1 = 0; ReceiverValue[0] = current_time - timer_1; } 
if (digitalRead(channel_2_pin)) { if (last_channel_2 == 0) { last_channel_2 = 1; timer_2 = current_time; } } else if (last_channel_2 == 1) { last_channel_2 = 0; ReceiverValue[1] = current_time - timer_2; } 
if (digitalRead(channel_3_pin)) { if (last_channel_3 == 0) { last_channel_3 = 1; timer_3 = current_time; } } else if (last_channel_3 == 1) { last_channel_3 = 0; ReceiverValue[2] = current_time - timer_3; } 
if (digitalRead(channel_4_pin)) { if (last_channel_4 == 0) { last_channel_4 = 1; timer_4 = current_time; } } else if (last_channel_4 == 1) { last_channel_4 = 0; ReceiverValue[3] = current_time - timer_4; } 
if (digitalRead(channel_5_pin)) { if (last_channel_5 == 0) { last_channel_5 = 1; timer_5 = current_time; } } else if (last_channel_5 == 1) { last_channel_5 = 0; ReceiverValue[4] = current_time - timer_5; } 
if (digitalRead(channel_6_pin)) { if (last_channel_6 == 0) { last_channel_6 = 1; timer_6 = current_time; } } else if (last_channel_6 == 1) { last_channel_6 = 0; ReceiverValue[5] = current_time - timer_6; }
}

void gyro_signals(void)
{
Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(); 
  Wire.requestFrom(0x68,6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); 
  Wire.write(0x8);
  Wire.endTransmission();                                                   
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  int16_t GyroX=Wire.read()<<8 | Wire.read();
  int16_t GyroY=Wire.read()<<8 | Wire.read();
  int16_t GyroZ=Wire.read()<<8 | Wire.read();
  RateRoll=(float)GyroX/65.5;
  RatePitch=(float)GyroY/65.5;
  RateYaw=(float)GyroZ/65.5;
  AccX=(float)AccXLSB/4096;
  AccY=(float)AccYLSB/4096;
  AccZ=(float)AccZLSB/4096;
  AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*57.29; //*1/(3.142/180);
  AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*57.29;
}

void pid_equation(float Error, float P, float I, float D, float PrevError, float PrevIterm)
{
  float Pterm = P * Error;
  float Iterm = PrevIterm +( I * (Error + PrevError) * (t/2));
  if (Iterm > 400)
  {
    Iterm = 400;
  }
  else if (Iterm < -400)
  {
  Iterm = -400;
  }
  float Dterm = D *( (Error - PrevError)/t);
  float PIDOutput = Pterm + Iterm + Dterm;
  if (PIDOutput > 400)
  {
    PIDOutput = 400;
  }
  else if (PIDOutput < -400)
  {
    PIDOutput = -400;
  }
  PIDReturn[0] = PIDOutput;
  PIDReturn[1] = Error;
  PIDReturn[2] = Iterm;
}

//WIFI tuning global code
AsyncWebServer server(80);


const char* PARAM_P_GAIN = "pGain";   //For Pitch & Roll RATE
const char* PARAM_I_GAIN = "iGain";
const char* PARAM_D_GAIN = "dGain";

const char* PARAM_P_A_GAIN = "pAGain";   //For Pitch & Roll ANGLE
const char* PARAM_I_A_GAIN = "iAGain";
const char* PARAM_D_A_GAIN = "dAGain";

const char* PARAM_P_YAW = "pYaw";     //For Yaw
const char* PARAM_I_YAW = "iYaw";
const char* PARAM_D_YAW = "dYaw";

const char* PARAM_TIME_CYCLE = "tc";  //Computation time cycle

// HTML web page to handle 6 input fields of PID gains
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
  <title>ESP Input Form</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <script>
    function submitMessage() {
      alert("Saved value to ESP SPIFFS");
      setTimeout(function(){ document.location.reload(false); }, 500);
    }
  </script></head><body>

     <form action="/get" target="hidden-form"><br>
    ESP32 Webserver for PID Gain value tuning of Quadcopter 
  </form><br><br>

  <form action="/get" target="hidden-form">
    P Pitch & Roll Gain (current value %pGain%): <input type="number" step="any" name="pGain">
    <input type="submit" value="Submit" onclick="submitMessage()">
  </form><br>
  <form action="/get" target="hidden-form">
    I Pitch & Roll Gain (current value %iGain%): <input type="number" step="any" name="iGain">
    <input type="submit" value="Submit" onclick="submitMessage()">
  </form><br>
  <form action="/get" target="hidden-form">
    D Pitch & Roll Gain (current value %dGain%): <input type="number" step="any" name="dGain">
    <input type="submit" value="Submit" onclick="submitMessage()">
  </form><br>
  <form action="/get" target="hidden-form">
    P Pitch & Roll Angle Gain (current value %pAGain%): <input type="number" step="any" name="pAGain">
    <input type="submit" value="Submit" onclick="submitMessage()">
  </form><br>
    <form action="/get" target="hidden-form">
    I Pitch & Roll Angle Gain (current value %iAGain%): <input type="number" step="any" name="iAGain">
    <input type="submit" value="Submit" onclick="submitMessage()">
  </form><br>
    <form action="/get" target="hidden-form">
    D Pitch & Roll Angle Gain (current value %dAGain%): <input type="number" step="any" name="dAGain">
    <input type="submit" value="Submit" onclick="submitMessage()">
  </form><br>
  <form action="/get" target="hidden-form">
    P Yaw Gain (current value %pYaw%): <input type="number" step="any" name="pYaw">
    <input type="submit" value="Submit" onclick="submitMessage()">
  </form><br>
  <form action="/get" target="hidden-form">
    I Yaw Gain (current value %iYaw%): <input type="number" step="any" name="iYaw">
    <input type="submit" value="Submit" onclick="submitMessage()">
  </form><br>
  <form action="/get" target="hidden-form">
    D Yaw Gain (current value %dYaw%): <input type="number" step="any" name="dYaw">
    <input type="submit" value="Submit" onclick="submitMessage()">
  </form><br><br>
    <form action="/get" target="hidden-form">
    Time cycle (current value %tc%): <input type="number" step="any" name="tc">
    <input type="submit" value="Submit" onclick="submitMessage()">
  </form><br><br>

  <iframe style="display:none" name="hidden-form"></iframe>
</body></html>)rawliteral";

void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

String readFile(fs::FS &fs, const char * path){
  Serial.printf("Reading file: %s\r\n", path);
  File file = fs.open(path, "r");
  if(!file || file.isDirectory()){
    Serial.println("- empty file or failed to open file");
    return String();
  }
  Serial.println("- read from file:");
  String fileContent;
  while(file.available()){
    fileContent+=String((char)file.read());
  }
  file.close();
  Serial.println(fileContent);
  return fileContent;
}

void writeFile(fs::FS &fs, const char * path, const char * message){
  Serial.printf("Writing file: %s\r\n", path);
  File file = fs.open(path, "w");
  if(!file){
    Serial.println("- failed to open file for writing");
    return;
  }
  if(file.print(message)){
    Serial.println("- file written");
  } else {
    Serial.println("- write failed");
  }
  file.close();
}

// Replaces placeholder with stored values
String processor(const String& var){
  //Serial.println(var);
if(var == "pGain"){
    return readFile(SPIFFS, "/pGain.txt");
}
else if(var == "iGain"){
    return readFile(SPIFFS, "/iGain.txt");
}
else if(var == "dGain"){
    return readFile(SPIFFS, "/dGain.txt");
}
else if(var == "pAGain"){
    return readFile(SPIFFS, "/pAGain.txt");
}
else if(var == "iAGain"){
    return readFile(SPIFFS, "/iAGain.txt");
}
else if(var == "dAGain"){
    return readFile(SPIFFS, "/dAGain.txt");
}
else if(var == "pYaw"){
    return readFile(SPIFFS, "/pYaw.txt");
}
else if(var == "dYaw"){
    return readFile(SPIFFS, "/dYaw.txt");
}
else if(var == "iYaw"){
    return readFile(SPIFFS, "/iYaw.txt");
}
else if(var == "tc"){
    return readFile(SPIFFS, "/tc.txt");
}

}





void setup(void) {
  
Serial.begin(115200);

//WIFI server setup START
// Initialize SPIFFS
  #ifdef ESP32
    if(!SPIFFS.begin(true)){
      Serial.println("An Error has occurred while mounting SPIFFS");
      return;
    }
  #else
    if(!SPIFFS.begin()){
      Serial.println("An Error has occurred while mounting SPIFFS");
      return;
    }
  #endif
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("WiFi Failed!");
    return;
  }
  Serial.println();
  Serial.print("IP Address: ");

  Serial.println(WiFi.localIP());
  delay(2000);
  // Send web page with input fields to client
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html, processor);
  });

  // Send a GET request to <ESP_IP>/get?inputString=<inputMessage>
  server.on("/get", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputMessage;
    // GET P Gain value on <ESP_IP>/get?pGain=<inputMessage>
if (request->hasParam(PARAM_P_GAIN)) {
    inputMessage = request->getParam(PARAM_P_GAIN)->value();
    writeFile(SPIFFS, "/pGain.txt", inputMessage.c_str());
}
// GET I Gain value on <ESP_IP>/get?iGain=<inputMessage>
else if (request->hasParam(PARAM_I_GAIN)) {
    inputMessage = request->getParam(PARAM_I_GAIN)->value();
    writeFile(SPIFFS, "/iGain.txt", inputMessage.c_str());
}
// GET D Gain value on <ESP_IP>/get?dGain=<inputMessage>
else if (request->hasParam(PARAM_D_GAIN)) {
    inputMessage = request->getParam(PARAM_D_GAIN)->value();
    writeFile(SPIFFS, "/dGain.txt", inputMessage.c_str());
}
else if (request->hasParam(PARAM_P_A_GAIN)) {
    inputMessage = request->getParam(PARAM_P_A_GAIN)->value();
    writeFile(SPIFFS, "/pAGain.txt", inputMessage.c_str());
}
else if (request->hasParam(PARAM_I_A_GAIN)) {
    inputMessage = request->getParam(PARAM_I_A_GAIN)->value();
    writeFile(SPIFFS, "/iAGain.txt", inputMessage.c_str());
}
else if (request->hasParam(PARAM_D_A_GAIN)) {
    inputMessage = request->getParam(PARAM_D_A_GAIN)->value();
    writeFile(SPIFFS, "/dAGain.txt", inputMessage.c_str());
}
else if (request->hasParam(PARAM_P_YAW)) {
    inputMessage = request->getParam(PARAM_P_YAW)->value();
    writeFile(SPIFFS, "/pYaw.txt", inputMessage.c_str());
  } else if (request->hasParam(PARAM_I_YAW)) {
    inputMessage = request->getParam(PARAM_I_YAW)->value();
    writeFile(SPIFFS, "/iYaw.txt", inputMessage.c_str());
  } else if (request->hasParam(PARAM_D_YAW)) {
    inputMessage = request->getParam(PARAM_D_YAW)->value();
    writeFile(SPIFFS, "/dYaw.txt", inputMessage.c_str());
  }
 else if (request->hasParam(PARAM_TIME_CYCLE)) {
    inputMessage = request->getParam(PARAM_TIME_CYCLE)->value();
    writeFile(SPIFFS, "/tc.txt", inputMessage.c_str());
  }
    else {
      inputMessage = "No message sent";
    }
    Serial.println(inputMessage);
    request->send(200, "text/text", inputMessage);
  });

  server.onNotFound(notFound);
  server.begin();
//WIFI server setup END

int led_time=100;
 pinMode(15, OUTPUT);
  digitalWrite(15, LOW);
  delay(led_time);
  digitalWrite(15, HIGH);
  delay(led_time);
  digitalWrite(15, LOW);
  delay(led_time);
  digitalWrite(15, HIGH);
  delay(led_time);
  digitalWrite(15, LOW);
  delay(led_time);
  digitalWrite(15, HIGH);
  delay(led_time);
  digitalWrite(15, LOW);
  delay(led_time);
  digitalWrite(15, HIGH);
  delay(led_time);
  digitalWrite(15, LOW);
  delay(led_time);


  pinMode(channel_1_pin, INPUT_PULLUP);
  pinMode(channel_2_pin, INPUT_PULLUP);
  pinMode(channel_3_pin, INPUT_PULLUP);
  pinMode(channel_4_pin, INPUT_PULLUP);
  pinMode(channel_5_pin, INPUT_PULLUP);
  pinMode(channel_6_pin, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(channel_1_pin), channelInterruptHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channel_2_pin), channelInterruptHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channel_3_pin), channelInterruptHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channel_4_pin), channelInterruptHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channel_5_pin), channelInterruptHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channel_6_pin), channelInterruptHandler, CHANGE);
  delay(100);
  
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

 	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);

  delay(1000);
  mot1.attach(mot1_pin,1000,2000);
  delay(1000);
  mot1.setPeriodHertz(ESCfreq);
  delay(100);
  mot2.attach(mot2_pin,1000,2000);
  delay(1000);
  mot2.setPeriodHertz(ESCfreq);
  delay(100);
  mot3.attach(mot3_pin,1000,2000);
  delay(1000);
  mot3.setPeriodHertz(ESCfreq);
  delay(100);
  mot4.attach(mot4_pin,1000,2000);
  delay(1000);
  mot4.setPeriodHertz(ESCfreq);
  delay(100);

  mot1.writeMicroseconds(1000);
  mot2.writeMicroseconds(1000);
  mot3.writeMicroseconds(1000);
  mot4.writeMicroseconds(1000);
   delay(500);
  digitalWrite(15, LOW);
  digitalWrite(15, HIGH);
  delay(500);
  digitalWrite(15, LOW);
  delay(500);

RateCalibrationRoll=0.27;
RateCalibrationPitch=-0.85;
RateCalibrationYaw=-2.09;
AccXCalibration=0.03;
AccYCalibration=0.01;
AccZCalibration=-0.07;

LoopTimer = micros();

}

void loop(void) {

  if(ReceiverValue[4] > 1500 ) //channel 5 for uploading values
{

PRateRoll = readFile(SPIFFS, "/pGain.txt").toFloat();
IRateRoll = readFile(SPIFFS, "/iGain.txt").toFloat();
DRateRoll = readFile(SPIFFS, "/dGain.txt").toFloat();

PRatePitch = PRateRoll;
IRatePitch = IRateRoll;
DRatePitch = DRateRoll;

PAngleRoll=readFile(SPIFFS, "/pAGain.txt").toFloat();; 
IAngleRoll=readFile(SPIFFS, "/iAGain.txt").toFloat();; 
DAngleRoll=readFile(SPIFFS, "/dAGain.txt").toFloat();; 

PAnglePitch=PAngleRoll;
IAnglePitch=IAngleRoll;
DAnglePitch=DAngleRoll;

PRateYaw = readFile(SPIFFS, "/pYaw.txt").toFloat();
IRateYaw = readFile(SPIFFS, "/iYaw.txt").toFloat();
DRateYaw = readFile(SPIFFS, "/dYaw.txt").toFloat();

t = readFile(SPIFFS, "/tc.txt").toFloat();
}


  //enter your loop code here
Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(); 
  Wire.requestFrom(0x68,6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); 
  Wire.write(0x8);
  Wire.endTransmission();                                                   
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  int16_t GyroX=Wire.read()<<8 | Wire.read();
  int16_t GyroY=Wire.read()<<8 | Wire.read();
  int16_t GyroZ=Wire.read()<<8 | Wire.read();
  RateRoll=(float)GyroX/65.5;
  RatePitch=(float)GyroY/65.5;
  RateYaw=(float)GyroZ/65.5;
  AccX=(float)AccXLSB/4096;
  AccY=(float)AccYLSB/4096;
  AccZ=(float)AccZLSB/4096;


RateRoll -= RateCalibrationRoll;
RatePitch -= RateCalibrationPitch;
RateYaw -= RateCalibrationYaw;

AccX -= AccXCalibration ;
AccY -= AccYCalibration ;
AccZ -= AccZCalibration;

  AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*57.29;
  AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*57.29;

// // Inlined Kalman Filter computation in the loop
// KalmanAngleRoll += t * RateRoll;
// KalmanUncertaintyAngleRoll += t * t * 16; // Variance of IMU (4 deg/s) squared
// KalmanGainRoll = KalmanUncertaintyAngleRoll / (KalmanUncertaintyAngleRoll + 9); // Error variance (3 deg) squared
// KalmanAngleRoll += KalmanGainRoll * (AngleRoll - KalmanAngleRoll);
// KalmanUncertaintyAngleRoll *= (1 - KalmanGainRoll);

// // Set output for Roll Kalman
// Kalman1DOutput[0] = KalmanAngleRoll;
// Kalman1DOutput[1] = KalmanUncertaintyAngleRoll;

// // Inlined Kalman Filter computation for Pitch
// KalmanAnglePitch += t * RatePitch;
// KalmanUncertaintyAnglePitch += t * t * 16; // Variance of IMU (4 deg/s) squared
// KalmanGainPitch = KalmanUncertaintyAnglePitch / (KalmanUncertaintyAnglePitch + 9); // Error variance (3 deg) squared
// KalmanAnglePitch += KalmanGainPitch * (AnglePitch - KalmanAnglePitch);
// KalmanUncertaintyAnglePitch *= (1 - KalmanGainPitch);

// // Set output for Pitch Kalman
// Kalman1DOutput[0] = KalmanAnglePitch;
// Kalman1DOutput[1] = KalmanUncertaintyAnglePitch;

// KalmanAngleRoll = (KalmanAngleRoll > 20) ? 20 : ((KalmanAngleRoll < -20) ? -20 : KalmanAngleRoll);
// KalmanAnglePitch = (KalmanAnglePitch > 20) ? 20 : ((KalmanAnglePitch < -20) ? -20 : KalmanAnglePitch);


complementaryAngleRoll=0.991*(complementaryAngleRoll+RateRoll*t) + 0.009*AngleRoll;
complementaryAnglePitch=0.991*(complementaryAnglePitch+RatePitch*t) + 0.009*AnglePitch;
// Clamping complementary filter roll angle to ±20 degrees
complementaryAngleRoll = (complementaryAngleRoll > 20) ? 20 : ((complementaryAngleRoll < -20) ? -20 : complementaryAngleRoll);
complementaryAnglePitch = (complementaryAnglePitch > 20) ? 20 : ((complementaryAnglePitch < -20) ? -20 : complementaryAnglePitch);



DesiredAngleRoll=0.1*(ReceiverValue[0]-1500);
DesiredAnglePitch=0.1*(ReceiverValue[1]-1500);
InputThrottle=ReceiverValue[2];
DesiredRateYaw=0.15*(ReceiverValue[3]-1500);


// Inlined PID equation for Roll
ErrorAngleRoll = DesiredAngleRoll - complementaryAngleRoll;
PtermRoll = PAngleRoll * ErrorAngleRoll;
ItermRoll = PrevItermAngleRoll + (IAngleRoll * (ErrorAngleRoll + PrevErrorAngleRoll) * (t / 2));
ItermRoll = (ItermRoll > 400) ? 400 : ((ItermRoll < -400) ? -400 : ItermRoll);
DtermRoll = DAngleRoll * ((ErrorAngleRoll - PrevErrorAngleRoll) / t);
PIDOutputRoll = PtermRoll + ItermRoll + DtermRoll;
PIDOutputRoll = (PIDOutputRoll > 400) ? 400 : ((PIDOutputRoll < -400) ? -400 : PIDOutputRoll);
DesiredRateRoll = PIDOutputRoll;
PrevErrorAngleRoll = ErrorAngleRoll;
PrevItermAngleRoll = ItermRoll;

ErrorAnglePitch = DesiredAnglePitch - complementaryAnglePitch;
PtermPitch = PAnglePitch * ErrorAnglePitch;
ItermPitch = PrevItermAnglePitch + (IAnglePitch * (ErrorAnglePitch + PrevErrorAnglePitch) * (t / 2));
ItermPitch = (ItermPitch > 400) ? 400 : ((ItermPitch < -400) ? -400 : ItermPitch);
DtermPitch = DAnglePitch * ((ErrorAnglePitch - PrevErrorAnglePitch) / t);
PIDOutputPitch = PtermPitch + ItermPitch + DtermPitch;
PIDOutputPitch = (PIDOutputPitch > 400) ? 400 : ((PIDOutputPitch < -400) ? -400 : PIDOutputPitch);
DesiredRatePitch = PIDOutputPitch;
PrevErrorAnglePitch = ErrorAnglePitch;
PrevItermAnglePitch = ItermPitch;

// Compute errors
ErrorRateRoll = DesiredRateRoll - RateRoll;
ErrorRatePitch = DesiredRatePitch - RatePitch;
ErrorRateYaw = DesiredRateYaw - RateYaw;

// Roll Axis PID
PtermRoll = PRateRoll * ErrorRateRoll;
ItermRoll = PrevItermRateRoll + (IRateRoll * (ErrorRateRoll + PrevErrorRateRoll) * (t / 2));
ItermRoll = (ItermRoll > 400) ? 400 : ((ItermRoll < -400) ? -400 : ItermRoll);
DtermRoll = DRateRoll * ((ErrorRateRoll - PrevErrorRateRoll) / t);
PIDOutputRoll = PtermRoll + ItermRoll + DtermRoll;
PIDOutputRoll = (PIDOutputRoll > 400) ? 400 : ((PIDOutputRoll < -400) ? -400 : PIDOutputRoll);

// Update output and previous values for Roll
InputRoll = PIDOutputRoll;
PrevErrorRateRoll = ErrorRateRoll;
PrevItermRateRoll = ItermRoll;

// Pitch Axis PID
PtermPitch = PRatePitch * ErrorRatePitch;
ItermPitch = PrevItermRatePitch + (IRatePitch * (ErrorRatePitch + PrevErrorRatePitch) * (t / 2));
ItermPitch = (ItermPitch > 400) ? 400 : ((ItermPitch < -400) ? -400 : ItermPitch);
DtermPitch = DRatePitch * ((ErrorRatePitch - PrevErrorRatePitch) / t);
PIDOutputPitch = PtermPitch + ItermPitch + DtermPitch;
PIDOutputPitch = (PIDOutputPitch > 400) ? 400 : ((PIDOutputPitch < -400) ? -400 : PIDOutputPitch);

// Update output and previous values for Pitch
InputPitch = PIDOutputPitch;
PrevErrorRatePitch = ErrorRatePitch;
PrevItermRatePitch = ItermPitch;

// Yaw Axis PID
PtermYaw = PRateYaw * ErrorRateYaw;
ItermYaw = PrevItermRateYaw + (IRateYaw * (ErrorRateYaw + PrevErrorRateYaw) * (t / 2));
ItermYaw = (ItermYaw > 400) ? 400 : ((ItermYaw < -400) ? -400 : ItermYaw);  // Clamp ItermYaw to [-400, 400]
DtermYaw = DRateYaw * ((ErrorRateYaw - PrevErrorRateYaw) / t);
PIDOutputYaw = PtermYaw + ItermYaw + DtermYaw;
PIDOutputYaw = (PIDOutputYaw > 400) ? 400 : ((PIDOutputYaw < -400) ? -400 : PIDOutputYaw);  // Clamp PIDOutputYaw to [-400, 400]


// Update output and previous values for Yaw
InputYaw = PIDOutputYaw;
PrevErrorRateYaw = ErrorRateYaw;
PrevItermRateYaw = ItermYaw;


  if (InputThrottle > 1800)
  {
    InputThrottle = 1800;
  }

  
  MotorInput1 =  (InputThrottle - InputRoll - InputPitch - InputYaw); // front right - counter clockwise
  MotorInput2 =  (InputThrottle - InputRoll + InputPitch + InputYaw); // rear right - clockwise
  MotorInput3 =  (InputThrottle + InputRoll + InputPitch - InputYaw); // rear left  - counter clockwise
  MotorInput4 =  (InputThrottle + InputRoll - InputPitch + InputYaw); //front left - clockwise


  if (MotorInput1 > 2000)
  {
    MotorInput1 = 1999;
  }

  if (MotorInput2 > 2000)
  {
    MotorInput2 = 1999;
  }

  if (MotorInput3 > 2000)
  {
    MotorInput3 = 1999;
  }

  if (MotorInput4 > 2000)
  {
    MotorInput4 = 1999;
  }


// int ThrottleIdle = 1150;
// int ThrottleCutOff = 1000;
  if (MotorInput1 < ThrottleIdle)
  {
    MotorInput1 = ThrottleIdle;
  }
  if (MotorInput2 < ThrottleIdle)
  {
    MotorInput2 = ThrottleIdle;
  }
  if (MotorInput3 < ThrottleIdle)
  {
    MotorInput3 = ThrottleIdle;
  }
  if (MotorInput4 < ThrottleIdle)
  {
    MotorInput4 = ThrottleIdle;
  }

 if (ReceiverValue[2] < 1030 ) // dont Arm the motors
  {
   
  MotorInput1 = ThrottleCutOff;
  MotorInput2 = ThrottleCutOff;
  MotorInput3 = ThrottleCutOff;
  MotorInput4 = ThrottleCutOff;

  PrevErrorRateRoll=0; PrevErrorRatePitch=0; PrevErrorRateYaw=0;
  PrevItermRateRoll=0; PrevItermRatePitch=0; PrevItermRateYaw=0;
  PrevErrorAngleRoll=0; PrevErrorAnglePitch=0;    
  PrevItermAngleRoll=0; PrevItermAnglePitch=0;
  
  }

// Calculate motor control values directly
mot1.writeMicroseconds(MotorInput1);
mot2.writeMicroseconds(MotorInput2);
mot3.writeMicroseconds(MotorInput3);
mot4.writeMicroseconds(MotorInput4);



//Reciever signals
  // Serial.print(ReceiverValue[0]);
  // Serial.print(" ");
  // Serial.print(ReceiverValue[1]);
  // Serial.print(" ");
  // Serial.print(ReceiverValue[2]);
  // Serial.print(" ");
  // Serial.print(ReceiverValue[3]);
  // Serial.print(" ");

  // // Print PWM values with labels
  // Serial.print("channel_1_pwm:");
  // Serial.print(channel_1_pwm);
  // Serial.print(" ");

  // Serial.print("channel_2_pwm:");
  // Serial.print(channel_2_pwm);
  // Serial.print(" ");

  // Serial.print("channel_3_pwm:");
  // Serial.print(channel_3_pwm);
  // Serial.print(" ");

  // Serial.print("channel_4_pwm:");
  // Serial.print(channel_4_pwm);
  // Serial.print(" ");

//  // Print Receiver signals with labels
//   Serial.print("ReceiverValue_0:");
//   Serial.print(ReceiverValue[0]);
//   Serial.print(" ");

//   Serial.print("ReceiverValue_1:");
//   Serial.print(ReceiverValue[1]);
//   Serial.print(" ");

//   Serial.print("ReceiverValue_2:");
//   Serial.print(ReceiverValue[2]);
//   Serial.print(" ");

//   Serial.print("ReceiverValue_3:");
//   Serial.print(ReceiverValue[3]); // End the line for Serial Plotter to process
  
//   Serial.print("MotorInput1:");
//   Serial.print(MotorInput1);
//   Serial.print(" ");

//   Serial.print("MotorInput2:");
//   Serial.print(MotorInput2);
//   Serial.print(" ");

//   Serial.print("MotorInput3:");
//   Serial.print(MotorInput3);
//   Serial.print(" ");

//   Serial.print("MotorInput4:");
//   Serial.println(MotorInput4); // End the line for Serial Plotter to process



  // Serial.print(channel_1_pwm);
  //   Serial.print(" ");
  // Serial.print(channel_2_pwm);
  //   Serial.print(" ");
  // Serial.print(channel_3_pwm);
  //   Serial.print(" ");
  // Serial.print(channel_4_pwm);
  //   Serial.print(" ");

//Reciever signals

  // Serial.println(ReceiverValue[0]);
  // Serial.print(" ");
  // Serial.print(ReceiverValue[1]);
  // Serial.print(" ");
  // Serial.print(ReceiverValue[2]);
  // Serial.print(" ");
  // Serial.println(ReceiverValue[3]);
  // Serial.print("  ");
 
  // Serial.print(ReceiverValue[4]);
  // Serial.print(" - ");
  // Serial.print(ReceiverValue[5]);
  // Serial.print(" - ");



//Motor PWMs in us
  // Serial.print("MotVals-");
  // Serial.print(MotorInput1);
  // Serial.print("  ");
  // Serial.print(MotorInput2);
  // Serial.print("  ");
  // Serial.print(MotorInput3);
  // Serial.print("  ");
  // Serial.print(MotorInput4);
  // Serial.print(" ");

// //Reciever translated rates
//   Serial.print(DesiredRateRoll);
//   Serial.print("  ");
//   Serial.print(DesiredRatePitch);
//   Serial.print("  ");
//   Serial.print(DesiredRateYaw);
//   Serial.print(" -- ");

// // //IMU values
  // Serial.print("Acc values: ");
  // Serial.print("AccX:");
  // Serial.print(AccX);
  // Serial.print("  ");
  // Serial.print("AccY:");
  // Serial.print(AccY);
  // Serial.print("  ");
  // Serial.print("AccZ:");
  // Serial.print(AccZ);
  // Serial.print(" -- ");
  // Print the gyroscope values
  // Serial.print("Gyro values: ");
  // Serial.print(RateRoll);
  // Serial.print("  ");
  // Serial.print(RatePitch);
  // Serial.print("  ");
  // Serial.print(RateYaw);
  // Serial.print(" -- ");



//PID outputs
// Serial.print("PID O/P ");
// Serial.print(InputPitch);
//   Serial.print("  ");
// Serial.print(InputRoll);
//   Serial.print("  ");
// Serial.print(InputYaw);
//   Serial.print(" -- ");

//Angles from MPU
  // Serial.print("AngleRoll:");
  // Serial.print(AngleRoll);
  // //serial.print("  ");
  //   Serial.print("AnglePitch:");
  // Serial.print(AnglePitch);

  // // Serial.print("KalmanAngleRoll:");
  // // Serial.print(KalmanAngleRoll);
  // // //serial.print("  ");
  // //   Serial.print("KalmanAnglePitch:");
  // // Serial.print(KalmanAnglePitch);


  
  //  Serial.println(" ");



  while (micros() - LoopTimer < (t*1000000));
  {
     LoopTimer = micros();

  }

}
