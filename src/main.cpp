#include <Arduino.h>
#include <Wire.h>
#include <BasicLinearAlgebra.h>
using namespace BLA;
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <Preferences.h>

// Thông tin WiFi
const char* ssid = "Redmi Note 13"; 
const char* password = "ronaldosiu";
Preferences preferences;

#define MPU6050_I2C_ADDR         0x68  
#define MPU6050_REG_ACCEL_XOUT_H 0x3B  
#define MPU6050_REG_GYRO_XOUT_H  0x43  

#define BMP280_I2C_ADDR          0x76  
#define BMP280_REG_PRESS_MSB     0xF7  

volatile float RateRoll, RatePitch, RateYaw;   //Tốc độ góc quay
volatile float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw; //biến hiệu chỉnh tốc độ góc quay
int RateCalibrationNumber;            //Biến đếm số lần hiệu chỉnh

uint32_t LoopTimer;  
float t=0.004;

//PID of Rotation Rate                    
volatile float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
volatile float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
volatile float InputRoll, InputPitch, InputYaw;
volatile float InputThrottle;
volatile float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
volatile float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
volatile float PIDReturn[] = {0,0,0}; //PID contains 3 variables(at here) and 3 constants(P,I,D)

//pid of 8045 pin 3s
float PRateRoll=1.848;          float PRatePitch=PRateRoll;     float PRateYaw=0.15;
float IRateRoll=0.3;    float IRatePitch=IRateRoll;     float IRateYaw=0.0002;
float DRateRoll=0.01;   float DRatePitch=DRateRoll;     float DRateYaw=0.008;

float PAngleRoll=2;       float PAnglePitch=PRateRoll;
float IAngleRoll=0;       float IAnglePitch=IRateRoll;
float DAngleRoll=0;       float DAnglePitch=DRateRoll;

float complementaryAngleRoll = 0.0f;
float complementaryAnglePitch = 0.0f;
volatile float MotorInput1, MotorInput2, MotorInput3, MotorInput4; 

//PID of Angle
volatile float DesiredAngleRoll, DesiredAnglePitch;
volatile float ErrorAngleRoll, ErrorAnglePitch; 
volatile float PrevErrorAngleRoll, PrevErrorAnglePitch;
volatile float PrevItermAngleRoll, PrevItermAnglePitch;

int ThrottleIdle=1070;
int ThrottleCutOff=1000;

//Kalman Filter for angle mode
float AccX, AccY, AccZ;               
float AngleRoll, AnglePitch;          
volatile float KalmanAngleRoll=0,              
               KalmanUncertaintyAngleRoll=2*2; 
volatile float KalmanAnglePitch=0, 
               KalmanUncertaintyAnglePitch=2*2;
volatile float Kalman1DOutput[]={0,0};      

volatile int ReceiverValue[6]; 

//Các chân nhận tín hiệu RC
const int channel_1_pin = 34; //Roll
const int channel_2_pin = 35; //Pitch
const int channel_3_pin = 32; //Throttle
const int channel_4_pin = 33; //Yaw
const int channel_5_pin = 25;
const int channel_6_pin = 26;

//Các chân ESC
const int esc_pin1 = 27;
const int esc_pin2 = 14;
const int esc_pin3 = 13;
const int esc_pin4 = 12;

//PID of Vertical Velocity 
float AccZInertial; // suy ra VelocityVertical
float VelocityVertical;
float DesiredVelocityVertical;
float ErrorVelocityVertical;
float PrevErrorVelocityVertical;
float PrevItermVelocityVertical;
float InputVelocityVertical;    
float PVelocityVertical=3.5; 
float IVelocityVertical=0.0015; 
float DVelocityVertical=0.01;

uint16_t dig_T1, dig_P1;
int16_t dig_T2, dig_T3;
int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
float AltitudeBarometer, AltitudeBarometerStartUp;
float AltitudeKalman, VelocityVerticalKalman; 
BLA::Matrix<2,2>F;BLA::Matrix<2,1>G;
BLA::Matrix<2,2>P;BLA::Matrix<2,2>Q;
BLA::Matrix<2,1>S;BLA::Matrix<1,2 >H;
BLA::Matrix<2,2>I;BLA::Matrix<1,1>Acc;
BLA::Matrix<2,1>K;BLA::Matrix<1,1>R;
BLA::Matrix<1,1>L;BLA::Matrix<1,1>M;

// Ngắt cho channel 1
void IRAM_ATTR channel1_interrupt() {
  static uint32_t last_rising = 0;
  uint32_t now = micros();
  if (digitalRead(channel_1_pin) == HIGH) {
    last_rising = now;
  } else {
    ReceiverValue[0] = now - last_rising;
  }
}

// Ngắt cho channel 2
void IRAM_ATTR channel2_interrupt() {
  static uint32_t last_rising = 0;
  uint32_t now = micros();
  if (digitalRead(channel_2_pin) == HIGH) {
    last_rising = now;
  } else {
    ReceiverValue[1] = now - last_rising;
  }
}

// Ngắt cho channel 3
void IRAM_ATTR channel3_interrupt() {
  static uint32_t last_rising = 0;
  uint32_t now = micros();
  if (digitalRead(channel_3_pin) == HIGH) {
    last_rising = now;
  } else {
    ReceiverValue[2] = now - last_rising;
  }
}

// Ngắt cho channel 4
void IRAM_ATTR channel4_interrupt() {
  static uint32_t last_rising = 0;
  uint32_t now = micros();
  if (digitalRead(channel_4_pin) == HIGH) {
    last_rising = now;
  } else {
    ReceiverValue[3] = now - last_rising;
  }
}

// Ngắt cho channel 5
void IRAM_ATTR channel5_interrupt() {
  static uint32_t last_rising = 0;
  uint32_t now = micros();
  if (digitalRead(channel_5_pin) == HIGH) {
    last_rising = now;
  } else {
    ReceiverValue[4] = now - last_rising;
  }
}

// Ngắt cho channel 6
void IRAM_ATTR channel6_interrupt() {
  static uint32_t last_rising = 0;
  uint32_t now = micros();
  if (digitalRead(channel_6_pin) == HIGH) {
    last_rising = now;
  } else {
    ReceiverValue[5] = now - last_rising;
  }
}

void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement){
  KalmanState=KalmanState+(t*KalmanInput);                            //1) Predict the current state of the system
  KalmanUncertainty=KalmanUncertainty+(t*t*4*4);                      //2) Calculate the Uncertainty of the prediction
  float KalmanGain=KalmanUncertainty/(KalmanUncertainty+3*3);         //3) Calculate the Kalman gain from the Uncertainties on the p  redictions and measurements
  KalmanState=KalmanState+KalmanGain*(KalmanMeasurement-KalmanState);   //4) Update the predicted state of the system with the measurement of the state through Kalman gain
  KalmanUncertainty=(1-KalmanGain)*KalmanUncertainty;                 //5) Update the Uncertainty of the predicted state
  Kalman1DOutput[0]=KalmanState;
  Kalman1DOutput[1]=KalmanUncertainty;
}

void kalman_2d(){
  Acc={AccZInertial};
  S=F*S+G*Acc;
  P=F*P*~F+Q;
  L=H*P*~H+R;
  K=P*~H*(1.0f/(float)L(0,0));  // ép kiểu rõ ràng về float
  M={AltitudeBarometer};
  S=S+K*(M-H*S);
  AltitudeKalman=S(0,0);
  VelocityVerticalKalman=S(1,0);
  P=(I-K*H)*P;
}

void gyro_signals(void) {
  Wire.beginTransmission(MPU6050_I2C_ADDR);
  Wire.write(MPU6050_REG_ACCEL_XOUT_H);     
  Wire.endTransmission();
  
  Wire.requestFrom(MPU6050_I2C_ADDR, 6);  
  int16_t AccXLSB = Wire.read() << 8 | Wire.read(); // Đọc AccX
  int16_t AccYLSB = Wire.read() << 8 | Wire.read(); // Đọc AccY
  int16_t AccZLSB = Wire.read() << 8 | Wire.read(); // Đọc AccZ
  
  Wire.beginTransmission(MPU6050_I2C_ADDR);
  Wire.write(MPU6050_REG_GYRO_XOUT_H);             
  Wire.endTransmission();
  
  Wire.requestFrom(MPU6050_I2C_ADDR, 6);  
  int16_t GyroX = Wire.read() << 8 | Wire.read(); // Đọc Gyro X
  int16_t GyroY = Wire.read() << 8 | Wire.read(); // Đọc Gyro Y
  int16_t GyroZ = Wire.read() << 8 | Wire.read(); // Đọc Gyro Z

  RateRoll = (float)GyroX / 65.5; 
  RatePitch = (float)GyroY / 65.5;
  RateYaw = (float)GyroZ / 65.5;
  AccX = (float)AccXLSB/4096;
  AccY = (float)AccYLSB/4096;
  AccZ = (float)AccZLSB/4096-0.22;

  AngleRoll = atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*57.296; 
  AnglePitch = -atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*57.296;
}

void barometer_signals(){
  Wire.beginTransmission(BMP280_I2C_ADDR);
  Wire.write(BMP280_REG_PRESS_MSB);
  Wire.endTransmission();
  Wire.requestFrom(BMP280_I2C_ADDR,6);
  uint32_t press_msb = Wire.read();   //0xF7
  uint32_t press_lsb = Wire.read();   //0xF8
  uint32_t press_xlsb = Wire.read();  //0xF9
  uint32_t temp_msb = Wire.read();    //0xFA
  uint32_t temp_lsb = Wire.read();    //0xFB
  uint32_t temp_xlsb = Wire.read();   //0xFC
  unsigned long int adc_P = (press_msb<<12)|(press_lsb<<4)|(press_xlsb>>4); 
  unsigned long int adc_T = (temp_msb<<12)|(temp_lsb<<4)|(temp_xlsb>>4);
  signed long int var1, var2;
  var1 = ((((adc_T >> 3) - ((signed long int)dig_T1 <<1)))* ((signed long int)dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((signed long int)dig_T1 )) * ((adc_T>>4) - ((signed long int)dig_T1))) >> 12) * ((signed long int)dig_T3)) >> 14;
  signed long int t_fine = var1 + var2;
  unsigned long int p;
  var1 = (((signed long int)t_fine) >> 1) - (signed long int)64000;
  var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((signed long int)dig_P6);
  var2 = var2 + ((var1 * ((signed long int)dig_P5)) << 1);
  var2 = (var2 >> 2) + (((signed long int)dig_P4) << 16);
  var1 = (((dig_P3 * (((var1>>2)*(var1>>2)) >> 13)) >> 3) + (((signed long int )dig_P2 * var1)>>1))>>18;
  var1 = (((32768+var1))*((signed long int )dig_P1)) >> 15;
  if (var1 == 0) { p=0;}
  p = (((unsigned long int)(((signed long int )1048576)-adc_P)-(var2>>12)))*3125;
  if(p<0x80000000){ p = (p << 1) / ((unsigned long int) var1);}
  else { p = (p / (unsigned long int)var1) * 2; }
  var1 = (((signed long int )dig_P9) * ((signed long int )((p>>3) * (p>>3))>>13))>>12;
  var2 = (((signed long int )dig_P8) * ((signed long int )var1))>>13;
  p = (unsigned long int)(((signed long int )p + ((var1 + var2+ dig_P7) >> 4)));
  
  double pressure=(double)p/100;
  AltitudeBarometer=44330*(1-pow(pressure/1013.25, 1/5.255))*100;
}

void neutralPositionAdjustment() {
  int min = 1490;
  int max = 1510;
  if (ReceiverValue[0] < max && ReceiverValue[0] > min)
  {
    ReceiverValue[0]= 1500;
  } 
  if (ReceiverValue[1] < max && ReceiverValue[1] > min)
  {
    ReceiverValue[1]= 1500;
  } 
  if (ReceiverValue[3] < max && ReceiverValue[3] > min)
  {
    ReceiverValue[3]= 1500;
  } 
  if(ReceiverValue[0]==ReceiverValue[1] && ReceiverValue[1]==ReceiverValue[3] && ReceiverValue[3]==ReceiverValue[0] )
  {
    ReceiverValue[0]= 1500;
    ReceiverValue[1]= 1500;
    ReceiverValue[3]= 1500;
  }
}

void pid_equation(float Error,float P,float I,float D,float PrevError,float PrevIterm){
  float Pterm = P*Error;
  float Iterm = PrevIterm+(I*(Error+PrevError)*t)/2;
  if(Iterm > 400) Iterm=400;   
  else if(Iterm < -400) Iterm=-400; 
  float Dterm = D*(Error-PrevError)/t;
  float PIDOutput = Pterm + Iterm + Dterm;
  if(PIDOutput > 400) PIDOutput=400;
  else if(PIDOutput < -400) PIDOutput=-400;
  
  PIDReturn[0]=PIDOutput;
  PIDReturn[1]=Error;
  PIDReturn[2]=Iterm;
}

void reset_pid(){
  PrevErrorRateRoll=0; PrevErrorRatePitch=0; PrevErrorRateYaw=0;
  PrevItermRateRoll=0; PrevItermRatePitch=0; PrevItermRateYaw=0;
  PrevErrorAngleRoll=0; PrevErrorAnglePitch=0;
  PrevItermAngleRoll=0; PrevItermAnglePitch=0;
  PrevErrorVelocityVertical=0; 
  PrevItermVelocityVertical=0;
}

AsyncWebServer server(80);

const char* PARAM_P_GAIN = "pGain";   
const char* PARAM_I_GAIN = "iGain";
const char* PARAM_D_GAIN = "dGain";

const char* PARAM_P_A_GAIN = "pAGain";   
const char* PARAM_I_A_GAIN = "iAGain";
const char* PARAM_D_A_GAIN = "dAGain";

const char* PARAM_P_YAW = "pYaw";     
const char* PARAM_I_YAW = "iYaw";
const char* PARAM_D_YAW = "dYaw";

const char* PARAM_TIME_CYCLE = "tc";  

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

String processor(const String& var){
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
  return String(); 
}

void setup() {
  Serial.begin(115200);    
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
  
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html, processor);
  });

  server.on("/get", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputMessage;
    if (request->hasParam(PARAM_P_GAIN)) {
      inputMessage = request->getParam(PARAM_P_GAIN)->value();
      writeFile(SPIFFS, "/pGain.txt", inputMessage.c_str());
    }
    else if (request->hasParam(PARAM_I_GAIN)) {
      inputMessage = request->getParam(PARAM_I_GAIN)->value();
      writeFile(SPIFFS, "/iGain.txt", inputMessage.c_str());
    }
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
    } 
    else if (request->hasParam(PARAM_I_YAW)) {
      inputMessage = request->getParam(PARAM_I_YAW)->value();
      writeFile(SPIFFS, "/iYaw.txt", inputMessage.c_str());
    } 
    else if (request->hasParam(PARAM_D_YAW)) {
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
    // Sửa thành "text/plain" chuẩn cho string response
    request->send(200, "text/plain", inputMessage);
  });

  server.onNotFound(notFound);
  server.begin();

  pinMode(2, OUTPUT);      
  digitalWrite(2, HIGH);   
  pinMode(15, OUTPUT);
  digitalWrite(15, HIGH); 

  pinMode(27, OUTPUT);
  pinMode(14, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(channel_1_pin, INPUT);
  pinMode(channel_2_pin, INPUT);
  pinMode(channel_3_pin, INPUT);
  pinMode(channel_4_pin, INPUT);
  pinMode(channel_5_pin, INPUT);
  pinMode(channel_6_pin, INPUT);

  ledcSetup(0, 250, 12);       
  ledcAttachPin(esc_pin1, 0);
  ledcSetup(1, 250, 12);
  ledcAttachPin(esc_pin2, 1);
  ledcSetup(2, 250, 12);
  ledcAttachPin(esc_pin3, 2);
  ledcSetup(3, 250, 12);
  ledcAttachPin(esc_pin4, 3);

  ledcWrite(0, 1024);
  ledcWrite(1, 1024);
  ledcWrite(2, 1024);
  ledcWrite(3, 1024);

  attachInterrupt(digitalPinToInterrupt(channel_1_pin), channel1_interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channel_2_pin), channel2_interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channel_3_pin), channel3_interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channel_4_pin), channel4_interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channel_5_pin), channel5_interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channel_6_pin), channel6_interrupt, CHANGE);
  delay(100);

  Wire.setClock(400000);    
  Wire.begin();             
  delay(250);               
  Wire.beginTransmission(0x68); 
  Wire.write(0x6B);      
  Wire.write(0x00);      
  Wire.endTransmission(); 
  Wire.beginTransmission(0x68); 
  Wire.write(0x1A);            
  Wire.write(0x05);        
  Wire.endTransmission();      
  
  Wire.beginTransmission(0x68); 
  Wire.write(0x1C);               
  Wire.write(0x10);               
  Wire.endTransmission();
  
  Wire.beginTransmission(0x68); 
  Wire.write(0x1B);            
  Wire.write(0x08);            
  Wire.endTransmission();       

  RateCalibrationRoll = 4.35; 
  RateCalibrationPitch = 0.78;
  RateCalibrationYaw = -0.8;
  
  Wire.beginTransmission(0x76); 
  Wire.write(0xF4);      
  Wire.write(0x57);      
  Wire.endTransmission(); 
  Wire.beginTransmission(0x76); 
  Wire.write(0xF5);      
  Wire.write(0x14);      
  Wire.endTransmission(); 
  
  uint8_t data[24], i=0;
  Wire.beginTransmission(0x76);
  Wire.write(0x88);
  Wire.endTransmission();
  Wire.requestFrom(0x76,24);
  while(Wire.available()){
      data[i] = Wire.read();
      i++;
  }
  dig_T1 = (data[1] << 8) | data[0];
  dig_T2 = (data[3] << 8) | data[2];
  dig_T3 = (data[5] << 8) | data[4];
  dig_P1 = (data[7] << 8) | data[6];
  dig_P2 = (data[9] << 8) | data[8];
  dig_P3 = (data[11] << 8) | data[10];
  dig_P4 = (data[13] << 8) | data[12];
  dig_P5 = (data[15] << 8) | data[14];
  dig_P6 = (data[17] << 8) | data[16];
  dig_P7 = (data[19] << 8) | data[18];
  dig_P8 = (data[21] << 8) | data[20];
  dig_P9 = (data[23] << 8) | data[22]; 
  delay(250);
  
  for (RateCalibrationNumber=0; RateCalibrationNumber<2000; RateCalibrationNumber++) {
    barometer_signals();
    AltitudeBarometerStartUp += AltitudeBarometer;
    delay(1);
  }
  AltitudeBarometerStartUp/=2000;
  
  F={1,t,0,1};
  G={0.5f*t*t,t};
  H={1,0};
  I={1,0,0,1};
  Q={G*~G*15.0f*15.0f};
  R={10*10};
  P={0,0,0,0};
  S={0,0};
  
  while (ReceiverValue[2] < 1020 || ReceiverValue[2] > 1050) {
    delay(10); 
    Serial.print("-"); 
  }
  LoopTimer = micros();
}

void loop() {
  if(ReceiverValue[4] > 1500 ){ 
    PRateRoll = readFile(SPIFFS, "/pGain.txt").toFloat();
    IRateRoll = readFile(SPIFFS, "/iGain.txt").toFloat();
    DRateRoll = readFile(SPIFFS, "/dGain.txt").toFloat();

    PRatePitch = PRateRoll;
    IRatePitch = IRateRoll;
    DRatePitch = DRateRoll;

    PAngleRoll=readFile(SPIFFS, "/pAGain.txt").toFloat();
    IAngleRoll=readFile(SPIFFS, "/iAGain.txt").toFloat();
    DAngleRoll=readFile(SPIFFS, "/dAGain.txt").toFloat();

    PAnglePitch=PAngleRoll;
    IAnglePitch=IAngleRoll;
    DAnglePitch=DAngleRoll;

    PRateYaw = readFile(SPIFFS, "/pYaw.txt").toFloat();
    IRateYaw = readFile(SPIFFS, "/iYaw.txt").toFloat();
    DRateYaw = readFile(SPIFFS, "/dYaw.txt").toFloat();

    t = readFile(SPIFFS, "/tc.txt").toFloat();
  }
  
  int min = 1490;
  int max = 1510;
  if (ReceiverValue[0] < max && ReceiverValue[0] > min) {
    ReceiverValue[0]= 1500;
  } 
  if (ReceiverValue[1] < max && ReceiverValue[1] > min) {
    ReceiverValue[1]= 1500;
  } 
  if (ReceiverValue[3] < max && ReceiverValue[3] > min) {
    ReceiverValue[3]= 1500;
  } 
  if(ReceiverValue[0]==ReceiverValue[1] && ReceiverValue[1]==ReceiverValue[3] && ReceiverValue[3]==ReceiverValue[0] ) {
    ReceiverValue[0]= 1500;
    ReceiverValue[1]= 1500;
    ReceiverValue[3]= 1500;
  }

  gyro_signals();                    

  RateRoll -= RateCalibrationRoll; 
  RatePitch -= RateCalibrationPitch;
  RateYaw -= RateCalibrationYaw;
 
  complementaryAngleRoll=0.991*(complementaryAngleRoll+RateRoll*t) + 0.009*AngleRoll;
  complementaryAnglePitch=0.991*(complementaryAnglePitch+RatePitch*t) + 0.009*AnglePitch;

  AccZInertial=-AccX*sin(AnglePitch*(PI/180))+AccY*sin(AngleRoll*(PI/180))*cos(AnglePitch*(PI/180))+AccZ*cos(AngleRoll*(PI/180))*cos(AnglePitch*(PI/180));
  AccZInertial=(AccZInertial-1)*9.81*100; 
           
  barometer_signals();     
  
  AltitudeBarometer-=AltitudeBarometerStartUp;
  kalman_2d();

  DesiredAngleRoll=0.06*(ReceiverValue[0]-1500);
  DesiredAnglePitch=0.06*(ReceiverValue[1]-1500);
  InputThrottle=ReceiverValue[2]; 
  DesiredRateYaw=0.15*(ReceiverValue[3]-1500);
  
  DesiredAngleRoll = constrain(DesiredAngleRoll, -30, 30);
  DesiredAnglePitch = constrain(DesiredAnglePitch, -30, 30);

  DesiredVelocityVertical=0.3*(ReceiverValue[2]-1500);
  ErrorVelocityVertical = DesiredVelocityVertical - VelocityVerticalKalman;
  pid_equation(ErrorVelocityVertical, PVelocityVertical, IVelocityVertical, DVelocityVertical, PrevErrorVelocityVertical, PrevItermVelocityVertical);
  InputThrottle=1500+PIDReturn[0];
  PrevErrorVelocityVertical=PIDReturn[1];
  PrevItermVelocityVertical=PIDReturn[2];

  ErrorAngleRoll=DesiredAngleRoll-complementaryAngleRoll;
  ErrorAnglePitch=DesiredAnglePitch-complementaryAnglePitch;

  pid_equation(ErrorAngleRoll,PAngleRoll,IAngleRoll,DAngleRoll,PrevErrorAngleRoll,PrevItermAngleRoll);
  DesiredRateRoll=PIDReturn[0];
  PrevErrorAngleRoll=PIDReturn[1];
  PrevItermAngleRoll=PIDReturn[2];

  pid_equation(ErrorAnglePitch,PAnglePitch,IAnglePitch,DAnglePitch,PrevErrorAnglePitch,PrevItermAnglePitch);
  DesiredRatePitch=PIDReturn[0];
  PrevErrorAnglePitch=PIDReturn[1];
  PrevItermAnglePitch=PIDReturn[2];
     
  float PtermPitch = PAnglePitch*ErrorAnglePitch;
  float ItermPitch = PrevItermAnglePitch+(IAnglePitch*(ErrorAnglePitch+PrevErrorAnglePitch)*t)/2;
  if(ItermPitch > 400) ItermPitch=400;       
  else if(ItermPitch < -400) ItermPitch=-400; 
  float DtermPitch = DAnglePitch*(ErrorAnglePitch-PrevErrorAnglePitch)/t;
  float PIDOutputPitch = PtermPitch + ItermPitch + DtermPitch;
  if(PIDOutputPitch > 400) PIDOutputPitch=400;
  else if(PIDOutputPitch < -400) PIDOutputPitch=-400;
  DesiredRatePitch = PIDOutputPitch;
  PrevErrorAnglePitch = ErrorAnglePitch;
  PrevItermAnglePitch = ItermPitch;

  ErrorRateRoll=DesiredRateRoll-RateRoll;
  ErrorRatePitch=DesiredRatePitch-RatePitch;
  ErrorRateYaw=DesiredRateYaw-RateYaw;

  pid_equation(ErrorRateRoll,PRateRoll,IRateRoll,DRateRoll,PrevErrorRateRoll,PrevItermRateRoll); 
  InputRoll=PIDReturn[0];
  PrevErrorRateRoll=PIDReturn[1];
  PrevItermRateRoll=PIDReturn[2];

  pid_equation(ErrorRatePitch,PRatePitch,IRatePitch,DRatePitch,PrevErrorRatePitch,PrevItermRatePitch);
  InputPitch=PIDReturn[0];
  PrevErrorRatePitch=PIDReturn[1];
  PrevItermRatePitch=PIDReturn[2];

  pid_equation(ErrorRateYaw,PRateYaw,IRateYaw,DRateYaw,PrevErrorRateYaw,PrevItermRateYaw);
  InputYaw=PIDReturn[0];
  PrevErrorRateYaw=PIDReturn[1];
  PrevItermRateYaw=PIDReturn[2];

  if(InputThrottle>1800) InputThrottle=1800;

  MotorInput1 = InputThrottle - InputPitch - InputRoll - InputYaw; 
  MotorInput2 = InputThrottle + InputPitch - InputRoll + InputYaw; 
  MotorInput3 = InputThrottle + InputPitch + InputRoll - InputYaw; 
  MotorInput4 = InputThrottle - InputPitch + InputRoll + InputYaw; 

  if(MotorInput1 > 2000) MotorInput1 = 1999;
  if(MotorInput2 > 2000) MotorInput2 = 1999;
  if(MotorInput3 > 2000) MotorInput3 = 1999;
  if(MotorInput4 > 2000) MotorInput4 = 1999;

  if(MotorInput1 < ThrottleIdle && InputThrottle >= 1050) MotorInput1 = ThrottleIdle;
  if(MotorInput2 < ThrottleIdle && InputThrottle >= 1050) MotorInput2 = ThrottleIdle;
  if(MotorInput3 < ThrottleIdle && InputThrottle >= 1050) MotorInput3 = ThrottleIdle;
  if(MotorInput4 < ThrottleIdle && InputThrottle >= 1050) MotorInput4 = ThrottleIdle;

  if(ReceiverValue[2]<1050) {
    MotorInput1=ThrottleCutOff;
    MotorInput2=ThrottleCutOff;
    MotorInput3=ThrottleCutOff;
    MotorInput4=ThrottleCutOff;
    PrevErrorRateRoll=0; PrevErrorRatePitch=0; PrevErrorRateYaw=0;
    PrevItermRateRoll=0; PrevItermRatePitch=0; PrevItermRateYaw=0;
    PrevErrorAngleRoll=0; PrevErrorAnglePitch=0;
    PrevItermAngleRoll=0; PrevItermAnglePitch=0;
    PrevErrorVelocityVertical=0; 
    PrevItermVelocityVertical=0;
  }

  ledcWrite(esc_pin1, map(MotorInput1, 1000, 2000, 1024, 2048));
  ledcWrite(esc_pin2, map(MotorInput2, 1000, 2000, 1024, 2048));
  ledcWrite(esc_pin3, map(MotorInput3, 1000, 2000, 1024, 2048));
  ledcWrite(esc_pin4, map(MotorInput4, 1000, 2000, 1024, 2048));
}