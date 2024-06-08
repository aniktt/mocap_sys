#include <Wire.h>
//#include <ESPAsyncWebServer.h>
//#include <MPU6050.h>
#include <string.h>
#include <WiFi.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Adafruit_NeoPixel.h>
#define TCAADDR1 0x70
#define TCAADDR2 0x71
#define MPUADDR 0x68

#define LED_PIN 2
#define BRIGHTNESS 50

const int buttonPin12 = 12; 
const int buttonPin14 = 14; 

MPU6050 mpus[15];
bool foundIMU[15] = {false};

float ypr[3];
float yproff[15][3];
float a, b, c;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
//#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define BUFFER_SIZE 100

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;

const char *ssid = "Mi 10T Lite";
const char *password = "anika123";
const char *host = "192.168.43.91";
const int port = 12345;
//AsyncWebServer server(80);
//AsyncWebSocket ws("/ws");
bool sendData = false;

WiFiClient client;

Adafruit_NeoPixel pixel(1, LED_PIN, NEO_RGB + NEO_KHZ800);


///   MULTIPLEXER   ///
void tcaselect(uint8_t i) {
  if (i > 15) return;
  Wire.beginTransmission(TCAADDR1);
  if (i > 7) {
    Wire.endTransmission(); 
    delay(10);
    i = i - 8;
    Wire.beginTransmission(TCAADDR2);
  } 
  Wire.write(1 << i);
  Wire.endTransmission();  
  delay(10);
}


///   ADAFRUIT NEOPIXEL   ///
void setColor(uint32_t color) {
 
  pixel.setPixelColor(0, color);         //  Set pixel's color (in RAM)
  pixel.show();                          //  Update strip to match
}

void pulseColor(int r, int g, int b) {
  for  (int blinks = 0; blinks < 3; blinks++) {
    for(int j=0; j<256; j++) { // Ramp up from 0 to 255
      // Fill entire strip with white at gamma-corrected brightness level 'j':
      pixel.fill(pixel.Color(r, g, b, pixel.gamma8(j)));
      pixel.show();
      delay(2);
    }

    for(int j=255; j>=0; j--) { // Ramp down from 255 to 0
      pixel.fill(pixel.Color(0, 0, 0, pixel.gamma8(j)));
      pixel.show();
      delay(2);
    }
  }
}


///   MPU6050   ///
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void ICACHE_RAM_ATTR dmpDataReady() {
    mpuInterrupt = true;
}

const int acel_deadzone = 10;
const int gyro_deadzone = 6;
const int buffersize = 70; 
int16_t ax, ay, az, gx, gy, gz;
int mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz, state = 0;
int ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;
bool getoffset = true;


void mpu_setup()
{
  Wire.setClock(400000);
   getoffset = true;
  // initialize device
    size_t index = 0;
    for (auto &an_mpu: mpus) {
      tcaselect(index);
      //delay(10);
      an_mpu.initialize();
      an_mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_1000);
      an_mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
      devStatus = an_mpu.dmpInitialize();

      an_mpu.setXAccelOffset(0);
      an_mpu.setYAccelOffset(0);
      an_mpu.setZAccelOffset(0);
      an_mpu.setXGyroOffset(0);
      an_mpu.setYGyroOffset(0);
      an_mpu.setZGyroOffset(0);


      //calibration();

      int count = 0;

      Serial.println(index);
       do {
        if (devStatus == 0) {
          // Calibration Time: generate offsets and calibrate our MPU6050
          int numReadings = 100; // Змініть кількість зчитувань тут
          float sumAccX = 0, sumAccY = 0, sumAccZ = 0;
          float sumGyroX = 0, sumGyroY = 0, sumGyroZ = 0;



          for (int a = 0; a < numReadings; a++) {
            
            int16_t ax, ay, az;
            int16_t gx, gy, gz;
            an_mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
            sumAccX += ax;
            sumAccY += ay;
            sumAccZ += az;
            sumGyroX += gx;
            sumGyroY += gy;
            sumGyroZ += gz;

            delay(10); // Затримка між зчитуваннями
          }
          float avgAccX = sumAccX / numReadings;
          float avgAccY = sumAccY / numReadings;
          float avgAccZ = sumAccZ / numReadings;
          float avgGyroX = sumGyroX / numReadings;
          float avgGyroY = sumGyroY / numReadings;
          float avgGyroZ = sumGyroZ / numReadings;

          // Використовуйте середні значення для калібрування
          an_mpu.setXAccelOffset(avgAccX);
          an_mpu.setYAccelOffset(avgAccY);
          an_mpu.setZAccelOffset(avgAccZ);
          an_mpu.setXGyroOffset(avgGyroX);
          an_mpu.setYGyroOffset(avgGyroY);
          an_mpu.setZGyroOffset(avgGyroZ);

          an_mpu.CalibrateAccel(15); 
          an_mpu.CalibrateGyro(15); 

          an_mpu.PrintActiveOffsets();

          an_mpu.setDMPEnabled(true);

          dmpReady = true;

          packetSize = an_mpu.dmpGetFIFOPacketSize();
          Serial.println(F("DMP Initialization completed"));
        }
        else {
          // ERROR!
          // 1 = initial memory load failed
          // 2 = DMP configuration updates failed
          // (if it's going to break, usually the code will be 1)
          Serial.print(F("DMP Initialization failed (code "));
          Serial.print(devStatus);
          Serial.println(F(")"));
        }
        count++;

        if  (count >= 20)  {
          pulseColor (255, 0, 0);
          esp_restart();
        }

       } while (devStatus != 0  );
      ++index;
    }
}

void sendToMaya(String data) {
  if (client.connected()) {
    client.print(data);
    client.println();
    //client.stop();
  }
  else {
    Serial.println("Connection lost. Reconnecting...");
    client.connect(host, port);
    client.print(data);
    client.println();
  }
}

void readCalibrationButton(void *pvParameters);
void readSendButton(void *pvParameters);

void setup() {
  Serial.begin(115200);
  Wire.begin(23, 19);

  pixel.begin();
  pixel.show(); 
  pixel.setBrightness(BRIGHTNESS); 

  pinMode(buttonPin12, INPUT_PULLUP); // Налаштування пінів як вхід з підтяжкою до землі (внутрішній резистор)
  pinMode(buttonPin14, INPUT_PULLUP);

  setColor(pixel.Color(0,   255,   255));
  mpu_setup();
  setColor(pixel.Color(0,   255,   0));

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  Serial.println("Connected to WiFi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  client.connect(host, port);

  xTaskCreatePinnedToCore(readSendButton, "Task1", 2048, NULL, 2, NULL, ARDUINO_RUNNING_CORE);
  xTaskCreatePinnedToCore(readCalibrationButton, "Task2", 2048, NULL, 2, NULL, ARDUINO_RUNNING_CORE);

/*
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/html", "<h1>SSE Example</h1>");
  });*/

  
  //Serial.println("WebSocket Server URL: " + String(ws.url()));
  //server.addHandler(&ws);
  //server.begin();
  
}

void mpu_loop()
{
  if (!dmpReady) return;
  size_t index = 0;
  String angldata;
  for (auto &an_mpu: mpus) {
    tcaselect(index);
    an_mpu.resetFIFO();
    //fifoCount = an_mpu.getFIFOCount();
    memset(fifoBuffer, 0, sizeof(fifoBuffer));
    memset(ypr, 0, sizeof(ypr));
    q.w = 0;
    q.x = 0;
    q.y = 0;
    q.z = 0;
    gravity.x = 0.0f;
    gravity.y = 0.0f;
    gravity.z = 0.0f;
    delay(10);
 
    an_mpu.dmpGetCurrentFIFOPacket(fifoBuffer);

    an_mpu.dmpGetQuaternion(&q, fifoBuffer);
    an_mpu.dmpGetGravity(&gravity, &q);
    an_mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
/*
    Serial.print("ypr\t");
    Serial.print(ypr[0] * 180/M_PI);
    Serial.print(" ");
    Serial.print(ypr[1] * 180/M_PI);
    Serial.print(" ");
    Serial.println(ypr[2] * 180/M_PI);
*/
    a = ypr[0]*180/M_PI;
    b = ypr[1]*180/M_PI;
    c = ypr[2]*180/M_PI;

    while (a  == 180 && b  == 0 && c  == 0 ) {
      an_mpu.resetFIFO();
      //fifoCount = an_mpu.getFIFOCount();
      memset(fifoBuffer, 0, sizeof(fifoBuffer));
      memset(ypr, 0, sizeof(ypr));
      q.w = 0;
      q.x = 0;
      q.y = 0;
      q.z = 0;
      gravity.x = 0.0f;
      gravity.y = 0.0f;
      gravity.z = 0.0f;
      delay(10);
  
      an_mpu.dmpGetCurrentFIFOPacket(fifoBuffer);

      an_mpu.dmpGetQuaternion(&q, fifoBuffer);
      an_mpu.dmpGetGravity(&gravity, &q);
      an_mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      
      a = ypr[0]*180/M_PI;
      b = ypr[1]*180/M_PI;
      c = ypr[2]*180/M_PI;
    }
    if (getoffset==true) {
      yproff[index][0] = a;
      yproff[index][1] = b;
      yproff[index][2] = c;
    }

    if (a  != 180 && b  != 0 && c  != 0 ) {
        angldata.clear(); 
        angldata = String(index) + " " + String( (a - yproff[index][0])) + " " + String((b - yproff[index][1])) + " " + String((c - yproff[index][2]));
        sendToMaya(angldata);
        Serial.println(angldata);
    }
    index++;
    an_mpu.resetFIFO();
  }
  getoffset=false;
  delay(100);
}

void readCalibrationButton(void *pvParameters) // This is a task.
{
    (void)pvParameters;

    while (1) // A Task shall never return or exit.
    {
      int buttonState14 = digitalRead(buttonPin14);

      if (buttonState14 == LOW) { // Коли кнопка натиснута
        sendData = false;
        delay(10);

        setColor(pixel.Color(0,   255,   255));
        mpu_setup();
        setColor(pixel.Color(0,   255,   0));
        delay(10); // Затримка для уникнення спаму в результаті утримання кнопки
      }
    }
}

void readSendButton(void *pvParameters) // This is a task.
{
    (void)pvParameters;

    while (1) // A Task shall never return or exit.
    {
      int buttonState12 = digitalRead(buttonPin12);
      
      if (buttonState12 == LOW) { // Коли кнопка натиснута
        sendData = true;
        setColor(pixel.Color(255,   255,   0));
        delay(5000);
        setColor(pixel.Color(255,   0,   255)); 
        delay(10); // Затримка для уникнення спаму в результаті утримання кнопки
      }
      while (sendData==true) { mpu_loop ();}
    }
}

void loop() {


  //String data = mpu_loop();

  /*
  if (!dmpReady) return;
  size_t index = 0;
  for (auto &an_mpu: mpus) {
    tcaselect(index);
    an_mpu.resetFIFO();
    //fifoCount = an_mpu.getFIFOCount();
    delay(10);
 
    
    an_mpu.dmpGetCurrentFIFOPacket(fifoBuffer);

    an_mpu.dmpGetQuaternion(&q, fifoBuffer);
    an_mpu.dmpGetGravity(&gravity, &q);
    an_mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
          //Serial.print("ypr\t");
          //Serial.print(ypr[0] * 180/M_PI);
          //Serial.print(" ");
          //Serial.print(ypr[1] * 180/M_PI);
          //Serial.print(" ");
          //Serial.println(ypr[2] * 180/M_PI);
    a = ypr[0]*180/M_PI;
    b = ypr[1]*180/M_PI;
    c = ypr[2]*180/M_PI;
    String data = String(index) + " " + String(a) + " " + String(b) + " " + String(c);
        //return  String(index) + " // " + String(a) + " " + String(b) + " " + String(c);
    sendToMaya(data);
      //an_mpu.resetFIFO();
      
    index++;
    an_mpu.resetFIFO();
  }
  //sendToMaya(data);
  //delay(1000);
  //ws.textAll(data);
  /*
  if (client.connect(host, port)) {
    client.print(data);
    client.stop();}
  */
 /*
  server.on("/events", HTTP_GET, [](AsyncWebServerRequest *request){
    AsyncResponseStream *response = request->beginResponseStream("text/event-stream");
    while (true) {
      String data = mpu_loop();
      response->print("data: " + data + "\n\n");
      //delay(1000);  // Adjust delay based on your needs
    }
    request->send(response);
  });*/
  /*
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    String data = mpu_loop();
    request->send(200, "text/plain", data);
  });*/
}


