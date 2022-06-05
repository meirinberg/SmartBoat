/*
  ESP32CAM Robot Car
  esp32cam-robot.ino (requires app_httpd.cpp)
  Based upon Espressif ESP32CAM Examples
  Uses TBA6612FNG H-Bridge Controller
  
  DroneBot Workshop 2021
  https://dronebotworkshop.com
*/

#include "esp_wifi.h"
#include "esp_camera.h"
#include <WiFi.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

#include <ESP32Servo.h>

// GPS
#include "TinyGPS++.h";
#include "HardwareSerial.h";

// Compass
#include <Wire.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_HMC5883_U.h"
#define I2C_SDA 14
#define I2C_SCL 15

#define MOTOR_PIN 2
#define SERVO_PIN 12

#define TX 13
#define RX 16

// Setup Access Point Credentials
const char* ssid1 = "ESP32-CAM Robot";
const char* password1 = "1234567890";

// extern volatile unsigned int  motor_speed;
extern void robot_stop();
extern void robot_setup();
extern uint8_t robo;
extern volatile unsigned long previous_time;        
extern volatile unsigned long move_interval; 

#define CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

volatile extern int speed;
volatile extern int turningAngle;

// GPS
TinyGPSPlus gps;
HardwareSerial SerialGPS(1);
float gpsLat;
float gpsLng;
float gpsAlt;

// Compass
/* Assign a unique ID to this sensor at the same time */
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
//TwoWire I2CSensors = TwoWire(0);
float compassX;
float compassY;
float compassZ;
float compassHeading;

void startCameraServer();

//Servo servoN1;
//Servo servoN2;
//Servo servo1;
//Servo servo2;

void displaySensorDetails(void)
{
  adafruit_sensor_t sensor;
  mag.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void setup() 
{
  robot_setup();


//        servo1.setPeriodHertz(50);    // standard 50 hz servo
//  servo2.setPeriodHertz(50);    // standard 50 hz servo
//  servoN1.attach(2, 1000, 2000);
//  servoN2.attach(13, 1000, 2000);
//  
//  servo1.attach(SERVO_PIN);
//  servo2.attach(MOTOR_PIN);
  ledcSetup(2, 50, 16); //channel, freq, resolution
  ledcAttachPin(SERVO_PIN, 2); // pin, channel
  ledcSetup(4, 50, 16);
  ledcAttachPin(MOTOR_PIN, 4);
    delay(100);


    // Compass
  Serial.begin(9600);
  //I2CSensors.begin(I2C_SDA, I2C_SCL, 100000);
  Serial.println("BeforeWireBegin"); Serial.println("");
  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.println("HMC5883 Magnetometer Test"); Serial.println("");

  /* Initialise the sensor */
  if(!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while(1);
  }
  
  /* Display some basic information on this sensor */
  displaySensorDetails();

  // GPS

//  gpsLat = 0;
//  gpsLng = 0;
//  gpsAlt = 0;
  
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // prevent brownouts by silencing them
  
  SerialGPS.begin(9600, SERIAL_8N1, TX, RX);
  Serial.setDebugOutput(true);
  Serial.println();



  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  //init with high specs to pre-allocate larger buffers
  if(psramFound()){
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  //drop down frame size for higher initial frame rate
  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_QVGA);
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);

  WiFi.softAP(ssid1, password1);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
  
  startCameraServer();

  ledcSetup(7, 5000, 8);
  ledcAttachPin(4, 7);  //pin4 is LED
  
  for (int i=0;i<5;i++) 
  {
    ledcWrite(7,10);  // flash led
    delay(50);
    ledcWrite(7,0);
    delay(50);    
  }
      
  previous_time = millis();
}

void loop() {
  // Compass
  /* Get a new sensor event */ 
  sensors_event_t event; 
  mag.getEvent(&event);
 
  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
  Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  ");Serial.println("uT");

  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  
  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  float declinationAngle = 2.15;
  heading += declinationAngle;
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/M_PI; 
  
  Serial.print("Heading (degrees): "); Serial.println(headingDegrees);

  compassX = event.magnetic.x;
  compassY = event.magnetic.y;
  compassZ = event.magnetic.z;
  compassHeading = headingDegrees;



    // GPS
    while (SerialGPS.available() >0) {
       gps.encode(SerialGPS.read());
    }

    gpsLat = gps.location.lat();
    gpsLng = gps.location.lng();
    gpsAlt = gps.altitude.meters();

    Serial.print("LAT=");  Serial.println(gps.location.lat(), 6);
    Serial.print("LONG="); Serial.println(gps.location.lng(), 6);
    Serial.print("ALT=");  Serial.println(gps.altitude.meters());




//  servo1.write(turningAngle);
//  servo2.write(speed);
 
  ledcWrite(2, map(turningAngle, 0, 180, 0, 255)); // channel, value
  ledcWrite(3, map(speed, 0, 180, 0, 255));
  
  delay(100);
  
//  if(robo)
//  {
//    unsigned long currentMillis = millis();
//    if (currentMillis - previous_time >= move_interval) {
//      previous_time = currentMillis;
//      robot_stop();
//      char rsp[32];
//      sprintf(rsp,"SPPED: %d",motor_speed);
//      Serial.println("Stop");
//      robo=0;
//    }
//  }
//  delay(1);
  yield();
}
