// Import required libraries
#ifdef ESP32
  #include <WiFi.h>
  #include <AsyncTCP.h>
#else
  #include <ESP8266WiFi.h>
  #include <ESPAsyncTCP.h>
#endif
#include <ESPAsyncWebServer.h>
#include <WiFiClient.h>
#include <ArduinoJson.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include "index.h" //configuration and controls HTML webpage contents

#include <EEPROM.h>

#define TIMER_INTERVAL 1000 //Timer interrupy every TIMER_INTERVAL µs
#define FREQ 15000
#define PWM_RANGE 1000  //PWM range
#define PWM_FREQ  10000 //PWM frequency
#define IN1 14
#define IN2 12
#define IN3 13
#define IN4 15
#define SDA1_PN 2 //4 2 SDA must enable high impedance
#define SCL1_PN 0 //5 0 SCL can be changed so GPIO 5 can be used for ENA to avoid the motor running wild during startup.
#define SDA2_PN 4 //4 2 SDA must enable high impedance so cannot be changed
#define SCL2_PN 5 //5 0 SCL can be changed so GPIO 5 can be used for ENA to avoid the motor running wild during startup.
#define I2C_SPEED 400000UL // 100000UL, 400000UL

//#include <Servo.h>
// Replace with your network credentials
//const char* ssid = "ssid";
//const char* password = "password";

#include "C:\Users\Johan\OneDrive\Documenten\Projects\Credentials\credentials.h"

// Set your Static IP address
IPAddress local_IP(192, 168, 2, 60);
// Set your Gateway IP address
IPAddress gateway(192, 168, 2, 1);

IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(8, 8, 8, 8);   //optional
IPAddress secondaryDNS(8, 8, 4, 4); //optional

//For use as access point on 192.168.4.1
//SSID and Password of your WiFi router
//const char* ssid = "ESP8266SBR";
//const char* password = "Algorithm";


AsyncWebServer server(80); //Server on port 80
AsyncWebSocket ws("/ws");
const char* PARAM_INPUT_1 = "state"; 




volatile bool OTAstate = false;
volatile bool remote = false;
volatile bool distanceDataReady = false;
bool robotDirection = false;
bool servoDirection = false;
bool started = false;
bool turnStarted = false;
bool stopRobot = false;
volatile bool startEndReading = false;
double readingPos = 0.00;
double value = 0.00;
double output = 0.00;
double speedRobot = 0.00;
int i = 0;
int servoTrigger = 0;
int positionR = 0;
int wheelDir, servoPos = 0;
int kpAddr = 0, kiAddr = 1, kdAddr = 2, kpAddrDec = 3, kiAddrDec = 4, kdAddrDec = 5, ad = 6, add = 7, ads = 8, za = 9, zadec = 10, mml = 11, mmh = 12, mMl = 13, mMh = 14, mao = 15, mbo = 16, tsh = 17, tsl = 18;
int val, val2;
int m1 = D1, m2 = D5, m3 = D7, m4 = D6;
int offset = 0;
//double Setpoint = 0.00, Input = 0.00, Output = 0.00;
//double Kp = 0.00, Ki = 0.00, Kd = 0.00;
//double pTemp = Kp, iTemp = Ki, dTemp = Kd;
double minimum, maximum;
double k = 0.00;
double desiredAngle = 0.00;
int turnSpeed = 0;
int speedTime = 0;
//PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT, P_ON_M);
//Servo myservo;
int trigPin = D0;
int echoPin = D8;
int OTAbutton = D2;
int servo = RX; // or D9 for node mcu
int led = TX;
int turnTime = 0;
int sampleTime = 15;
double distance = 20.00;
volatile unsigned long start;
volatile unsigned long finish;
unsigned long speedTimer;
unsigned long turnTimer;
unsigned long sr04Timer;
double adjustAngle = 0.00;
double zeroAngle = 0.00;
double motorMinVoltage = 0.00;
double motorMaxVoltage = 0.00;
double pitch = 0.00;
int offsetMotorA = 0;
int offsetMotorB = 0;
byte eepromVal = 0;
// MPU control/status vars
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
uint8_t mpuIntStatus;
// orientation/motion vars
//Quaternion q;           // [w, x, y, z]         quaternion container
//VectorFloat gravity;    // [x, y, z]            gravity vector

long Position_left, Position_right, pwm_left, pwm_right, pwm;

#include "WebSocketEvent.h"


/* For I2C */
#include "I2Cdev.h"
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

/* For SSD1306 OLED Display */
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define OLED_RESET -1 //jma: No Reset Pin used.
Adafruit_SSD1306 display(OLED_RESET);
#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

/* For AS5600 Magnetic Encoder */
#include <AS5600.h>
AMS_5600 ams5600;

#include <PID_v1.h>
//Define Variables we'll be connecting to
double Setpoint_left = 0.0, Input_left = 0.0, Output_left = 0.0;
double Setpoint_right = 0.0, Input_right = 0.0, Output_right = 0.0;
double Speed_left = 0.0, Speed_right = 0.0;
//variable to increment setpoint every xx cycles to test the pid at "constant speed"
int Increment = 1;
int Reduction = 1; //Increment only once every Reduction Cycles
int ReductionCounter = 0;
//Specify the links and initial tuning parameters
double Kp=5, Ki=50, Kd=0.1;
PID myPID_left(&Input_left, &Output_left, &Setpoint_left, Kp, Ki, Kd, P_ON_E, DIRECT);  //P_ON_E = proportional on Error P_ON_M = on Measurement
PID myPID_right(&Input_right, &Output_right, &Setpoint_right, Kp, Ki, Kd, P_ON_E, DIRECT);  //P_ON_E = proportional on Error P_ON_M = on Measurement

//The registers are defined in esp8266_peri.h for the esp8266 in C:\Users\Johan\AppData\Local\Arduino15\packages\esp8266\hardware\esp8266\2.7.4\cores\esp8266
#define timer2_read()       (T2V)
#define timer2_read_alarm() (T2A)  
#define USE_US_TIMER 1
#define os_timer_arm_us(a, b, c) ets_timer_arm_new(a, b, c, 0)
//#include <hw_timer.h>
#include <user_interface.h>

#include <ets_sys.h>
#include <os_type.h>




int IncPwm = 1;

volatile int prev_raw_ang_left = 0, raw_ang_left = 0;
volatile int prev_raw_ang_right = 0, raw_ang_right = 0;
float ang;
int Color = 0;

long Revolutions_left = 0, Revolutions_right = 0;   // number of revolutions the encoder has made

unsigned int count_up, count_down;

unsigned char temp, EncoderState;
bool PIDcycle;

volatile int32_t btime, etime, duration;
int32_t thistime, prevtime;

static inline int32_t asm_ccount(void) {
  int32_t r;
  asm volatile ("rsr %0, ccount" : "=r"(r));
  return r;
}


//jma an other try from Neil Kolban: os_timer_t
//#include "os_type.h"
#include <osapi.h>
os_timer_t myTimer;



int gotSDA, gotSCL;

int TimerCallbackCounter = 0;
long Positions_left[10], Positions_right[10];
#include "TimerCallback.h"

void setup() {
  Serial.begin(74880); // At 74880 baud the boot message is readabel if xtal is 26MHz; with 40MHz xtal it is 115200
  Serial.println();
  Serial.println(">>>>>>>>>>>>>>>>>>>>>>>>>>> ");
//Wifi Stuff

/* //Access point
  WiFi.mode(WIFI_AP);           //Only Access point
  WiFi.setPhyMode(WIFI_PHY_MODE_11N);
  Serial.println("Setting soft-AP ... ");
  boolean result = WiFi.softAP(ssid, password, 14, 0, 2);  //Start HOTspot removing password will disable security
//Access point End */

// Configures static IP address
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("STA Failed to configure");
  }

  // Connect to Wi-Fi network with SSID and password
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  // Print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
//  server.begin();

  Serial.print("Soft-AP IP address = ");
  Serial.println(WiFi.softAPIP());
  // Print ESP Local IP Address
  Serial.print("Local IP Address = ");
  Serial.println(WiFi.localIP());
  Serial.println("Setup start");

  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    Serial.println("/MAIN_page");
    //String s = MAIN_page; //Read HTML contents
    request->send_P(200, "text/html", MAIN_page); //Send web page
  });

  server.on("/getReset", HTTP_GET, [](AsyncWebServerRequest * request) {
    Serial.println("getReset");
    ESP.restart();
  });

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  server.onNotFound(notFound);

  server.begin();

  
  
  Serial.println("setup() end.");





  
  system_timer_reinit();  //Used to set a micro second timer
  os_timer_setfn(&myTimer, timerCallback, NULL);
  //os_timer_arm(&myTimer, 5, 1); //os_timer_arm(*ptimer, miliseconds, repeat_flag)
  os_timer_arm_us(&myTimer, TIMER_INTERVAL, 1); //micro seconds timer. For miliseconds use os_timer_arm(*ptimer, miliseconds, repeat_flag)

  digitalWrite(IN1, HIGH);
  pinMode(IN1, OUTPUT);
  digitalWrite(IN2, HIGH);
  pinMode(IN2, OUTPUT);
  digitalWrite(IN3, HIGH);
  pinMode(IN3, OUTPUT);
  digitalWrite(IN4, HIGH);
  pinMode(IN4, OUTPUT);

  analogWriteFreq(PWM_FREQ);
  analogWriteRange(PWM_RANGE);

  //Wire.begin(SDA_PN, SCL_PN); // default SDA and SCL not available
  //Wire.setClock(I2C_SPEED);
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  //Wire.begin(SDA_PN, SCL_PN);
  Wire.begin(SDA2_PN, SCL2_PN); //Default: D2,D1 = GPIO4,GPIO5 Alt: D4,D3; D3,D4 seems to be unreliable. Writes some times and then gets stuck
  Wire.setClock(450000); //450000 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(450, true);
#endif


#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  //Wire.begin(SDA_PN, SCL_PN);
  Wire.begin(SDA1_PN, SCL1_PN); //Default: D2,D1 = GPIO4,GPIO5 Alt: D4,D3; D3,D4 seems to be unreliable. Writes some times and then gets stuck
//  Wire.setClock(100000); //450000 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire2::setup(450, true);
#endif

/*******************
 * Initialize left wheel positions
********************/
  Wire.begin(SDA1_PN, SCL1_PN); //Default: D2,D1 = GPIO4,GPIO5 Alt: D4,D3;
  Position_left = ams5600.getRawAngle();
  for (int Index = 0; Index <10; Index++){
    Positions_left[Index] = Position_left;
  }
/*******************
 * Initialize right wheel positions
********************/
  Wire.begin(SDA2_PN, SCL2_PN); //Default:D2,D1 = GPIO4,GPIO5 Alt: D4,D3;
  Position_right = 4095 - ams5600.getRawAngle(); //right wheel encoder is reversed: subtract from 4095
  for (int Index = 0; Index <10; Index++){
    Positions_right[Index] = Position_right;
  }

  Wire.begin(SDA1_PN, SCL1_PN); //Default: D2,D1 = GPIO4,GPIO5 Alt: D4,D3;
  //Oled display ssd1306 128x64
  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x64)
  //set orientation of the display 0 = default = yellow bar on top; 1 = 90cw; 2 = 180; 3 = 90ccw
  display.setRotation(0); //2 = yellow bar on the bottom
  // Clear the display buffer.
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE,BLACK); //Set background color to black to overwrite the pixels
  display.setCursor(0,0);
  display.println("AS5600 Encoder");
  display.display(); // Show the display


  Wire.begin(SDA2_PN, SCL2_PN); //Default: D2,D1 = GPIO4,GPIO5 Alt: D4,D3;
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x64)
  //set orientation of the display 0 = default = yellow bar on top; 1 = 90cw; 2 = 180; 3 = 90ccw
  display.setRotation(0); //2 = yellow bar on the bottom
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE,BLACK); //Set background color to black to overwrite the pixels
  display.setCursor(0,0);
  display.println("AS5601 Encoder");

  prevtime = asm_ccount(); //Initialise time

}


  float a = 0.0;
  float offsetrad = 0.0;

// the loop function runs over and over again forever
void loop() {
  //btime = asm_ccount();
  Setpoint_left = 0; //reset setpoint
  delay(10); // Wait xx ms
  distance = speedRobot;
  offsetrad = offset * PI / 180;
//First Display  
  Wire.begin(SDA1_PN, SCL1_PN); //Default: D2,D1 = GPIO4,GPIO5 Alt: D4,D3;
  Wire.getpins(&gotSDA,&gotSCL); 
  display.clearDisplay();
  display.setCursor(0,0);
  display.printf("Left s=% 012ld\r\nv= % 06.3f\r\n", Position_left, Speed_left);
  display.printf("a:% 05.3f o:% 06.4f", a, offsetrad);

  int x = sin(a+offsetrad)*40;
  int y = cos(a+offsetrad)*10;
//  u8g.drawStr( 0, 0, charBuf);
  display.fillCircle(63+x,40,12,WHITE);
  display.fillCircle(63+x,40,3,BLACK);
  display.drawCircle(63,16+100,100,WHITE);
  display.drawCircle(63,63-100,100,WHITE);
  //display.fillCircle(90,40+y,9,WHITE);
  //display.drawCircle(90,40,20,WHITE);
  a +=(3.1415/90);
  if (a>6.283) a=0;
  a = (Position_left%512)*6.283/512;
/*
  //Displaying takes about 30 ms
  display.setTextSize(1);
  //display.setTextSize(2);
  display.setCursor(0,16);
  display.printf("Rev: % 07d",Revolutions);
  display.setCursor(0,24);
  display.printf("Pos: % 07d",Position);
  display.setCursor(0,32);
  display.printf("Setp:% 07d",(long)Setpoint);
  display.setCursor(0,48);
  display.setTextSize(1);
  display.printf("usec: % 07d",duration);
  display.setCursor(0,56);
  display.printf("pid: % 07.2f",Output);
*/

  display.display(); //Refresh display at the end of the loop. this takes about 30 ms.


  //btime = asm_ccount();
//  digitalWrite(LED_BUILTIN, HIGH);  // Turn the LED off by making the voltage HIGH
  //delay(2000); // Wait for two seconds (to demonstrate the active low LED)
//Second Display 
  Wire.begin(SDA2_PN, SCL2_PN); //Default: D2,D1 = GPIO4,GPIO5
  Wire.getpins(&gotSDA,&gotSCL);
  display.clearDisplay();
  display.setCursor(0,0);
  display.printf("Right s=% 012ld\r\nv= % 06.3f\r\n", Position_right, Speed_right);
  //display.printf("pwm: % 04d",pwm);
  display.println(pwm);
/*
  //Displaying takes about 30 ms
  display.setTextSize(1);
  //display.setTextSize(2);
  display.setCursor(0,16);
  display.printf("Rev: % 07d",Revolutions);
  display.setCursor(0,24);
  display.printf("Pos: % 07d",Position);
  display.setCursor(0,32);
  display.printf("Setp:% 07d",(long)Setpoint);
  display.setCursor(0,48);
  display.setTextSize(1);
  display.printf("usec: % 07d",duration);
    display.drawCircle(63,16+100,100,WHITE);
//  display.setCursor(0,56);
//  display.printf("pid: % 07.2f",Output);

  display.setCursor(0,56);
  display.printf("pwm: % 04d",pwm);
*/
///*
  display.fillCircle(63+x,40,12,WHITE);
  display.fillCircle(63+x,40,3,BLACK);
  display.drawCircle(63,16+100,100,WHITE);
  display.drawCircle(63,63-100,100,WHITE);
//*/

  display.display(); //Refresh display at the end of the loop. this takes about 30 ms.

//  display.dimi(Position);
  
  thistime = asm_ccount(); //get time

  //pwm = abs(Position/4096*RANGE);









  //etime = asm_ccount();
  duration = etime - btime;
  if (duration < 0) { duration+=4294967295;}
  duration /=80; //Each tick is 1/80 000 000 s = 12.5 ns /80 to get it in µs
  Serial.print("duration:");
  Serial.print(duration);
  Serial.print(",Setpoint:");
  Serial.print(Setpoint_left);
  Serial.print(",PWM:");
  Serial.println(pwm);  
/*  
  Serial.print("  Timer 1: ");
  Serial.print(timer1_read());
  Serial.print("  Timer 2: ");
  Serial.print(timer2_read());
  Serial.print("  Timer 2 alarm: ");
  Serial.println(timer2_read_alarm());
*/
}
