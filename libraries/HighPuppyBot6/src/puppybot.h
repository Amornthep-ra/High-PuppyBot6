#pragma once
// #include <Servo.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Select the combined ST7735/ST7789 library first.
#include <Adafruit_ST7735.h>
#include <Adafruit_TCS34725.h>
#include <SPI.h>
#include <Wire.h>
#include "hardware/pwm.h"


#define TFT_CS   (13)
#define TFT_DC   (8)
#define TFT_MOSI (11)
#define TFT_SCLK (10)
#define TFT_RST  (9)

Adafruit_ST7735 tft_(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X);
Adafruit_TCS34725 tcsB = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS,TCS34725_GAIN_4X);

int TCS_R = 0, TCS_G = 0, TCS_B = 0, TCS_T = 0, TCS_L = 0;
bool tcsFrontReady = false;
bool tcsBackReady = false;


#include <Adafruit_MCP3008.h>

Adafruit_MCP3008 adc;

#define SERVO_COUNT 6
// Servo servo1;
// Servo servo2;
// Servo servo3;
// Servo servo4;
// Servo servo5;
// Servo servo6;
// #define _servo1 34
// #define _servo2 35
// #define _servo3 36
// #define _servo4 37
// #define _servo5 38
// #define _servo6 39



#define motor1A  21
#define motor1B  22
#define motor2A  23
#define motor2B  24
#define motor3A  25
#define motor3B  26
#define motor4A  28
#define motor4B  29
#define motor5A  30
#define motor5B  31
#define motor6A  32
#define motor6B  33

int PID_NumPin = 3;
int Front_color = 0;
int PID_SetupPin[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int PID_Min[] = {10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10};
int PID_Max[] = {4000,4000,4000,4000,4000,4000,4000,4000,4000,4000,4000,4000,4000,4000,4000,4000};
float errors = 0;
float output = 0;
float integral = 0;
float derivative = 0;
float previous_error = 0;
uint16_t state_on_Line = 0;
uint32_t _lastPosition;
bool first_state_for_calribrate = 0;

int PID_NumPin_B = 3;
int Back_color = 0;
int PID_SetupPin_B[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int PID_Min_B[] = {10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10};
int PID_Max_B[] = {4000,4000,4000,4000,4000,4000,4000,4000,4000,4000,4000,4000,4000,4000,4000,4000};
float errors_B = 0;
float output_B = 0;
float integral_B = 0;
float derivative_B = 0;
float previous_error_B = 0;
uint16_t state_on_Line_B = 0;
uint32_t _lastPosition_B;
bool first_state_for_calribrate_B = 0;

int _Sensitive_F = 80;
int _Sensitive_B = 80;
int current_degree_servo  = 90;

// Legacy v1 pitch/roll globals (used by template.c loop1).
// If not needed, remove these two variables and the loop1 updates.
float pitch_loop1 = 0;
float roll_loop1 = 0;

float yaw_loop1 = 0;
float Con_yaw_loop1 = 0;
float offset_yaw_loop1 = 0;
uint8_t status_resetYaw = 0;
uint8_t status_resetYaw180 = 0;
int selection_Encoder = 1;

unsigned long loop_timer;
int Sensor_mode_F = 0;
int Sensor_mode_B = 0;

int servoPins[SERVO_COUNT] = {34, 35, 36, 37, 38, 39};
// int swpin = 3;
int swpin = 27;

static const uint16_t SERVO_MIN_PULSE_US = 1000;
static const uint16_t SERVO_MAX_PULSE_US = 2000;

uint16_t angleToLevel(int angle) {
  const int safeAngle = constrain(angle, 0, 180);
  const int pulse = map(safeAngle, 0, 180, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
  return (uint16_t)constrain(pulse, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
}

void setupServos() {
  const float divider = ((float)F_CPU) / 1000000.0f; // 1 microsecond per PWM tick
  for (int i = 0; i < SERVO_COUNT; i++) {
    gpio_set_function(servoPins[i], GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(servoPins[i]);
    pwm_set_wrap(slice, 19999);          // 20,000 ticks = 20 ms = 50 Hz
    pwm_set_clkdiv(slice, divider);
    pwm_set_enabled(slice, true);
  }
}

// ฟังก์ชันสั่งมุม: servoRun(ช่องมอเตอร์ 0-5, มุม 0-180)
void servoRun(int channel, int angle) {
  if (channel < 0 || channel >= SERVO_COUNT) return; // ป้องกัน index ผิด
  int pin = servoPins[channel];
  pwm_set_gpio_level(pin, angleToLevel(angle));
}

void softTone(int frequency, int duration) {
  long period = 1000000L / frequency; // คาบเวลา (microseconds)
  long cycles = (long)frequency * duration / 1000; // จำนวนรอบที่ต้องทำ
  
  pinMode(2, OUTPUT);
  
  for (long i = 0; i < cycles; i++) {
    digitalWrite(2, HIGH);
    delayMicroseconds(period / 2);  // ครึ่งคาบ (HIGH)
    digitalWrite(2, LOW);
    delayMicroseconds(period / 2);  // ครึ่งคาบ (LOW)
  }
}


void buzzer(int freq, int timr_delay) {
  // pinMode(7, OUTPUT);
  softTone(freq,timr_delay );
  // delay(timr_delay);
  // tone(7, 0);
  // noTone(7);
}


void puppybot_setup() {
  pinMode(0, OUTPUT);
  pinMode(1, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(40, OUTPUT);
  pinMode(41, OUTPUT);
  pinMode(42, OUTPUT);
  pinMode(43, OUTPUT);
  pinMode(44, OUTPUT);
  pinMode(45, OUTPUT);
  pinMode(46, OUTPUT);
  pinMode(47, OUTPUT);
  pinMode(27, OUTPUT);

  pinMode(34, OUTPUT);
  pinMode(35, OUTPUT);
  pinMode(36, OUTPUT);
  pinMode(37, OUTPUT);
  pinMode(38, OUTPUT);
  pinMode(39, OUTPUT);

  pinMode(15, OUTPUT);
  pinMode(16, OUTPUT);

  adc.begin(10, 11, 12, 14);
  analogWriteResolution(10);
  analogWriteRange(1023);
  Wire.setSDA(4);
  Wire.setSCL(5);
  Wire.begin();
  tft_.initR(INITR_BLACKTAB);
  tft_.setSPISpeed(40000000);
  tft_.setRotation(1);
  tft_.fillScreen(ST77XX_BLACK);
  pinMode(motor1A,OUTPUT);
  pinMode(motor2A,OUTPUT);
  pinMode(motor3A,OUTPUT);
  pinMode(motor4A,OUTPUT);
  pinMode(motor5A,OUTPUT);
  pinMode(motor6A,OUTPUT);
  pinMode(3,INPUT_PULLUP);
  pinMode(27,INPUT_PULLUP);
  tcsFrontReady = tcs.begin();
  if (tcsFrontReady) Serial.println("Found front color sensor");
   buzzer(500,100);
   setupServos();
}
int ADC(int analog_CH) {
   int val = 0;
    if (analog_CH < 8 ) {
      val = adc.readADC(analog_CH);
    }
    else if (analog_CH >= 8 ) {
      val = analogRead(32+analog_CH);
    }  
  return val;
}
int ADC_F(int analog_CH) {
   int val = 0;
   if(Sensor_mode_F == 0){
    if (analog_CH < 8 ) {
      val = adc.readADC(analog_CH);
    }
    else if (analog_CH >= 8 ) {
      val = analogRead(32+analog_CH);
    }  
   }
   else if(Sensor_mode_F == 1){
        // S0=41 S1=42 S2=43 S3=44  Out=40  (CD74HC4067, up to 16ch)
        // reversed mapping: sensor 0 = highest MUX address
        static const int _selF[4] = {41, 42, 43, 44};
        int numCh = (PID_NumPin >= 9) ? 16 : 8;
        int addr = constrain((numCh - 1) - analog_CH, 0, numCh - 1);
        int numBits = (numCh >= 9) ? 4 : 3;
        for(int _b = 0; _b < numBits; _b++){
          gpio_init(_selF[_b]);
          gpio_set_dir(_selF[_b], GPIO_OUT);
          gpio_put(_selF[_b], (addr >> _b) & 1);
        }
        delayMicroseconds(200);
        val = analogRead(40);
   }
  

  return val;
}
int ADC_B(int analog_CH) {
   int val = 0;
   if(Sensor_mode_B == 0){
    if (analog_CH < 8 ) {
      val = adc.readADC(analog_CH);
    }
    else if (analog_CH >= 8 ) {
      val = analogRead(32+analog_CH);
    }  
   }
   else if(Sensor_mode_B == 1){
        // 8-channel MUX: S2=47 S1=46 S0=45  Out=44
        static const int _muxB8[8][3] = {{1,1,1},{1,1,0},{1,0,1},{1,0,0},{0,1,1},{0,1,0},{0,0,1},{0,0,0}};
        int ch = constrain(analog_CH, 0, 7);
        gpio_init(45); gpio_set_dir(45, GPIO_OUT); gpio_put(45, _muxB8[ch][2]);
        gpio_init(46); gpio_set_dir(46, GPIO_OUT); gpio_put(46, _muxB8[ch][1]);
        gpio_init(47); gpio_set_dir(47, GPIO_OUT); gpio_put(47, _muxB8[ch][0]);
        delayMicroseconds(200);
        val = analogRead(44);
   }
   else if(Sensor_mode_B == 2){
        // 16-channel MUX: S0=9 S1=11 S2=13 S3=47  Out=46
        static const int _selB16[4] = {9, 11, 13, 47};
        int addr = constrain(analog_CH, 0, 15);
        for(int _b = 0; _b < 4; _b++){
          gpio_init(_selB16[_b]);
          gpio_set_dir(_selB16[_b], GPIO_OUT);
          gpio_put(_selB16[_b], (addr >> _b) & 1);
        }
        delayMicroseconds(200);
        val = analogRead(46);
   }

  return val;
}

// int ADC(int analog_CH) {
//   int val = 0;
//   if (analog_CH < 8 ) {
//     pinMode(22, OUTPUT);
//     pinMode(23, OUTPUT);
//     pinMode(24, OUTPUT);
//     int muxChannel[8][3] = {{0, 1, 0}, {1, 0, 0}, {0, 0, 0}, {1, 1, 0}, {0, 0, 1}, {0, 1, 1}, {1, 1, 1}, {1, 0, 1}};
//     digitalWrite(22, muxChannel[analog_CH][0]);
//     digitalWrite(23, muxChannel[analog_CH][1]);
//     digitalWrite(24, muxChannel[analog_CH][2]);
//     val = analogRead(26);
//   }
//   else if (analog_CH >= 8 && analog_CH < 11 ) {
//     val = analogRead(19 + analog_CH);
  
//   }
//   return val;
// }

int IN(int _pins) {
  pinMode(_pins, INPUT); 
  return digitalRead(_pins);
}
void OUT(int _pins,uint8_t _Status){
  pinMode(_pins, OUTPUT); 
  digitalWrite(_pins,_Status);
}

void printText(uint8_t x,uint8_t y,String text,uint8_t size,uint16_t  color){
	tft_.setCursor(x, y);
	tft_.setTextSize(size);
  tft_.setTextColor(color);
  tft_.setTextWrap(true);
  tft_.println(text);
}
void printText(uint8_t x,uint8_t y,int text,uint8_t size,uint16_t  color){
  // tft_.setCursor(x, y);
  // tft_.setTextSize(size);
  // tft_.setTextColor(color);
  // tft_.setTextWrap(true);
  // tft_.println(String(text));

}
void printText(uint8_t x,uint8_t y,String text,uint8_t size,uint16_t  color1,uint16_t  color2){
  tft_.setCursor(x, y);
  tft_.setTextSize(size);
  tft_.setTextColor(color1,color2);
  tft_.setTextWrap(true);
  tft_.println(text);
}
void printText(uint8_t x,uint8_t y,long text,uint8_t size,uint16_t  color1,uint16_t  color2){
  tft_.setCursor(x, y);
  tft_.setTextSize(size);
  tft_.setTextColor(color1,color2);
  tft_.setTextWrap(true);
  tft_.println(String(text));
}
void printnumber(uint8_t x,uint8_t y,long text,uint8_t size,uint16_t  color1,uint16_t  color2){
  tft_.setCursor(x, y);
  tft_.setTextSize(size);
  tft_.setTextColor(color1,color2);
  tft_.setTextWrap(true);
  String text_ ;
  if(text <9){
    text_ = String(text) + " "+ " "+ " "; 
  }
  else if(text <99){
    text_ =  String(text) + " "+ " "; 
  }
  else if(text <999){
    text_ =  String(text) + " "; 
  }
  else if(text <9999){
    text_ =  String(text) + " "; 
  }
  else{
   text_ =  String(text) ; 
  }

  //text_ =  String(text) ;
  tft_.println(text_);
}
void printnumber(uint8_t x,uint8_t y,String Input_text,int text,uint8_t size,uint16_t  color1,uint16_t  color2){
  tft_.setCursor(x, y);
  tft_.setTextSize(size);
  tft_.setTextColor(color1,color2);
  tft_.setTextWrap(true);
  String text_ ;
  if(text <9){
    text_ = String(text) + " "+ " "+ " "; 
  }
  else if(text <99){
    text_ =  String(text) + " "+ " "; 
  }
  else if(text <999){
    text_ =  String(text) + " "; 
  }
  tft_.println(Input_text + text_);
}
void printnumber(uint8_t x,uint8_t y,String Input_text,uint8_t size,uint16_t  color1,uint16_t  color2){
  tft_.setCursor(x, y);
  tft_.setTextSize(size);
  tft_.setTextColor(color1,color2);
  tft_.setTextWrap(true);
  // String text_ ;
  // if(text <9){
  //   text_ = String(text) + " "+ " "+ " "; 
  // }
  // else if(text <99){
  //   text_ =  String(text) + " "+ " "; 
  // }
  // else if(text <999){
  //   text_ =  String(text) + " "; 
  // }
  tft_.println(Input_text);
}
void drawString(String text, uint8_t x,uint8_t y) {
  tft_.setCursor(x, y);
  tft_.println(text);
}
void wait_SW1_one() {
  int state_waitSW1 = 0;
  pinMode(swpin, INPUT_PULLUP); 
  tft_.setTextSize(2);
  tft_.fillScreen(ST77XX_BLACK);
  tft_.setTextSize(1);
  tft_.setTextColor(ST77XX_YELLOW, ST77XX_RED);
    drawString("KBIDE SW1 Press  ",30,105);
    tft_.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  do {
    tft_.setTextSize(2);
    for(int i =0;i<=10;i++){
      for(int j =0;j<25;j++){
        if(i %2 == 0){
          printnumber(50,50,String(i)+"=",ADC(i),2,ST77XX_YELLOW,ST77XX_RED);
        }
        else{
          printnumber(50,50,String(i)+"=",ADC(i),2,ST77XX_WHITE,ST77XX_BLACK);
        }
        
        if(digitalRead(swpin) == 0){pinMode(swpin,OUTPUT);break;}
      }
      if(digitalRead(swpin) == 0){pinMode(swpin,OUTPUT);break;}
    }
    
  } while (digitalRead(swpin) == 1);
  pinMode(swpin,OUTPUT);
  buzzer(2000,100);
  
}
void wait_SW1() {
  int state_waitSW1 = 0;
  pinMode(swpin, INPUT_PULLUP);

  tft_.setTextSize(2);
  tft_.fillScreen(ST77XX_BLACK);
  tft_.setTextSize(2);
  tft_.setTextColor(ST77XX_YELLOW, ST77XX_RED);
    drawString("Wait SW1",30,105);
    tft_.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  do {
    tft_.setTextSize(2);
    drawString("0="+String(ADC(0))+"  ",0,0);if(digitalRead(swpin) == 0){pinMode(swpin,OUTPUT);break;}
    drawString("1="+String(ADC(1))+"  ",80,0);if(digitalRead(swpin) == 0){pinMode(swpin,OUTPUT);break;}
    drawString("2="+String(ADC(2))+"  ",0,16);if(digitalRead(swpin) == 0){pinMode(swpin,OUTPUT);break;}
    drawString("3="+String(ADC(3))+"  ",80,16);if(digitalRead(swpin) == 0){pinMode(swpin,OUTPUT);break;}
    drawString("4="+String(ADC(4))+"  ",0,32);if(digitalRead(swpin) == 0){pinMode(swpin,OUTPUT);break;}
    drawString("5="+String(ADC(5))+"  ",80,32);if(digitalRead(swpin) == 0){pinMode(swpin,OUTPUT);break;}
    drawString("6="+String(ADC(6))+"  ",0,48);if(digitalRead(swpin) == 0){pinMode(swpin,OUTPUT);break;}
    drawString("7="+String(ADC(7))+"  ",80,48);if(digitalRead(swpin) == 0){pinMode(swpin,OUTPUT);break;}
    drawString("8="+String(ADC(8))+"  ",0,64);if(digitalRead(swpin) == 0){pinMode(swpin,OUTPUT);break;}
    drawString("9="+String(ADC(9))+"  ",80,64);if(digitalRead(swpin) == 0){pinMode(swpin,OUTPUT);break;}
    drawString("15="+String(ADC(15))+"  ",0,80);if(digitalRead(swpin) == 0){pinMode(swpin,OUTPUT);break;}
    drawString("G="+String((int)Con_yaw_loop1)+"  ",80,80);if(digitalRead(swpin) == 0){pinMode(swpin,OUTPUT);break;}
    
    if(Con_yaw_loop1 > 0){
      tft_.setTextColor(ST77XX_BLUE, ST77XX_GREEN);
      drawString("Wait SW1",30,105);
      tft_.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    }
  } while (digitalRead(swpin) == 1);
  pinMode(swpin,OUTPUT);
  // buzzer(500,100);
  // delay(200);
  // buzzer(700,100);
  // delay(200);
  // buzzer(1000,100);
  // delay(200);
  buzzer(2000,100);
  // pinMode(swpin, OUTPUT);
  
}
void wait_SW1_ALL() {
  int state_waitSW1 = 0;
  pinMode(swpin, INPUT_PULLUP);

  tft_.setTextSize(2);
  tft_.fillScreen(ST77XX_BLACK);
  tft_.setTextSize(2);
  tft_.setTextColor(ST77XX_YELLOW, ST77XX_RED);
    drawString("Wait SW1",30,110);
    tft_.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  do {
    tft_.setTextSize(1);
	    tft_.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
	    drawString("A0="+String(ADC(0))+"  ",0,0);if(digitalRead(swpin) == 0){pinMode(swpin,OUTPUT);break;}
	    drawString("A1="+String(ADC(1))+"  ",50,0);if(digitalRead(swpin) == 0){pinMode(swpin,OUTPUT);break;}
	    drawString("A2="+String(ADC(2))+"  ",100,0);if(digitalRead(swpin) == 0){pinMode(swpin,OUTPUT);break;}
	    drawString("A3="+String(ADC(3))+"  ",0,10);if(digitalRead(swpin) == 0){pinMode(swpin,OUTPUT);break;}
	    drawString("A4="+String(ADC(4))+"  ",50,10);if(digitalRead(swpin) == 0){pinMode(swpin,OUTPUT);break;}
	    drawString("A5="+String(ADC(5))+"  ",100,10);if(digitalRead(swpin) == 0){pinMode(swpin,OUTPUT);break;}
	    drawString("A6="+String(ADC(6))+"  ",0,20);if(digitalRead(swpin) == 0){pinMode(swpin,OUTPUT);break;}
	    drawString("A7="+String(ADC(7))+"  ",50,20);if(digitalRead(swpin) == 0){pinMode(swpin,OUTPUT);break;}
    if(Sensor_mode_F == 1){
    	tft_.setTextColor(ST77XX_ORANGE, ST77XX_BLACK);
    	drawString("F0="+String(ADC_F(0))+"  ",0,35);if(digitalRead(swpin) == 0){pinMode(swpin,OUTPUT);break;}
	    drawString("F1="+String(ADC_F(1))+"  ",50,35);if(digitalRead(swpin) == 0){pinMode(swpin,OUTPUT);break;}
	    drawString("F2="+String(ADC_F(2))+"  ",100,35);if(digitalRead(swpin) == 0){pinMode(swpin,OUTPUT);break;}
	    drawString("F3="+String(ADC_F(3))+"  ",0,45);if(digitalRead(swpin) == 0){pinMode(swpin,OUTPUT);break;}
	    drawString("F4="+String(ADC_F(4))+"  ",50,45);if(digitalRead(swpin) == 0){pinMode(swpin,OUTPUT);break;}
	    drawString("F5="+String(ADC_F(5))+"  ",100,45);if(digitalRead(swpin) == 0){pinMode(swpin,OUTPUT);break;}
	    drawString("F6="+String(ADC_F(6))+"  ",0,55);if(digitalRead(swpin) == 0){pinMode(swpin,OUTPUT);break;}
	    drawString("F7="+String(ADC_F(7))+"  ",50,55);if(digitalRead(swpin) == 0){pinMode(swpin,OUTPUT);break;}
	    // drawString("F8="+String(ADC_F(8))+"  ",100,50);if(digitalRead(swpin) == 0){pinMode(swpin,OUTPUT);break;}
    }
    else {
    	tft_.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    	drawString("A8="+String(ADC(8))+"  ",0,35);if(digitalRead(swpin) == 0){pinMode(swpin,OUTPUT);break;}
	    drawString("A9="+String(ADC(9))+"  ",50,35);if(digitalRead(swpin) == 0){pinMode(swpin,OUTPUT);break;}
	    drawString("A10="+String(ADC(10))+"  ",100,35);if(digitalRead(swpin) == 0){pinMode(swpin,OUTPUT);break;}
	    drawString("A11="+String(ADC(11))+"  ",0,45);if(digitalRead(swpin) == 0){pinMode(swpin,OUTPUT);break;}
	    // drawString("F4="+String(ADC(4))+"  ",50,45);if(digitalRead(swpin) == 0){pinMode(swpin,OUTPUT);break;}
    }
    if(Sensor_mode_B == 0){
      
      drawString("A12="+String(ADC(12))+"  ",0,80);if(digitalRead(swpin) == 0){pinMode(swpin,OUTPUT);break;}
      drawString("A13="+String(ADC(13))+"  ",50,80);if(digitalRead(swpin) == 0){pinMode(swpin,OUTPUT);break;}
      drawString("A14="+String(ADC(14))+"  ",100,80);if(digitalRead(swpin) == 0){pinMode(swpin,OUTPUT);break;}
      drawString("A15="+String(ADC(15))+"  ",0,90);if(digitalRead(swpin) == 0){pinMode(swpin,OUTPUT);break;}
      drawString("G="+String(Con_yaw_loop1)+"  ",100,100);if(digitalRead(swpin) == 0){pinMode(swpin,OUTPUT);break;}
    }
    else if(Sensor_mode_B == 1){
      tft_.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
      drawString("B0="+String(ADC_B(0))+"  ",0,70);if(digitalRead(swpin) == 0){pinMode(swpin,OUTPUT);break;}
      drawString("B1="+String(ADC_B(1))+"  ",50,70);if(digitalRead(swpin) == 0){pinMode(swpin,OUTPUT);break;}
      drawString("B2="+String(ADC_B(2))+"  ",100,70);if(digitalRead(swpin) == 0){pinMode(swpin,OUTPUT);break;}
      drawString("B3="+String(ADC_B(3))+"  ",0,80);if(digitalRead(swpin) == 0){pinMode(swpin,OUTPUT);break;}
      drawString("B4="+String(ADC_B(4))+"  ",50,80);if(digitalRead(swpin) == 0){pinMode(swpin,OUTPUT);break;}
      drawString("B5="+String(ADC_B(5))+"  ",100,80);if(digitalRead(swpin) == 0){pinMode(swpin,OUTPUT);break;}
      drawString("B6="+String(ADC_B(6))+"  ",0,90);if(digitalRead(swpin) == 0){pinMode(swpin,OUTPUT);break;}
      drawString("B7="+String(ADC_B(7))+"  ",50,90);if(digitalRead(swpin) == 0){pinMode(swpin,OUTPUT);break;}
      drawString("G="+String(Con_yaw_loop1)+"  ",100,100);if(digitalRead(swpin) == 0){pinMode(swpin,OUTPUT);break;}
    }
    
    
    tft_.setTextSize(2);
    if(Con_yaw_loop1 > 0){
      tft_.setTextColor(ST77XX_BLUE, ST77XX_GREEN);
      drawString("Wait SW1",30,110);
      tft_.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    }
  } while (digitalRead(swpin) == 1);
  pinMode(swpin,OUTPUT);
  // buzzer(500,100);
  // delay(200);
  // buzzer(700,100);
  // delay(200);
  // buzzer(1000,100);
  // delay(200);
  buzzer(2000,100);

}

void showResetReason() {
  tft_.setRotation(3);
  RP2040::resetReason_t r = rp2040.getResetReason();
  const char* s;
  int waitMs = 10000;
  switch (r) {
    case RP2040::PWRON_RESET:    s = "POWER-ON";  waitMs = 5000; break;
    case RP2040::SOFT_RESET:     s = "SOFTWARE";  waitMs = 5000; break;
    case RP2040::RUN_PIN_RESET:  s = "RUN PIN";   break;
    case RP2040::WDT_RESET:      s = "WATCHDOG";  break;
    case RP2040::DEBUG_RESET:    s = "DEBUG";     break;
    case RP2040::GLITCH_RESET:   s = "GLITCH";    break;
    case RP2040::BROWNOUT_RESET: s = "BROWNOUT";  break;
    default:                     s = "UNKNOWN";   break;
  }
  tft_.fillScreen(ST77XX_BLACK);
  tft_.setTextSize(1);
  tft_.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
  drawString(String("RST: ") + s, 10, 50);
  Serial.print("Reset reason: "); Serial.println(s);
  delay(waitMs);
}

void motor(int pin, int speed_Motor) {
  if (speed_Motor > 100)speed_Motor = 100;
  if (speed_Motor < -100)speed_Motor = -100;
  if (pin == 1) {
    if (speed_Motor < 0) {
      speed_Motor = abs(speed_Motor) * 10.23;
      digitalWrite(motor1A,0);
      analogWrite(motor1B,abs(speed_Motor));
    }
    else {
      speed_Motor = abs(speed_Motor) * 10.23;
      digitalWrite(motor1A,1);
      analogWrite(motor1B,abs(speed_Motor));
    }
  }
  else if (pin == 2) {
    if (speed_Motor < 0) {
      speed_Motor = abs(speed_Motor) * 10.23;
      digitalWrite(motor2A,0);
      analogWrite(motor2B, abs(speed_Motor));
    }
    else {
      speed_Motor = abs(speed_Motor) * 10.23;
      digitalWrite(motor2A,1);
      analogWrite(motor2B, abs(speed_Motor));
    }
  }
  else if (pin == 3) {
    if (speed_Motor < 0) {
      speed_Motor = abs(speed_Motor) * 10.23;
      digitalWrite(motor3A,0);
      analogWrite(motor3B,abs(speed_Motor));
    }
    else {
      speed_Motor = abs(speed_Motor) * 10.23;
      digitalWrite(motor3A,1);
      analogWrite(motor3B, abs(speed_Motor));
    }
  }
  else if (pin == 4) {
    if (speed_Motor < 0) {
      speed_Motor = abs(speed_Motor) * 10.23;
      digitalWrite(motor4A,0);
      analogWrite(motor4B,abs(speed_Motor));
    }
    else {
      speed_Motor = abs(speed_Motor) * 10.23;
      digitalWrite(motor4A,1);
      analogWrite(motor4B,abs(speed_Motor));
    }
  }
  else if (pin == 5) {
    if (speed_Motor < 0) {
      speed_Motor = abs(speed_Motor) * 10.23;
      digitalWrite(motor5A,0);
      analogWrite(motor5B,abs(speed_Motor));
    }
    else {
      speed_Motor = abs(speed_Motor) * 10.23;
      digitalWrite(motor5A,1);
      analogWrite(motor5B,abs(speed_Motor));
    }
  }
  else if (pin == 6) {
    if (speed_Motor < 0) {
      speed_Motor = abs(speed_Motor) * 10.23;
      digitalWrite(motor6A,0);
      analogWrite(motor6B,abs(speed_Motor));
    }
    else {
      speed_Motor = abs(speed_Motor) * 10.23;
      digitalWrite(motor6A,1);
      analogWrite(motor6B,abs(speed_Motor));
    }
  }
  else if (pin == 7) {
    // Keep speed in percent here. Each channel converts percent to 10-bit PWM once.
    for (int motorChannel = 1; motorChannel <= 6; motorChannel++) {
      motor(motorChannel, speed_Motor);
    }
  }
}
void motor_control(uint8_t state , int _speed) {
  switch (state) {
    case 0: {
        motor(1, _speed);
        motor(2, _speed);
        motor(3, _speed);
        motor(4, _speed);
      } break;
    case 1: {
        motor(1,- _speed);
        motor(2,- _speed);
        motor(3,- _speed);
        motor(4,- _speed);
      } break;
    case 2: {
        motor(1, 0);
        motor(2, _speed);
        motor(3, 0);
        motor(4, _speed);
      } break;
    case 3: {
        motor(1, _speed);
        motor(2, 0);
        motor(3, _speed);
        motor(4, 0);
      } break;
    case 4: {
        motor(1, -_speed);
        motor(2, _speed);
        motor(3,-_speed);
        motor(4, _speed);
      } break;
    case 5: {
        motor(1, _speed);
        motor(2, -_speed);
        motor(3, _speed);
        motor(4, -_speed);
      } break;
    case 6: {
        motor(1, -_speed);
        motor(2, _speed);
        motor(3, _speed);
        motor(4, -_speed);
      } break;
    case 7: {
        motor(1, _speed);
        motor(2, -_speed);
        motor(3, -_speed);
        motor(4, _speed);
      } break;
    case 8: {
        motor(1, 0);
        motor(2, _speed);
        motor(3, _speed);
        motor(4, 0);
      } break;
    case 9: {
        motor(1, _speed);
        motor(2, 0);
        motor(3, 0);
        motor(4, _speed);
      } break;
    
  }
}
void ao(){
	analogWrite(motor1B, 0);
    analogWrite(motor2B, 0);
    analogWrite(motor3B, 0);
    analogWrite(motor4B, 0);
    analogWrite(motor5B, 0);
    analogWrite(motor6B, 0);
}
void aoS(int speed_break){
  speed_break = constrain(speed_break, 0, 100);
  speed_break = speed_break * 10.23;
  analogWrite(motor1B, 0);
  analogWrite(motor2B, 0);
  analogWrite(motor3B, 0);
  analogWrite(motor4B, 0);
  analogWrite(motor5B, 0);
  analogWrite(motor6B, 0);
}
void motorStop(int motor_ch){
  	if(motor_ch == 0){
	  analogWrite(motor1B, 0);
    analogWrite(motor2B, 0);
    analogWrite(motor3B, 0);
    analogWrite(motor4B, 0);
    analogWrite(motor5B, 0);
    analogWrite(motor6B, 0);
	}
	else if(motor_ch == 1 ){
	  analogWrite(motor1B, 0);
	  analogWrite(motor1A, 0);
	}
	else if(motor_ch == 2 ){
	  analogWrite(motor2B, 0);
	  analogWrite(motor2A, 0);
	}
	else if(motor_ch == 3 ){
	  analogWrite(motor3B, 0);
	  analogWrite(motor3A, 0);
	}
	else if(motor_ch == 4 ){
	  analogWrite(motor4B, 0);
	  analogWrite(motor4A, 0);
	}
	else{
	  analogWrite(motor1B, 0);
    analogWrite(motor2B, 0);
    analogWrite(motor3B, 0);
    analogWrite(motor4B, 0);
    analogWrite(motor5B, 0);
    analogWrite(motor6B, 0);
	}
}
void motorBreak(){
    analogWrite(motor1B, 0);
    analogWrite(motor2B, 0);
    analogWrite(motor3B, 0);
    analogWrite(motor4B, 0);
    analogWrite(motor5B, 0);
    analogWrite(motor6B, 0);
}
void motorBreak(int motor_ch){
  
	if(motor_ch == 0){
	  analogWrite(motor1B, 0);
	  analogWrite(motor2B, 0);
	  analogWrite(motor3B, 0);
	  analogWrite(motor4B, 0);
    analogWrite(motor5B, 0);
    analogWrite(motor6B, 0);

	}
	else if(motor_ch == 1 ){
	  analogWrite(motor1B, 1023);
	  analogWrite(motor1A, 1023);
	}
	else if(motor_ch == 2 ){
	  analogWrite(motor2B, 1023);
	  analogWrite(motor2A, 1023);
	}
	else if(motor_ch == 3 ){
	  analogWrite(motor3B, 1023);
	  analogWrite(motor3A, 1023);
	}
	else if(motor_ch == 4 ){
	  analogWrite(motor4B, 1023);
	  analogWrite(motor4A, 1023);
	}
	else{
	  analogWrite(motor1B, 1023);
	  analogWrite(motor1A, 1023);
	  analogWrite(motor2B, 1023);
	  analogWrite(motor2A, 1023);
	  analogWrite(motor3B, 1023);
	  analogWrite(motor3A, 1023);
	  analogWrite(motor4B, 1023);
	  analogWrite(motor4A, 1023);
	}
}
void fd(int speed_Motor){
	motor(1,speed_Motor);
	motor(2,speed_Motor);
	motor(3,speed_Motor);
	motor(4,speed_Motor);
}
void fd2(int speed_MotorA,int speed_MotorB){
  motor(1,speed_MotorA);
  motor(2,speed_MotorB);
}
void bk(int speed_Motor){
	motor(1,-speed_Motor);
	motor(2,-speed_Motor);
	motor(3,-speed_Motor);
	motor(4,-speed_Motor);
}
void bk2(int speed_MotorA,int speed_MotorB){
  motor(1,-speed_MotorA);
  motor(2,-speed_MotorB);
}
void tl(int speed_Motor){
	motor(1,0);
	motor(2,speed_Motor);
}
void tr(int speed_Motor){
	motor(1,speed_Motor);
	motor(2,0);
}
void sl(int speed_Motor){
	motor(1,-speed_Motor);
	motor(2,speed_Motor);
}
void sr(int speed_Motor){
	motor(1,speed_Motor);
	motor(2,-speed_Motor);
}

// void servoRun(uint8_t servo_ch, int16_t angle) {

//   if (servo_ch == 1)
//   {
//     servo1.attach(34);
//     servo1.write(angle);
//   }
//   if (servo_ch == 2)
//   {
//     if(angle == -1){servo2.detach();}
//     servo2.attach(_servo2,300,2500,angle);
//   }
//   if (servo_ch == 3)
//   {
//     if(angle == -1){servo3.detach();}
//     servo3.attach(_servo3,300,2500,angle);
//   }
//   if (servo_ch == 4)
//   {
//     if(angle == -1){servo4.detach();}
//     servo4.attach(_servo4,300,2500,angle);
//   }
// }
void control_servo(int servo_ch,int servo_degree,int traget_degree,int servo_speed){
  if(abs(current_degree_servo - traget_degree) > 2){
    if(servo_degree < traget_degree ){
      for(int i = servo_degree;i<traget_degree;i++){
        servoRun(servo_ch,i);
        delay(servo_speed);
        current_degree_servo = i;
      }
    }
    else {
      for(int i = servo_degree;i>traget_degree;i--){
        servoRun(servo_ch,i);
        delay(servo_speed);
        current_degree_servo = i;
      }
    }
  }
}
void servoSweep(int channel, int startAngle, int endAngle, int step, int delayMs) {
  if (channel < 0 || channel >= SERVO_COUNT) return;
  if (startAngle < endAngle) {
    for (int a = startAngle; a <= endAngle; a += step) {
      servoRun(channel, a);
      delay(delayMs);
    }
  } else {
    for (int a = startAngle; a >= endAngle; a -= step) {
      servoRun(channel, a);
      delay(delayMs);
    }
  }
}
float ultrasonic(uint8_t Echo_pin , uint8_t Trig_pin) {
  int ECHO = Echo_pin;
  int TRIG = Trig_pin;
  pinMode(ECHO, INPUT);
  pinMode(TRIG, OUTPUT);
  long duration = 0;
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  duration += pulseIn(ECHO, HIGH);

  float dis_ = (duration) * 0.034 / 2;
  if(dis_ <= 0 ) dis_=99;

  // Calculating the distance
  return dis_;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setSensitive_F(int _SensorSensitive)
{
    _Sensitive_F = _SensorSensitive;
}
void PID_set_Min(int S0,int S1,int S2,int S3,int S4,int S5,int S6,int S7){
  PID_Min[0] = S0;PID_Min[1] = S1;PID_Min[2] = S2;PID_Min[3] = S3;
  PID_Min[4] = S4;PID_Min[5] = S5;PID_Min[6] = S6;PID_Min[7] = S7;
}
void PID_set_Max(int S0,int S1,int S2,int S3,int S4,int S5,int S6,int S7){
  PID_Max[0] = S0;PID_Max[1] = S1;PID_Max[2] = S2;PID_Max[3] = S3;
  PID_Max[4] = S4;PID_Max[5] = S5;PID_Max[6] = S6;PID_Max[7] = S7;
}
void PID_set_Pin(int S0,int S1,int S2,int S3,int S4,int S5,int S6,int S7){
  PID_SetupPin[0] = S0;PID_SetupPin[1] = S1;PID_SetupPin[2] = S2;PID_SetupPin[3] = S3;
  PID_SetupPin[4] = S4;PID_SetupPin[5] = S5;PID_SetupPin[6] = S6;PID_SetupPin[7] = S7;
}
void PID_set_Min16(int S0,int S1,int S2,int S3,int S4,int S5,int S6,int S7,
                   int S8,int S9,int S10,int S11,int S12,int S13,int S14,int S15){
  PID_Min[0]=S0;PID_Min[1]=S1;PID_Min[2]=S2;PID_Min[3]=S3;
  PID_Min[4]=S4;PID_Min[5]=S5;PID_Min[6]=S6;PID_Min[7]=S7;
  PID_Min[8]=S8;PID_Min[9]=S9;PID_Min[10]=S10;PID_Min[11]=S11;
  PID_Min[12]=S12;PID_Min[13]=S13;PID_Min[14]=S14;PID_Min[15]=S15;
}
void PID_set_Max16(int S0,int S1,int S2,int S3,int S4,int S5,int S6,int S7,
                   int S8,int S9,int S10,int S11,int S12,int S13,int S14,int S15){
  PID_Max[0]=S0;PID_Max[1]=S1;PID_Max[2]=S2;PID_Max[3]=S3;
  PID_Max[4]=S4;PID_Max[5]=S5;PID_Max[6]=S6;PID_Max[7]=S7;
  PID_Max[8]=S8;PID_Max[9]=S9;PID_Max[10]=S10;PID_Max[11]=S11;
  PID_Max[12]=S12;PID_Max[13]=S13;PID_Max[14]=S14;PID_Max[15]=S15;
}
void PID_set_Pin16(int S0,int S1,int S2,int S3,int S4,int S5,int S6,int S7,
                   int S8,int S9,int S10,int S11,int S12,int S13,int S14,int S15){
  PID_SetupPin[0]=S0;PID_SetupPin[1]=S1;PID_SetupPin[2]=S2;PID_SetupPin[3]=S3;
  PID_SetupPin[4]=S4;PID_SetupPin[5]=S5;PID_SetupPin[6]=S6;PID_SetupPin[7]=S7;
  PID_SetupPin[8]=S8;PID_SetupPin[9]=S9;PID_SetupPin[10]=S10;PID_SetupPin[11]=S11;
  PID_SetupPin[12]=S12;PID_SetupPin[13]=S13;PID_SetupPin[14]=S14;PID_SetupPin[15]=S15;
}
void setCalibrate(int round){
  buzzer(500,100);
  tft_.fillScreen(ST77XX_BLACK);
  printText(0,0,"Calibrate Front Sensor",3,ST77XX_YELLOW,ST77XX_RED);
  // display.clear();
  // display.setFont(ArialMT_Plain_24);
  // display.drawString(0,0,"Front Sensor");
  // display.drawString(0,25,"  Calribrate  ");
  // display.display();
  if(first_state_for_calribrate == 0){
    for (uint8_t i = 0; i < PID_NumPin; i++)
    {
      PID_Max[i] = 0;
      PID_Min[i] = 1023;
    }
    first_state_for_calribrate = 1;
  }
  for(int roundOfCalribtate = 0; roundOfCalribtate < round ;roundOfCalribtate++ ){
    for (uint8_t i = 0; i < PID_NumPin; i++)
    {
      if(ADC_F(PID_SetupPin[i]) > PID_Max[i] || PID_Max[i] >= 1023 ){
        PID_Max[i]  = ADC_F(PID_SetupPin[i]);
        if(PID_Max[i] > 1023 )PID_Max[i] = 1023;
      }
    }
    for (uint8_t i = 0; i < PID_NumPin; i++)
    {
      if(ADC_F(PID_SetupPin[i]) < PID_Min[i] || PID_Min[i] == 0){
        PID_Min[i] = ADC_F(PID_SetupPin[i]);
        if(PID_Min[i] < 0) PID_Min[i] = 0;
      }
    }
    delay(1);
  }
  tft_.fillScreen(ST77XX_BLACK);
  printText(0,0,"Calibrate completed",3,ST77XX_YELLOW,ST77XX_RED);
  buzzer(500,100);
}
int ReadSensorMinValue(uint8_t _Pin){
    return PID_Min[_Pin];
}
int ReadSensorMaxValue(uint8_t _Pin){
    return PID_Max[_Pin];
}



// int readline() {
//   bool onLine = false;  // ตัวบอกว่ามีเซ็นเซอร์ใดที่ตรวจจับเส้นได้หรือไม่
//   long avg = 0;         // ผลรวมถ่วงน้ำหนักของดัชนีเซ็นเซอร์
//   long sum = 0;         // ผลรวมของค่าที่เซ็นเซอร์อ่านได้

//   bool centerSensorsOnLine = false;  // ตัวบอกว่ามีเซ็นเซอร์ตรงกลางที่ตรวจจับเส้นได้หรือไม่
//   long centerAvg = 0;                // ผลรวมถ่วงน้ำหนักของดัชนีเซ็นเซอร์ตรงกลาง
//   long centerSum = 0;                // ผลรวมของค่าที่เซ็นเซอร์ตรงกลางอ่านได้

//   for (uint8_t i = 0; i < PID_NumPin; i++)  // เปลี่ยนจำนวนเซ็นเซอร์เป็น 16
//   {
//     long value;
//     if(Front_color == 0){
//       value = map(ADC_F(PID_SetupPin[i]), PID_Min[i], PID_Max[i], 100, 0);
//     }
//     else {
//       value = map(ADC_F(PID_SetupPin[i]), PID_Min[i], PID_Max[i], 0, 100);
//     }
    
//     value = constrain(value,0,100);
//     if (value > 100 - _Sensitive_F) {         // ตรวจสอบว่าค่าที่เซ็นเซอร์อ่านได้เกินกว่าค่าความไวหรือไม่
//       onLine = true;                  // ถ้าเกิน ให้ตั้งค่า onLine เป็นจริง
//     }
//     if (i >= 1 && i <= 3 && value > 100 - _Sensitive_F) {  // ตรวจสอบว่าค่าที่เซ็นเซอร์ตรงกลาง (7 ถึง 9) อ่านได้เกินกว่าค่าความไวหรือไม่
//       centerSensorsOnLine = true;                   // ถ้าเกิน ให้ตั้งค่า centerSensorsOnLine เป็นจริง
//     }
//     if (value > 5)  // ตรวจสอบว่าค่าที่เซ็นเซอร์อ่านได้เกินกว่าค่าขั้นต่ำหรือไม่
//     {
//       avg += (long)value * (i * 100);          // เพิ่มค่าน้ำหนักที่ได้จากเซ็นเซอร์ลงใน avg
//       sum += value;                            // เพิ่มค่าที่เซ็นเซอร์อ่านได้ลงใน sum
//       if (i >= 1 && i <= 3) {                 // ถ้าเป็นเซ็นเซอร์ตรงกลาง (7 ถึง 9)
//         centerAvg += (long)value * (i * 100);  // เพิ่มค่าน้ำหนักที่ได้จากเซ็นเซอร์ตรงกลางลงใน centerAvg
//         centerSum += value;                    // เพิ่มค่าที่เซ็นเซอร์ตรงกลางอ่านได้ลงใน centerSum
//       }
//     }
//   }

//   if (!onLine)  // ถ้าไม่มีเซ็นเซอร์ใดที่ตรวจจับเส้นได้
//   {
//     if (_lastPosition < (PID_NumPin - 1) * 100 / 2)  // ตรวจสอบตำแหน่งล่าสุดว่าอยู่ในครึ่งซ้ายหรือไม่
//     {
//       return 0;  // ถ้าใช่ ให้ส่งคืนค่า 0 แสดงว่าเส้นอยู่ทางซ้าย
//     } else       // ถ้าตำแหน่งล่าสุดอยู่ในครึ่งขวา
//     {
//       return (PID_NumPin - 1) * 100;  // ส่งคืนค่าสูงสุด แสดงว่าเส้นอยู่ทางขวา
//     }
//   }

//   if (centerSensorsOnLine) {                // ถ้ามีเซ็นเซอร์ตรงกลางที่ตรวจจับเส้นได้
//     _lastPosition = centerAvg / centerSum;  // คำนวณตำแหน่งเฉลี่ยถ่วงน้ำหนักของเซ็นเซอร์ตรงกลาง
//   } else {                                  // ถ้าไม่มีเซ็นเซอร์ตรงกลางที่ตรวจจับเส้นได้
//     _lastPosition = avg / sum;              // คำนวณตำแหน่งเฉลี่ยถ่วงน้ำหนักของเซ็นเซอร์ทั้งหมด
//   }

//   return _lastPosition;  // ส่งคืนตำแหน่งที่คำนวณได้
// }
int readline()   
{                
  int onLine = 0;
  long avg = 0;
  long sum = 0;
  for (uint8_t i = 0; i < PID_NumPin ; i++) 
  {
    long value;
    if(Front_color == 0){
      value = map(ADC_F(PID_SetupPin[i]), PID_Min[i], PID_Max[i], 100, 0);
    }
    else {
      value = map(ADC_F(PID_SetupPin[i]), PID_Min[i], PID_Max[i], 0, 100);
    }
    
    value = constrain(value,0,100);      
    if (value > 100 - _Sensitive_F) { 
      onLine = 1;
    }
    if (value > 5){
      avg += (long)value * (i * 100)+ 50;
      sum += value;
    }
  }
  if (onLine == 0)
  {
    if (_lastPosition < ((PID_NumPin-1) * 100)/2){
      return 0;
    }
    else{
      return ((PID_NumPin-1) * 100);
    }
  }
  _lastPosition = avg / sum;
  return _lastPosition;
}
void Run_PID(int RUN_PID_speed,float RUN_PID_KP,float RUN_PID_KD){
  
  int present_position = readline();
  int setpoint = ((PID_NumPin - 1) * 100) / 2;
  errors = present_position - setpoint;
  integral = integral + errors ;
  derivative = (errors - previous_error) ;
  output = RUN_PID_KP * errors  + RUN_PID_KD * derivative;
    
  int m1Speed = RUN_PID_speed + output ;
  int m2Speed = RUN_PID_speed - output;
  motor(1,m1Speed);
  motor(2,m2Speed);
  delay(1);
  previous_error = errors;
}
void Run_PID4DW(int RUN_PID_speed,float RUN_PID_KP,float RUN_PID_KD){
  
  int present_position = readline();
  int setpoint = ((PID_NumPin - 1) * 100) / 2;
  errors = present_position - setpoint;
  integral = integral + errors ;
  derivative = (errors - previous_error) ;
  output = RUN_PID_KP * errors  + RUN_PID_KD * derivative;
    
  int m1Speed = RUN_PID_speed + output ;
  int m2Speed = RUN_PID_speed - output;
  motor(1,m1Speed);
  motor(2,m2Speed);
  motor(3,m1Speed);
  motor(4,m2Speed);
  delay(1);
  previous_error = errors;
}




void setSensitive_B(int _SensorSensitive)
{
    _Sensitive_B = _SensorSensitive;
}
void PID_set_Min_B(int S0,int S1,int S2,int S3,int S4,int S5,int S6,int S7){
  PID_Min_B[0] = S0;PID_Min_B[1] = S1;PID_Min_B[2] = S2;PID_Min_B[3] = S3;
  PID_Min_B[4] = S4;PID_Min_B[5] = S5;PID_Min_B[6] = S6;PID_Min_B[7] = S7;
}
void PID_set_Max_B(int S0,int S1,int S2,int S3,int S4,int S5,int S6,int S7){
  PID_Max_B[0] = S0;PID_Max_B[1] = S1;PID_Max_B[2] = S2;PID_Max_B[3] = S3;
  PID_Max_B[4] = S4;PID_Max_B[5] = S5;PID_Max_B[6] = S6;PID_Max_B[7] = S7;
}
void PID_set_Pin_B(int S0,int S1,int S2,int S3,int S4,int S5,int S6,int S7){
  PID_SetupPin_B[0] = S0;PID_SetupPin_B[1] = S1;PID_SetupPin_B[2] = S2;PID_SetupPin_B[3] = S3;
  PID_SetupPin_B[4] = S4;PID_SetupPin_B[5] = S5;PID_SetupPin_B[6] = S6;PID_SetupPin_B[7] = S7;
}
void PID_set_Min16_B(int S0,int S1,int S2,int S3,int S4,int S5,int S6,int S7,
                     int S8,int S9,int S10,int S11,int S12,int S13,int S14,int S15){
  PID_Min_B[0]=S0;PID_Min_B[1]=S1;PID_Min_B[2]=S2;PID_Min_B[3]=S3;
  PID_Min_B[4]=S4;PID_Min_B[5]=S5;PID_Min_B[6]=S6;PID_Min_B[7]=S7;
  PID_Min_B[8]=S8;PID_Min_B[9]=S9;PID_Min_B[10]=S10;PID_Min_B[11]=S11;
  PID_Min_B[12]=S12;PID_Min_B[13]=S13;PID_Min_B[14]=S14;PID_Min_B[15]=S15;
}
void PID_set_Max16_B(int S0,int S1,int S2,int S3,int S4,int S5,int S6,int S7,
                     int S8,int S9,int S10,int S11,int S12,int S13,int S14,int S15){
  PID_Max_B[0]=S0;PID_Max_B[1]=S1;PID_Max_B[2]=S2;PID_Max_B[3]=S3;
  PID_Max_B[4]=S4;PID_Max_B[5]=S5;PID_Max_B[6]=S6;PID_Max_B[7]=S7;
  PID_Max_B[8]=S8;PID_Max_B[9]=S9;PID_Max_B[10]=S10;PID_Max_B[11]=S11;
  PID_Max_B[12]=S12;PID_Max_B[13]=S13;PID_Max_B[14]=S14;PID_Max_B[15]=S15;
}
void PID_set_Pin16_B(int S0,int S1,int S2,int S3,int S4,int S5,int S6,int S7,
                     int S8,int S9,int S10,int S11,int S12,int S13,int S14,int S15){
  PID_SetupPin_B[0]=S0;PID_SetupPin_B[1]=S1;PID_SetupPin_B[2]=S2;PID_SetupPin_B[3]=S3;
  PID_SetupPin_B[4]=S4;PID_SetupPin_B[5]=S5;PID_SetupPin_B[6]=S6;PID_SetupPin_B[7]=S7;
  PID_SetupPin_B[8]=S8;PID_SetupPin_B[9]=S9;PID_SetupPin_B[10]=S10;PID_SetupPin_B[11]=S11;
  PID_SetupPin_B[12]=S12;PID_SetupPin_B[13]=S13;PID_SetupPin_B[14]=S14;PID_SetupPin_B[15]=S15;
}
void setCalibrate_B(int round){
  buzzer(700,100);
  tft_.fillScreen(ST77XX_BLACK);
  printText(0,0,"Calibrate Back Sensor",3,ST77XX_RED,ST77XX_GREEN);
  // display.clear();
  // display.setFont(ArialMT_Plain_24);
  // display.drawString(0,0,"Back Sensor");
  // display.drawString(0,25,"  Calribrate  ");
  // display.display();
  if(first_state_for_calribrate_B == 0){
    for (uint8_t i = 0; i < PID_NumPin_B; i++)
    {
      PID_Max_B[i] = 0;
      PID_Min_B[i] = 1023;
    }
    first_state_for_calribrate_B = 1;
  }
  for(int roundOfCalribtate = 0; roundOfCalribtate < round ;roundOfCalribtate++ ){
    for (uint8_t i = 0; i < PID_NumPin_B; i++)
    {
      if(ADC_B(PID_SetupPin_B[i]) > PID_Max_B[i] || PID_Max_B[i] >= 1023 ){
        PID_Max_B[i]  = ADC_B(PID_SetupPin_B[i]);
        if(PID_Max_B[i] > 1023 )PID_Max_B[i] = 1023;
      }
    }
    for (uint8_t i = 0; i < PID_NumPin_B; i++)
    {
      if(ADC_B(PID_SetupPin_B[i]) < PID_Min_B[i] || PID_Min_B[i] == 0){
        PID_Min_B[i] = ADC_B(PID_SetupPin_B[i]);
        if(PID_Min_B[i] < 0) PID_Min_B[i] = 0;
      }
    }
    delay(1);
  }
  tft_.fillScreen(ST77XX_BLACK);
  printText(0,0,"Calibrate completed",3,ST77XX_RED,ST77XX_GREEN);
  buzzer(700,100);
}
int readline_B()   
{                
  bool onLine_B = false;
  long avg_B = 0;
  long sum_B = 0;
  for (uint8_t i = 0; i < PID_NumPin_B ; i++) 
  {
    long value_B;
    if(Back_color == 0){
      value_B = map(ADC_B(PID_SetupPin_B[i]), PID_Min_B[i], PID_Max_B[i], 100, 0);
    }
    else {
      value_B = map(ADC_B(PID_SetupPin_B[i]), PID_Min_B[i], PID_Max_B[i], 0, 100);
    }
    value_B = constrain(value_B,0,100); 
    if (value_B > 100 - _Sensitive_B) { 
      onLine_B = true;
    }
    if (value_B > 5){
      avg_B += (long)value_B * (i * 100)+50;
      sum_B += value_B;
    }
  }
  if (!onLine_B)
  {
    if (_lastPosition_B < ((PID_NumPin_B-1) * 100)/2){
      return 0;
    }
    else{
      return ((PID_NumPin_B-1) * 100);
    }
  }
  _lastPosition_B = avg_B / sum_B;
  return _lastPosition_B;
}
void Run_PID_B(int RUN_PID_speed,float RUN_PID_KP,float RUN_PID_KD){
  int present_position_B = readline_B();
  int setpoint_B = ((PID_NumPin_B - 1) * 100) / 2;
  errors_B = present_position_B - setpoint_B;
  integral_B = integral_B + errors_B ;
  derivative_B = (errors_B - previous_error_B) ;
  output_B = RUN_PID_KP * errors_B  + RUN_PID_KD * derivative_B;
  
  int m1Speed = RUN_PID_speed - output_B ;
  int m2Speed = RUN_PID_speed + output_B;

  motor(1,-m1Speed);
  motor(2,-m2Speed);
  delay(1);
  previous_error_B = errors_B;
}
void Run_PID_B4DW(int RUN_PID_speed,float RUN_PID_KP,float RUN_PID_KD){
  int present_position_B = readline_B();
  int setpoint_B = ((PID_NumPin_B - 1) * 100) / 2;
  errors_B = present_position_B - setpoint_B;
  integral_B = integral_B + errors_B ;
  derivative_B = (errors_B - previous_error_B) ;
  output_B = RUN_PID_KP * errors_B  + RUN_PID_KD * derivative_B;
  
  int m1Speed = RUN_PID_speed - output_B ;
  int m2Speed = RUN_PID_speed + output_B;

  motor(1,-m1Speed);
  motor(2,-m2Speed);
  motor(3,-m1Speed);
  motor(4,-m2Speed);
  delay(1);
  previous_error_B = errors_B;
}




bool Read_status_sensor(int pin_sensor){
	return ADC_F(PID_SetupPin[pin_sensor]) < ((PID_Max[pin_sensor] + PID_Min[pin_sensor]) / 2) ? true : false;
}
bool Read_status_sensor_B(int pin_sensor){
	return ADC_B(PID_SetupPin_B[pin_sensor]) < ((PID_Max_B[pin_sensor] + PID_Min_B[pin_sensor]) / 2) ? true : false;
}

int Read_sumValue_sensor(){
	int value = 0;
	for(int i = 0;i<PID_NumPin;i++){
		if(Front_color == 0){
	      value += map(ADC_F(PID_SetupPin[i]), PID_Min[i], PID_Max[i], 100, 0);
	    }
	    else {
	      value += map(ADC_F(PID_SetupPin[i]), PID_Min[i], PID_Max[i], 0, 100);
	    }	
	}
	 
    return value;
}

int Read_sumValue_sensor_B(){
  int value = 0;
  for(int i = 0;i<PID_NumPin_B;i++){
    if(Back_color == 0){
        value += map(ADC_B(PID_SetupPin_B[i]), PID_Min_B[i], PID_Max_B[i], 100, 0);
      }
      else {
        value += map(ADC_B(PID_SetupPin_B[i]), PID_Min_B[i], PID_Max_B[i], 0, 100);
      } 
  }
   
    return value;
}


int ReadSensorMinValue_B(uint8_t _Pin){
    return PID_Min_B[_Pin];
}
int ReadSensorMaxValue_B(uint8_t _Pin){
    return PID_Max_B[_Pin];
}

void Run_PID_B_until_backSensor(int RUN_PID_speed,float RUN_PID_KP,float RUN_PID_KD,int sumValue_traget){
  do{
    int present_position_B = readline_B();
    int setpoint_B = ((PID_NumPin_B - 1) * 100) / 2;
    errors_B = present_position_B - setpoint_B;
    integral_B = integral_B + errors_B ;
    derivative_B = (errors_B - previous_error_B) ;
    output_B = RUN_PID_KP * errors_B  + RUN_PID_KD * derivative_B;
    
    int m1Speed = RUN_PID_speed - output_B ;
    int m2Speed = RUN_PID_speed + output_B;

    motor(1,-m1Speed);
    motor(2,-m2Speed);
    delay(1);
    previous_error_B = errors_B;
  }while(Read_sumValue_sensor_B() < sumValue_traget);
}
void Run_PID_B4DW_until_backSensor(int RUN_PID_speed,float RUN_PID_KP,float RUN_PID_KD ,int sumValue_traget){
  do{
    int present_position_B = readline_B();
    int setpoint_B = ((PID_NumPin_B - 1) * 100) / 2;
    errors_B = present_position_B - setpoint_B;
    integral_B = integral_B + errors_B ;
    derivative_B = (errors_B - previous_error_B) ;
    output_B = RUN_PID_KP * errors_B  + RUN_PID_KD * derivative_B;
    
    int m1Speed = RUN_PID_speed - output_B ;
    int m2Speed = RUN_PID_speed + output_B;

    motor(1,-m1Speed);
    motor(2,-m2Speed);
    motor(3,-m1Speed);
    motor(4,-m2Speed);
    delay(1);
    previous_error_B = errors_B;
  }while(Read_sumValue_sensor_B() < sumValue_traget);
}

void Run_PID_until_frontSensor(int RUN_PID_speed,float RUN_PID_KP,float RUN_PID_KD,int sumValue_traget){
  do{
    int present_position = readline();
    int setpoint = ((PID_NumPin - 1) * 100) / 2;
    errors = present_position - setpoint;
    integral = integral + errors ;
    derivative = (errors - previous_error) ;
    output = RUN_PID_KP * errors  + RUN_PID_KD * derivative;
    
    int m1Speed = RUN_PID_speed + output ;
    int m2Speed = RUN_PID_speed - output;
    motor(1,m1Speed);
    motor(2,m2Speed);
    delay(1);
    previous_error = errors;
  }while(Read_sumValue_sensor() < sumValue_traget);
}
void Run_PID4DW_until_frontSensor(int RUN_PID_speed,float RUN_PID_KP,float RUN_PID_KD ,int sumValue_traget){
  do{
    int present_position = readline();
    int setpoint = ((PID_NumPin - 1) * 100) / 2;
    errors = present_position - setpoint;
    integral = integral + errors ;
    derivative = (errors - previous_error) ;
    output = RUN_PID_KP * errors  + RUN_PID_KD * derivative;
    

    int m1Speed = RUN_PID_speed + output ;
    int m2Speed = RUN_PID_speed - output;
    motor(1,m1Speed);
    motor(2,m2Speed);
    motor(3,m1Speed);
    motor(4,m2Speed);
    delay(1);
    previous_error = errors;
  }while(Read_sumValue_sensor() < sumValue_traget);
}

bool set_TCS_back(){
  Wire1.setSDA(6);
  Wire1.setSCL(7);
  Wire1.begin();
  Wire1.setClock(400000);
  tcsBackReady = tcsB.begin(0x29, &Wire1);
  Wire1.setClock(400000);
  if (tcsBackReady) Serial.println("Found back color sensor");
  else Serial.println("Back color sensor initialization failed");
  return tcsBackReady;
}

bool isTCSFrontReady() { return tcsFrontReady; }
bool isTCSBackReady() { return tcsBackReady; }

void clearTCSBackCache() {
  TCS_R = 0;
  TCS_G = 0;
  TCS_B = 0;
  TCS_T = 0;
  TCS_L = 0;
}

bool isConnected(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}
bool isConnected_B(uint8_t addr) {
  if (!tcsBackReady) return false;
  Wire1.beginTransmission(addr);
  return (Wire1.endTransmission() == 0);
}

long Read_Color_TCS(int color_of_sensor)
{
  float average_lib, r_lib, g_lib, b_lib;
  long data_color = 0.00;
  uint16_t clearcol_lib, red_lib, green_lib, blue_lib,lux,colorTemp;
 //delay(100); // Farbmessung dauert c. 50ms 

	if(tcsFrontReady && isConnected(0x29)){
		 tcs.getRawData(&red_lib, &green_lib, &blue_lib, &clearcol_lib);
     const uint32_t rgbSum = (uint32_t)red_lib + green_lib + blue_lib;
     if (rgbSum == 0) return 0;
		 colorTemp = tcs.calculateColorTemperature(red_lib, green_lib, blue_lib);
		  lux = tcs.calculateLux(red_lib, green_lib, blue_lib);

		 average_lib = rgbSum / 3.0f;
		 r_lib = red_lib/average_lib;
		 g_lib = green_lib/average_lib;
		 b_lib = blue_lib/average_lib;
		 if(color_of_sensor == 0){
		  data_color =  r_lib*100;
		 }
		 else if(color_of_sensor == 1){
		  data_color =  g_lib*100;
		 }
		  else if(color_of_sensor == 2){
		  data_color =  b_lib*100;
		 }
		 else if(color_of_sensor == 3){
		  data_color =  colorTemp;
		 }
		 else if(color_of_sensor == 4){
		  data_color =  lux;
		}
 	}
  
  
  return data_color;
}

bool updateTCSBackCache() {
  if (!tcsBackReady || !isConnected_B(0x29)) {
    clearTCSBackCache();
    return false;
  }

  uint16_t clearValue = 0, red = 0, green = 0, blue = 0;
  tcsB.getRawData(&red, &green, &blue, &clearValue);
  const uint32_t rgbSum = (uint32_t)red + green + blue;
  if (rgbSum == 0) {
    clearTCSBackCache();
    return false;
  }

  const float average = rgbSum / 3.0f;
  TCS_R = (int)((red / average) * 100.0f);
  TCS_G = (int)((green / average) * 100.0f);
  TCS_B = (int)((blue / average) * 100.0f);
  TCS_T = (int)tcsB.calculateColorTemperature(red, green, blue);
  TCS_L = (int)tcsB.calculateLux(red, green, blue);
  return true;
}

int Read_Color_TCS_B(int color_of_sensor)
{
  if (!tcsBackReady) return 0;
  long data_color = 0.00;
	 if(color_of_sensor == 0){
	  data_color =  TCS_R;
	 }
	 else if(color_of_sensor == 1){
	  data_color =  TCS_G;
	 }
	  else if(color_of_sensor == 2){
	  data_color =  TCS_B;
	 }
	 else if(color_of_sensor == 3){
	  data_color =  TCS_T;
	 }
	 else if(color_of_sensor == 4){
	  data_color =  TCS_L;
	 }

  return data_color;
}

int Read_Color_TCS_B_Raw(int color_of_sensor)
{
  if (!updateTCSBackCache()) return 0;
  return Read_Color_TCS_B(color_of_sensor);
}

int textWidth(const char* str, int size) {
  return strlen(str) * 6 * size;
}
int textHeight(int size) {
  return 8 * size;
}

void menu_show(int idx, const char* label) {
  int W = tft_.width();
  int H = tft_.height();

  tft_.fillScreen(ST77XX_BLACK);

  const char* title = "Menu";
  int titleSize = 2;
  int titleX = (W - textWidth(title, titleSize)) / 2;
  int titleY = (H / 4) - (textHeight(titleSize) / 2);
  printText(titleX, titleY, title, titleSize, ST77XX_WHITE);

  char buf[16];
  snprintf(buf, sizeof(buf), "#%d", idx);
  int numSize = 3;
  int numX = (W - textWidth(buf, numSize)) / 2;
  int numY = (H / 2) - (textHeight(numSize) / 2);
  printText(numX, numY, buf, numSize, ST77XX_YELLOW);

  int labelSize = 2;
  int labelX = (W - textWidth(label, labelSize)) / 2;
  int labelY = (3 * H / 4) - (textHeight(labelSize) / 2);
  printText(labelX, labelY, label, labelSize, ST77XX_GREEN);
}
