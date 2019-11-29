//
// Joystick HID interface used for RC receier input to USB HID joystick device
// by MarkL/Mark Liu
// include these 2 libraries: 
//	https://github.com/GreyGnome/EnableInterrupt
//	https://github.com/GreyGnome/EnableInterrupt 
//
#include "EnableInterrupt.h"
#include "Joystick.h"

#define SERIAL_PORT_SPEED 57600
#define RC_NUM_CHANNELS  4

#define RC_CH1  0
#define RC_CH2  1
#define RC_CH3  2
#define RC_CH4  3

#define RC_CH1_INPUT  11 //2
#define RC_CH2_INPUT  10 //3
#define RC_CH3_INPUT  9  //7
#define RC_CH4_INPUT  A0 //8

uint16_t rc_values[RC_NUM_CHANNELS];
uint32_t rc_start[RC_NUM_CHANNELS];
volatile uint16_t rc_shared[RC_NUM_CHANNELS];

Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID,JOYSTICK_TYPE_GAMEPAD,
  2, 0,                  // Button Count, Hat Switch Count
  true, true, false,     // X and Y, but no Z Axis
  false, false, false,   // No Rx, Ry, or Rz
  false, false,          // No rudder or throttle
  false, false, false);  // No accelerator, brake, or steering

void setup() {
  //Serial.begin(SERIAL_PORT_SPEED);

  pinMode(RC_CH1_INPUT, INPUT);
  pinMode(RC_CH2_INPUT, INPUT);
  pinMode(RC_CH3_INPUT, INPUT);
  pinMode(RC_CH4_INPUT, INPUT);

  enableInterrupt(RC_CH1_INPUT, calc_ch1, CHANGE);
  enableInterrupt(RC_CH2_INPUT, calc_ch2, CHANGE);
  enableInterrupt(RC_CH3_INPUT, calc_ch3, CHANGE);
  enableInterrupt(RC_CH4_INPUT, calc_ch4, CHANGE);

  Joystick.begin();
  Joystick.setXAxisRange(1100, 1900);
  Joystick.setYAxisRange(1100, 1900);
}

void loop() {
  rc_read_values();
  
//  int averageX = 0;
//  int numOfAverages = 16; 
//
//  for(int i = 0; i<numOfAverages; i++) {
//    averageX += rc_values[RC_CH1];
//  }
//  averageX /= numOfAverages;
  
  Joystick.setXAxis(rc_values[RC_CH2]);
  Joystick.setYAxis(rc_values[RC_CH1]);
  
  if(rc_values[RC_CH3] < 1300 || rc_values[RC_CH3] > 1700) {
      Joystick.setButton(0,1); 
  }
  else {
      Joystick.setButton(0,0); 
  }

  if(rc_values[RC_CH4] < 1300 || rc_values[RC_CH4] > 1700) {
      Joystick.setButton(1,1); 
  }
  else {
      Joystick.setButton(1,0); 
  }

  delay(5);
}

void rc_read_values() {
  noInterrupts();
  memcpy(rc_values, (const void *)rc_shared, sizeof(rc_shared));
  interrupts();
}

void calc_input(uint8_t channel, uint8_t input_pin) {
  if (digitalRead(input_pin) == HIGH) {
    rc_start[channel] = micros();
  } else {
    uint16_t rc_compare = (uint16_t)(micros() - rc_start[channel]);
    rc_shared[channel] = rc_compare;
  }
}

void calc_ch1() { calc_input(RC_CH1, RC_CH1_INPUT); }
void calc_ch2() { calc_input(RC_CH2, RC_CH2_INPUT); }
void calc_ch3() { calc_input(RC_CH3, RC_CH3_INPUT); }
void calc_ch4() { calc_input(RC_CH4, RC_CH4_INPUT); }
