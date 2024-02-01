#define ENCA 3
#define ENCB 2
#define PWM 9
#define IN1 8

#undef DISPLAY  // Undefine DISPLAY macro
#include "ArrbotMonitor.h"

#include "ArrbotMonitor.h"
volatile int encoderCount = 0;
volatile int encoderState = 0;
volatile int lastEncoderState = 0;
long prevT = 0;
float eprev = 0;
float eintegral = 0;
void setup() {
Serial.begin(9600);
pinMode(9,OUTPUT);
pinMode(8,OUTPUT);
pinMode(ENCA,INPUT_PULLUP);
pinMode(ENCB,INPUT_PULLUP);
attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, CHANGE);
attachInterrupt(digitalPinToInterrupt(ENCB), readEncoder, CHANGE);
Serial.println("target pos");
}
void loop() {
// set target position
int target = 0;
// PID constants
float kp = 7;
float kd = 0;
float ki = 0;
// time difference
long currT = micros();
float deltaT = ((float) (currT - prevT))/( 1.0e6 );
prevT = currT;
// error
int e = (int)(encoderCount*360/1593)-target;
// derivative
float dedt = (e-eprev)/(deltaT);
// integral
eintegral = eintegral + e*deltaT;
// control signal
if(abs(e)>3){
float u = kp*e + kd*dedt + ki*eintegral;
// motor power
float pwr = fabs(u);
if( pwr > 255 ){
pwr = 255;
} // motor direction
int dir = 1;
if(u<0){
dir = -1;
}
// store previous error
eprev = e;
// signal the motor
setMotor(dir,pwr,PWM,IN1);
}
Serial.print(target);
Serial.print(" ");
Serial.print((int)encoderCount);
Serial.print(" ");
Serial.println();
}
void setMotor(int dir, int pwmVal, int pwm, int in1){
analogWrite(pwm,pwmVal);
if(dir == 1){
digitalWrite(in1,HIGH);
}
else if(dir == -1){
digitalWrite(in1,LOW);
}
else{
digitalWrite(in1,LOW);
}}
void readEncoder(){
encoderState = digitalRead(ENCA) << 1 | digitalRead(ENCB);
if (encoderState != lastEncoderState) {
if ((lastEncoderState == 0b00 && encoderState == 0b01) || (lastEncoderState == 0b01 &&
encoderState == 0b11) || (lastEncoderState == 0b11 && encoderState == 0b10) ||
(lastEncoderState == 0b10 && encoderState == 0b00)) {
encoderCount++;
} else {
encoderCount--;
} lastEncoderState =
encoderState;
}
}
