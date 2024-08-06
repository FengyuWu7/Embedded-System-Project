
#include "mbed.h"
#include "C12832.h" //Imports the library for the LCD screen
#include "QEI.h" //Imports the Quadrature Encoder Interface Library.
#include "math.h" //Imports the Math library for easy
#define PI 3.14159265358979323846
class PWM{
private:
PwmOut PwmSignal;
public:
PWM(PinName pin) : PwmSignal(pin){};
void write(float a){ //Public member function for turning the LED on
PwmSignal=a; //Set output to 0 (LED is active low)
}
void read(void){ //Public member function for turning the LED off
PwmSignal.read(); //Set output to 1 (LED is active low)
}
void period_us(int b){
PwmSignal.period_us(b);
}
};
class LED{
protected:
DigitalOut outputSignal; //Declaration of DigitalOut object
bool status; //Variable to recall the state of the LED
public: //Public declarations
LED(PinName pin) : outputSignal(pin){off();} //Constructor - user provided pin name is assigned to the DigitalOut
void on(void) //Public member function for turning the LED on
{
outputSignal = 0; //Set output to 0 (LED is active low)
status = true; //Set the status variable to show the LED is on
}
void off(void) //Public member function for turning the LED off
{
outputSignal = 1; //Set output to 1 (LED is active low)
status = false; //Set the status variable to show the LED is off
}
};
float setpoint; // desired output
float processVariable; // current output
float error123; // difference between setpoint and processVariable
float previousError; // error in previous iteration
float integral; // integral of error
float derivative; // derivative of error
float kp=0.5; // proportional gain
float ki=0.07; // integral gain
float kd=0.2; // derivative gain
float output; // output of the controller
float calculateOutput(float setpoint, float processVariable) {
error123 = setpoint - processVariable;
integral += error123;
derivative = error123 - previousError;
output = kp * error123 + ki * integral + kd * derivative;
previousError = error123;
return output;
}
//Initialisation of the encoders.
Timer encoderTimer;
float EncoderT,revolutionCounter1, revolutionCounter2,RPM1,RPM2, Speed1, Speed2, disCount,angleCount;
QEI encoder1 (PC_2, PC_3, NC, 512, QEI::X2_ENCODING);
QEI encoder2 (PC_1, PC_0, NC, 512, QEI::X2_ENCODING);
C12832 lcd(D11, D13, D12, D7, D10); //Creates an LCD Object from the LCD library
//Define the function getPuls here, so that can be called later on.
typedef enum {initialisation, TrackTheLine, TurnAround} ProgramState; //Initialisation of the state machines
ProgramState state;
int Pulses1=0, Pulses2=0; //Used to show pulses from encoders for TDA.
void getPuls(void){
EncoderT=encoderTimer.read();
encoderTimer.stop();
revolutionCounter1=encoder1.getPulses(),revolutionCounter2=encoder2.getPulses();
Pulses1=Pulses1+revolutionCounter1, Pulses2=Pulses2+revolutionCounter2;
RPM1=revolutionCounter1/EncoderT, RPM2=revolutionCounter2/EncoderT;
RPM1=RPM1/512, RPM2=RPM2/512*-1;
Speed1=RPM1*0.08*PI, Speed2=RPM2*0.08*PI;
disCount=disCount+((Speed1+Speed2)/2*EncoderT);
angleCount=angleCount+((Speed1-Speed2)/0.135*EncoderT);
encoder1.reset(), encoder2.reset(),encoderTimer.reset();
encoderTimer.start();
}
DigitalOut Direction1(PC_6), Direction2(PB_8);
PWM out1(PA_15), out2(PB_14);
static float PwmOut1=0.5, PwmOut2=0.5;
void brakes(){
Direction1=1,Direction2=1;
wait(0.03);
Direction1=0,Direction2=0;
PwmOut1=0.5,PwmOut2=0.5;
out1.write(PwmOut1),out2.write(PwmOut2);
wait(0.5);
}
LED greenLED(D9);
DigitalOut SO1(PA_10),SO2(PB_3),SO3(PB_5),SO4(PB_4),SO5(PB_13),SO6(PA_9);
AnalogIn SI1(PA_1),SI2(PB_1),SI3(PC_4),SI4(PA_4),SI5(PB_0),SI6(PC_5);
bool s1,s2,s3,s4,s5,s6;
float angleCountChanges=0;
void setPWM(void){
switch(state){ //Set the duty cycles according to the state machines. PID wil be a
case (initialisation) :
greenLED.off();
PwmOut1=0.5,PwmOut2=0.5;
out1.write(PwmOut1),out2.write(PwmOut2);
break;
case (TrackTheLine) :
greenLED.on();
//insert PID here
if(s3){
PwmOut1=0.6, PwmOut2=0.59;
};
if(s4){
PwmOut1=0.59, PwmOut2=0.6;
};
if(s5){
PwmOut1=0.55, PwmOut2=0.60;
};
if(s6){
PwmOut1=0.53, PwmOut2=0.60;
};
if(s2){
PwmOut1=0.6, PwmOut2=0.55;
};
if(s1){
PwmOut1=0.60, PwmOut2=0.53;
};
out1.write(PwmOut1+calculateOutput(PwmOut1,(Speed1+2.5)/5)),out2.write(PwmOut2+calculateOutput(PwmOut2,(Speed1+2.5)/5));
break;

case (TurnAround) :
angleCountChanges=angleCountChanges+abs((Speed1-Speed2)/0.14*EncoderT);
greenLED.off();
//insert PID here
PwmOut1=0.68, PwmOut2=0.33;
if(angleCountChanges>=(PI)){
if(s3||s4){
state = TrackTheLine;
disCount=0;
angleCount=0;
angleCountChanges=0;
}
}
out1.write(PwmOut1),out2.write(PwmOut2);
break;
}
}
//BLE
Serial hm10(PA_11, PA_12);
Serial pc(USBTX, USBRX);
char s;
char w;
void serial_config();
void serial_config(){
if (pc.readable()) {
w = pc.getc();
hm10.putc(w);
}
}
Timer noinput();
//
int main() {
//Powring on the sensor.
SO1=1,SO2=1,SO3=1,SO4=1,SO5=1,SO6=1;
//Codes for Bluetooth;
pc.baud(9600);
hm10.baud(9600); // Set up baud rate for serial communication
while (!hm10.writeable()) {
} // wait until the HM10 is ready
//End of Bluetooth;
Ticker encoderTicker, PwmTicker;
//Initialisation of the motors.
DigitalOut Bipolar1(PC_8), Bipolar2(PC_9), Enable(PB_12);
Bipolar1=1, Bipolar2=1, Enable=1; Direction1=0, Direction2=0; //Set the pin to TTL high.
out1.period_us(45); out2.period_us(45); // Setting the period to 25us so that wont hear the beeping noise.
C12832 lcd(D11, D13, D12, D7, D10); //Creates an LCD Object from the LCD library
state=initialisation;
/*This is the ticker code for encoders*/
encoderTicker.attach(&getPuls,0.01);
PwmTicker.attach(&setPWM, 0.005);
float result=0.0;
noinput.start();
while(1){
//sensor codes
if(SI1.read()<=0.6f){
s1=1;

}
else{
s1=0;
}
if(SI2.read()<=0.6f){
s2=1;

}
else{
s2=0;
}
if(SI3.read()<=0.6f){
s3=1;

}
else{
s3=0;
}
if(SI4.read()<=0.6f){
s4=1;

}
else{
s4=0;
}
if(SI5.read()<=0.6f){
s5=1;

}
else{
s5=0;
}
if(SI6.read()<=0.6f){
s6=1;

}
else{
s6=0;
}
if (hm10.readable()) {
s = hm10.getc();
pc.putc(s);
if (s == '1') {
state=TrackTheLine;
disCount=0;
angleCount=0;
}
if (s == '0') {
state=initialisation;
disCount=0;
angleCount=0;
}
if (s == '2') {
state=TurnAround;
disCount=0;
angleCount=0;
angleCountChanges=0;
encoderTimer.start();
}
serial_config();
}
lcd.locate(0,0);
lcd.printf("P: %.1f %.1f %.1f\n",disCount,angleCount, angleCountChanges);
lcd.printf("Bool: %d %d %d %d %d %d\n", s1,s2,s3,s4,s5,s6);
if(s1||s2||s3||s4||s5||s6==1){
noinput.reset();
}
result=noinput.read();
if (result>0.5){
PwmOut1=0.5, PwmOut2=0.5;
}
}
/* serial_config allows you to set up your HM-10 module via USB serial port*/
}

