#include "mbed.h"
#include "C12832.h"
#include "QEI.h"

Serial pc(USBTX, USBRX);
C12832 LCD(D11, D13, D12, D7, D10);
Serial HM10(PA_11, PA_12);


//All the motor relavent objects created [Straight]
PwmOut ML(PC_8);
PwmOut MR(PC_6);
DigitalOut Enable(PC_9);
DigitalOut DirL(PB_9);
DigitalOut DirR(PB_8);
QEI WL(PC_1, PC_0, NC, 1024, QEI::X4_ENCODING);
QEI WR(PB_5, PB_3, NC, 1024, QEI::X4_ENCODING);


//Sensor class - takes one input which is the input pin and returns a analogue reading
class sensor
{
private:
AnalogIn inputSignal;
public:
sensor(PinName pin) : inputSignal(pin) {
}

float analogVal(void) {
return inputSignal.read();
}

float NormVal(void) {
float normVal = (inputSignal.read()/0.90);
if (normVal > 1) {normVal = 1;}
return normVal;
}
};

float iPWM = 0.70, rtPWM = 0.10, ltPWM = 0.10, sPWM = 0.50, Kp = 0.00015;

//One object for each sensor
sensor R(PB_1); sensor C(PC_4); sensor L(PC_2);

/* RPM measurement function as well all the relavent variables */
int oPulR = 0, nPulR = 0; // Old, new, and change in pulse value (Right)
int oPulL = 0, nPulL = 0; // Old, new, and change pulse value (Left)
float rRPM = 0.0, lRPM = 0.0; // Right and left wheel RPM

void rpmMeas() {
// Read new pulses
nPulR = WR.getPulses(); nPulL = WL.getPulses();
// Calculate change in pulses
float dPulR = nPulR - oPulR; float dPulL = nPulL - oPulL;
// Calculate RPM
rRPM = ((dPulR/0.05) * 60.0) / 1024.0;
lRPM = ((dPulL/0.05) * 60.0) / 1024.0;
// Update old pulses
oPulR = nPulR;
oPulL = nPulL;
}

void Uturn () {

DirL = false; DirR = true;
ML.write(1); MR.write(1);

wait(0.50);
WL.reset(); WR.reset();

while(WR.getPulses() <= 1350 && WL.getPulses() <= 1350) {
ML.write(0.55);
MR.write(0.6);
}

ML.write(1); MR.write(1);
}

bool Flag;

int main() {

//Setting up bluetooth
HM10.baud(9600);

//Turning the motors on
Enable = true;

while (1) {

rpmMeas();

if (rRPM < 270 || lRPM < 270) {sPWM = 0.50;}
if (rRPM > 310 || lRPM > 310) {sPWM = 0.80;}

if(HM10.readable()) {Flag = true;}

if(Flag){
//U-turn
char C = HM10.getc();

if(C == 'A') {Uturn();}
Flag = false;
}

while (L.NormVal() < 0.23 && C.NormVal() < 0.23 && R.NormVal() < 0.23) {
ML.write(1.0); MR.write(1.0);
}

DirL = false; DirR = false;

//While the center is right on top of the line, buggy moves straight
while(C.NormVal() >= 0.82) {

if(HM10.readable()) {Flag = true;}
if (Flag) {break;}

ML.write(/*SpeedControl(150, lRPM, sPWM)*/ 0.55); MR.write(/*SpeedControl(150, lRPM, sPWM)*/0.55);
}

//Turning right
if (R.NormVal() > L.NormVal()) {

ML.write(iPWM); MR.write(iPWM);
DirL = true; DirR = false;

while (C.NormVal() < 1) {

if(HM10.readable()) {Flag = true;}
if (Flag) {break;}

ML.write(rtPWM);
MR.write(rtPWM);

if (C.NormVal() < 0.82) {break;}
}

ML.write(iPWM); MR.write(iPWM);
}

//Turning left
if (L.NormVal() > R.NormVal()) {

ML.write(iPWM); MR.write(iPWM);
DirL = false; DirR = true;

while (C.NormVal() < 1) {

if(HM10.readable()) {Flag = true;}
if (Flag) {break;}

ML.write(ltPWM);
MR.write(ltPWM);

if (C.NormVal() < 0.82) {break;}
}

ML.write(iPWM); MR.write(iPWM);

}
}

return 0;
}