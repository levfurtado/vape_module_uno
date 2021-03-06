#include <Arduino.h>

#include <Wire.h>
#include <SPI.h>
#include "Vape.h"
#include <Adafruit_ADS1015.h>
#include <PID_v1.h>

int pioPin = 4;
int csPin = 10;
int ohmEnable = 7;
int fivePin = 8;
uint8_t adsAddress = 0x49;
int probe = 2;
double KpRaw = 1;
double KiRaw = 4;
double KdRaw = 0;
double pidSetpoint = 0;

float userWattage = 10.000;
float setpoint = userWattage;
int operatingMode = 1;

bool not_init = true;

FiveFiveFive five(fivePin);
PhysicalInput pio(pioPin);
DigitalPot digiPot(csPin);
Translator translate(ohmEnable);
VoltageMeter vMeter(adsAddress);

double InputRaw = 0;
double OutputRaw = 0;
double SetpointRaw = 0;
PID myPIDRaw(&InputRaw, &OutputRaw, &SetpointRaw, KpRaw, KiRaw, KdRaw, DIRECT);

void setup() {
  Serial.begin(9600);
  pinMode(6, OUTPUT);
  myPIDRaw.SetMode(AUTOMATIC);
  myPIDRaw.SetOutputLimits(0, 255);
  myPIDRaw.SetSampleTime(1);
}

void loop() {

  if(not_init) {
    Serial.println("not init");
    pidSetpoint = translate.takeInput( operatingMode, setpoint);
    not_init = false;
  }

  if(digitalRead(pioPin) == 1) {
    // Used in order for raspberry pi monitoring device to get "up to speed"
    delay(500);
    five.fiveEnable();
    while(digitalRead(pioPin) == 1) {
      digitalWrite(6, 1);
      // digitalWrite(6, 0);
      InputRaw = vMeter.voltage(probe);
      SetpointRaw = pidSetpoint;
      myPIDRaw.Compute();
      int intOutput = OutputRaw;
      digiPot.writeToPot(intOutput);
    }
  }
  digitalWrite(6, 0);
  five.fiveKill();
  digiPot.killPot();
}
