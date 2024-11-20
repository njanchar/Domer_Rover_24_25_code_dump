// Arm Control Proto
#include <Encoder.h>
#include <SoftwareSerial.h>
#include <Cytron_SmartDriveDuo.h>

int dig1Pin = 12;
int dig2Pin = 13;
int ana1Pin = 10;
int ana2Pin = 11;
int potPin = A0;

Encoder myEnc(2, 3);
Cytron_SmartDriveDuo smartDriveDuo30(PWM_INDEPENDENT, dig1Pin, dig2Pin, ana1Pin, ana2Pin);

long oldPos = -999;
long newPos;
int potPos;

int maxSlide = 988;
int minslide = 144;



void setup() {
  Serial.begin(9600);
  smartDriveDuo30.control(0, 0);
  read_enc();
  read_pot();
  Serial.print(potPos);
  delay(3000);
  //slideToPos(400, 25);
}

void loop() {
  //Serial.println(potPos);
  read_pot();
  read_enc();
  smartDriveDuo30.control(25, 25);
  delay(3000);
  smartDriveDuo30.control(-25, -25);
  delay(3000);

}

void read_enc(){
   newPos = myEnc.read();
  if(newPos != oldPos){
    oldPos = newPos;
    Serial.println(newPos);
  }
}

void read_pot(){
  potPos = analogRead(potPin);
}

void slideToPos(int length, int speed){
  int dir = 1;
  if(length < potPos){dir = -1;}
  smartDriveDuo30.control(dir*speed, 0);
  int diff = potPos - length;
  while(diff > 5 || diff < -5){
    read_pot();
    diff = potPos - length;
    delay(10);
  }
  smartDriveDuo30.control(0, 0);
}

