//For use with the Adafruit Motor Shield v2 
//---->	http://www.adafruit.com/products/1438
//
// uses the hc-sr04 ping sensor

// slam some libraries in
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"
#include <Servo.h> 
#include <NewPing.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

// And connect a DC motor
Adafruit_DCMotor *myMotor = AFMS.getMotor(1);

// Servo library
Servo servo1;

// ping sensor setup
#define TRIGGER_PIN  2  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     3  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 400 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

//global variables here
  int Wall = 50;
  int MaxSpeed = 100;
  int i;
 
void setup() {
  Serial.begin(115200);           // set up Serial library

  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
  
  // Attach a servo
  servo1.attach(10);
  servo1.write(90); // set servo to neutral
   
}

// Print data to serial for diagnostics
void SerialPrintStuff(int DistCM, int rawLight) {
  
  Serial.print("Ping: ");
  Serial.print(DistCM); // Convert ping time to distance in cm and print result (0 = outside set distance range)
  Serial.print("cm");
  Serial.print(" ||| ");
  Serial.print("Light Level:");
  Serial.println(rawLight, DEC);
 
}

// All sonar calcs to go here
int FrontDistance(){
  delay(35);                      // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
  unsigned int uS = sonar.ping(); // Send ping, get ping time in microseconds (uS)
  int DistCM = (uS / US_ROUNDTRIP_CM);
  return DistCM; // send back distance in CM

}

// all light readings and calcs
int LightLevel(){
  int rawLight = analogRead(A0); // set light pin
  return rawLight; //send back raw light value 
}

// All Motor control stuff here
void MotorForward(){
   myMotor->setSpeed(MaxSpeed);    
   myMotor->run(FORWARD);
}

void MotorStop(){
    myMotor->setSpeed(0);    
    myMotor->run(RELEASE);
    delay(1000);
 }

void Backward(){
   servo1.write(130); // turn wheels right 
   myMotor->setSpeed(MaxSpeed);    
   myMotor->run(BACKWARD);
   delay(2000);
   servo1.write(90); 
   MotorStop();   
   TakeOffAgain();
 }

void TakeOffAgain(){
   servo1.write(50); // turn wheels left 
   myMotor->setSpeed(MaxSpeed);    
   myMotor->run(FORWARD);
   delay(2000);
   servo1.write(90); 
 }

void loop() {
  
  if (FrontDistance() <= Wall){ 
       MotorStop();
       Backward();
  }else{
      MotorForward(); // default movement
  }

// send things we care about to serial function
 SerialPrintStuff(FrontDistance(), LightLevel());
 
}
