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

// And connect a DC motor to port M3
Adafruit_DCMotor *myMotor = AFMS.getMotor(1);

// Servo library
Servo servo1;

// ping sensor setup
#define TRIGGER_PIN  2  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     3  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 500 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

void setup() {
  Serial.begin(115200);           // set up Serial library at 9600 bps

  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
  
  // Attach a servo to pin #10
  servo1.attach(10);
   
  // turn on motor M1
  myMotor->setSpeed(200);
  myMotor->run(RELEASE);
}

// Print data to serial for diagnostics
void SerialPrintStuff(int ToPrint) {
  
  Serial.print("Ping: ");
  Serial.print(ToPrint); // Convert ping time to distance in cm and print result (0 = outside set distance range)
  Serial.println("cm");
  
}

// All sonar calcs to go here
int FrontDistance(){
  unsigned int uS = sonar.ping(); // Send ping, get ping time in microseconds (uS)
  int DistCM = (uS / US_ROUNDTRIP_CM);
  if (DistCM == 0){
    DistCM = 21;
  }
return DistCM;

  if (DistCM < 20){
    return 1;
  }else{
    return 0;
  }
}

// Motor control stuff here
void RunMotor(int DistCM){
  
  if (DistCM > 20){
    myMotor->run(FORWARD);
    myMotor->setSpeed(100);  
  }else{
    myMotor->run(RELEASE);
  }
}


void loop() {
  delay(50);                      // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
  
 RunMotor(FrontDistance());
 SerialPrintStuff(FrontDistance());
}
