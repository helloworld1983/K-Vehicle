#include <AFMotor.h>

// Adafruit Motor shield library
// copyright Adafruit Industries LLC, 2009
// this code is public domain, enjoy!


#include <Servo.h> 

#include <NewPing.h>

#define TRIGGER_PIN  12  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     11  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.



// DC motor on M2
AF_DCMotor motor(2);
AF_DCMotor motork(1);
// DC hobby servo
Servo servo1;
// Stepper motor on M3+M4 48 steps per revolution
AF_Stepper stepper(48, 2);

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Motor party!");
  
  // turn on servo
  servo1.attach(9);
   
  // turn on motor #1
  motor.setSpeed(200);
  motor.run(RELEASE);

   // turn on motor #2
  motork.setSpeed(200);
  motork.run(RELEASE);
}

int i;

// Test the DC motor, stepper and servo ALL AT ONCE!
void loop() {

   delay(50);                     // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
  Serial.print("Ping: ");
  Serial.print(sonar.ping_cm()); // Send ping, get distance in cm and print result (0 = outside set distance range)
  Serial.println("cm");
  
  motor.run(FORWARD);
  motork.run(FORWARD);
  for (i=0; i<255; i++) {
    servo1.write(i);
    motor.setSpeed(i);  
    stepper.step(1, FORWARD, INTERLEAVE);
    delay(3);
 }
 
  for (i=255; i!=0; i--) {
    servo1.write(i-255);
    motor.setSpeed(i);  
    stepper.step(1, BACKWARD, INTERLEAVE);
    delay(3);
 }
 
  motor.run(BACKWARD);
  motork.run(BACKWARD);
  for (i=0; i<255; i++) {
    servo1.write(i);
    motor.setSpeed(i);  
    delay(3);
    stepper.step(1, FORWARD, DOUBLE);
 }
 
  for (i=255; i!=0; i--) {
    servo1.write(i-255);
    motor.setSpeed(i);  
    stepper.step(1, BACKWARD, DOUBLE);
    delay(3);
 }
}
