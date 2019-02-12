// ---------------------------------------------------------------------------
// Final Sketch for Automous Object Aviodance Gps car With Ping & Compass
Sensor
// ---------------------------------------------------------------------------
/*
 ------------------Pin Configation For modules
 Pins POWERS
 GPS- 3.3V
 Compass- SDA- & SCL- {ANALOG PINS} 5.0V
 Ping - TRIGER- & ECHO - 5.0V
 Drive MOtor- Motor No. 2 {Motor Shield} EXTERNAL POWER
REQURIED FOR BETTER WORKING FOR BOTH MOTOR
 Turn Motor- Motor No. 4 {Motor Shield}
*/
////////////////////////////////////////////Include Files/////////////////////////////////////////////
#include "IRremote.h"
#include <NewPing.h> ////For ping sensor
#include <AFMotor.h> ////For motor Sheild
// Reference the I2C Library
#include <Wire.h>
// Reference the HMC5883L Compass Library
#include <HMC5883L.h>
//LIBARY FOR GPS
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <math.h> // used by: GPS
#include <waypointClass.h>
#include <LiquidCrystal.h>
////////////////////////////////////////////////////////////////////////////////////////////////////////
// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
// Signal Pin of IR receiver to Arduino Digital Pin 33
int receiver = 33; 
//BIT LALPUR Ext. CENTER | Batch 2014-17 K-Vehicle | 57
//Written Report
/*-----( Declare objects )-----*/
IRrecv irrecv(receiver); // create instance of 'irrecv'
decode_results results; // create instance of 'decode_results'
////////////////////defination of motor//////////////////////////////
AF_DCMotor motor(2);
AF_DCMotor motork(4);
/////////////////////////////////////////////////////////////////////
//////////////////////////////////Defination of ping sensor/////////////////////////////////////////////////
#define TRIGGER_PIN 22 // Arduino pin tied to trigger pin on the ultrasonic
sensor.
#define ECHO_PIN 23// Arduino pin tied to echo pin on the ultrasonic sensor.
#define TigR 24
#define EchR 25
#define TigL 26
#define EchL 27
#define TigB 28
#define EchB 29
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in
centimeters). Maximum sensor distance is rated at 400-500cm.
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing
setup of pins and maximum distance.
NewPing sonar_RT(TigR , EchR , MAX_DISTANCE);
NewPing sonar_LT(TigL, EchL , MAX_DISTANCE);
NewPing sonar_BK(TigB , EchB , MAX_DISTANCE);
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////Extra Configration////////////////////////////////////////////
// Object avoidance distances (in inches)
#define SAFE_DISTANCE 70
#define TURN_DISTANCE 21
#define STOP_DISTANCE 12
#define SSD 70
#define TURN_LEFT BACKWARD
BIT LALPUR Ext. CENTER | Batch 2014-17 K-Vehicle | 58
Written Report
#define TURN_RIGHT FORWARD
#define TURN_STRAIGHT RELEASE
// Speeds (range: 0 - 255)
#define FAST_SPEED 150
#define NORMAL_SPEED 125
#define TURN_SPEED 100
#define SLOW_SPEED 75
int speed = NORMAL_SPEED;
int sonarDistance, SR, SL, BckDist = 0;
// Steering/turning
enum directions {left = TURN_LEFT, right = TURN_RIGHT, straight =
TURN_STRAIGHT} ;
directions turnDirection = straight;
///////////////////////////////////////////////////////////////////////////////////////////////////////
//BIT LALPUR Ext. CENTER | Batch 2014-17 K-Vehicle | 59
//Written Report
#define NUMBER_WAYPOINTS 5 // enter the numebr of way points here
(will run from 0 to (n-1))
int waypointNumber = -1; // current waypoint number; will run from 0 to
(NUMBER_WAYPOINTS -1); start at -1 and gets initialized during setup()
waypointClass waypointList[NUMBER_WAYPOINTS] = {
waypointClass(23.371809,85.323378), waypointClass(24.037528,84.058476 ),
waypointClass(24.037387,84.058550), waypointClass(24.037092,84.058844),
waypointClass(24.038089, 84.059084)};
// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences.
#define GPSECHO true
// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////confi for compasss//////////////////////////////////////
// Store our compass as a variable.
HMC5883L compass;
// Record any errors that may occur in the compass.
int error = 0;
// Compass navigation
int targetHeading = 0; // where we want to go to reach current waypoint
int currentHeading; // where we are actually facing now
int headingError; // signed (+/-) difference between targetHeading and
currentHeading
#define HEADING_TOLERANCE 5 // tolerance +/- (in degrees) within which
we don't attempt to turn to intercept targetHeading
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
BIT LALPUR Ext. CENTER | Batch 2014-17 K-Vehicle | 60
Written Report
//////////////////////////////////////////////////////////////////////////////////////////Setup programs////////////////////////////////////////////////
void setup() {
 // set up the LCD's number of columns and rows:
 lcd.begin(16, 2);
 // Print a message to the LCD.
 lcd.print("Welcome!");
 //////Setup IR Remote
 Serial.println("IR Receiver Button Decode");
 irrecv.enableIRIn(); // Start the receiver

//////////////////////////////////////////////////////////////////////////////// // Setup for motor////////////////////////
 Serial.begin(115200); // Open serial monitor at 115200 baud to see ping results.
 // turn on motor
 motor.setSpeed(200);
 motork.setSpeed(255);
 Serial.println("Motor test!");
 motor.run(RELEASE);
 motork.run(RELEASE);
 /////////////////////////////////////////////////////////////////End-of motor//////////////////
//////////////////////////////////////////////////////Steup. For GPS System//////////////////////////////////
Serial.println("Adafruit GPS library basic test!");
 // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
 GPS.begin(9600);

 // uncomment this line to turn on RMC (recommended minimum) and GGA (fix
data) including altitude

 GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
 // uncomment this line to turn on only the "minimum recommended" data
 //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
 // For parsing data, we don't suggest using anything but either RMC only or
RMC+GGA since
BIT LALPUR Ext. CENTER | Batch 2014-17 K-Vehicle | 61
Written Report
 // the parser doesn't care about other sentences at this time

 // Set the update rate
 GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate


 // For the parsing code to work nicely and have time to sort thru the data, and
 // print it out we don't suggest using anything higher than 1 Hz
 // Request updates on antenna status, comment out to keep quiet
 GPS.sendCommand(PGCMD_ANTENNA);
 // the nice thing about this code is you can have a timer0 interrupt go off
 // every 1 millisecond, and read data from the GPS for you. that makes the
 // loop code a heck of a lot easier!
 useInterrupt(true);
 delay(1000);

 // Ask for firmware version
 mySerial.println(PMTK_Q_RELEASE);
 //
 // get initial waypoint; also sets the distanceToTarget and courseToTarget
varilables
// nextWaypoint();
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////setup for compasss//////////////////////////////////////
// Initialize the serial port.
 Serial.println("Starting the I2C interface.");
 Wire.begin(); // Start the I2C interface.
 compass = HMC5883L(); // Construct a new HMC5883 compass.


 error = compass.SetScale(1.3); // Set the scale of the compass.
BIT LALPUR Ext. CENTER | Batch 2014-17 K-Vehicle | 62
Written Report
 if(error != 0) // If there is an error, print it out.
 Serial.println(compass.GetErrorText(error));

 Serial.println("Setting measurement mode to continous.");
 error = compass.SetMeasurementMode(Measurement_Continuous); // Set the
measurement mode to Continuous
 if(error != 0) // If there is an error, print it out.
 Serial.println(compass.GetErrorText(error));
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
}
//////////////////////////////////////////////////////////////////////////////////////////////////END OF SETUP//////////////////////////////////////
//////////////////////////EXTRA GPS CONFIGRATION//////////////////////////////////////////
// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
 char c = GPS.read();

 // if you want to debug, this is a good time to do it!
#ifdef UDR0
 if (GPSECHO)
 if (c) UDR0 = c;

 // writing direct to UDR0 is much much faster than Serial.print

 // but only one character can be written at a time.
#endif
}
void useInterrupt(boolean v) {
 if (v) {
 // Timer0 is already used for millis() - we'll just interrupt somewhere
 // in the middle and call the "Compare A" function above

 OCR0A = 0xAF;
 TIMSK0 |= _BV(OCIE0A);
 usingInterrupt = true;
 } else {

 // do not call the interrupt function COMPA anymore
//BIT LALPUR Ext. CENTER | Batch 2014-17 K-Vehicle | 63
//Written Report
 TIMSK0 &= ~_BV(OCIE0A);
 usingInterrupt = false;
 }
}
uint32_t timer = millis();
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////// Main Loop of
Sketch/////////////////////////////////////////////
void loop() {
lcd.setCursor(0, 0);
 lcd.print("Aquring-GPS...");
 lcd.setCursor(1, 0);
 lcd.print("Please-Wait....");
 processGPS();
 if (GPS.fix) {
 lcd.setCursor(0, 0);
 lcd.print("GPS-Aquried......");
 if (irrecv.decode(&results)) // have we received an IR signal?
 {
 lcd.setCursor(0, 0);
 lcd.print("Engaging-IR....");
 lcd.setCursor(1, 0);
 lcd.print("Please-Wait..");
 translateIR();
 irrecv.resume(); // receive the next value
 }
 else{
 lcd.setCursor(0, 0);
 lcd.print("Enaging-AutoCont");
 lcd.setCursor(1, 0);
 lcd.print("Starting-Navgt..");
 // navigate
 currentHeading = readCompass(); // get our current heading
BIT LALPUR Ext. CENTER | Batch 2014-17 K-Vehicle | 64
Written Report
 calcDesiredTurn(); // calculate how we would optimatally turn, without
regard to obstacles

 checkSonar();
 moveAndAvoid();
 updatelcd();

 }
}
}
/////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////Update-Lcd//////////////////////////////////////////////////
void updatelcd() {
 // set the cursor to column 0, line 1
 // (note: line 1 is the second row, since counting begins with 0):
 lcd.setCursor(0, 0);
 lcd.print("TaD:");
 lcd.setCursor(0, 4);
 lcd.print(distanceToTarget);
 lcd.setCursor(0, 7);
 lcd.print(" ");
 lcd.setCursor(0, 8);
 lcd.print("TrH:");
 lcd.setCursor(0, 12);
 lcd.print(currentHeading);
 lcd.setCursor(1, 0);
 lcd.print("TrD:");
 lcd.setCursor(1, 4);
 lcd.print(turnDirection);
 lcd.setCursor(1, 7);
 lcd.print(" ");
 lcd.setCursor(1, 8);
 lcd.print("ObD:");
 lcd.setCursor(1, 12);
 lcd.print(sonarDistance);
}
/////////////////////////////////////////////////////////////////////////////////////////////////
//BIT LALPUR Ext. CENTER | Batch 2014-17 K-Vehicle | 65
//Written Report
///////////////////////////////////////////////Function-For-IRRemote///////////////////////////////////////////////
/*-----( Function )-----*/
void translateIR() // takes action based on IR code received
// describing Remote IR codes
{
 switch(results.value)
 {
 case 0xFF629D: Serial.println(" FORWARD"); motor.run(FORWARD);
 motork.run( straight);
 break;
 case 0xFF22DD: Serial.println(" LEFT"); motor.run(FORWARD);
 motork.run(left);
 break;
 case 0xFF02FD: Serial.println(" -OK-");
 break;
 case 0xFFC23D: Serial.println(" RIGHT"); motor.run(FORWARD);
 motork.run(right);
 break;
 case 0xFFA857: Serial.println(" REVERSE"); motor.run(BACKWARD);
 motork.run( straight);
 break;
 default:
 Serial.println(" other button ");
 }// End Case
 delay(500); // Do not get immediate repeat
}
/////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////GPS MODULE////////////////////////////////////
//Function for gps parsing
void processGPS(void)
{
BIT LALPUR Ext. CENTER | Batch 2014-17 K-Vehicle | 66
Written Report
 // in case you are not using the interrupt above, you'll
 // need to 'hand query' the GPS, not suggested :(

 if (! usingInterrupt) {
 // read data from the GPS in the 'main loop'

 char c = GPS.read();
 // if you want to debug, this is a good time to do it!
 if (GPSECHO)
 if (c) Serial.print(c);
 }

 // if a sentence is received, we can check the checksum, parse it...
 if (GPS.newNMEAreceived()) {

 // a tricky thing here is if we print the NMEA sentence, or data
 // we end up not listening and catching other sentences!
 // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
 //Serial.println(GPS.lastNMEA()); // this also sets the newNMEAreceived()
flag to false

 if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived()
flag to false
 return; // we can fail to parse a sentence in which case we should just wait for
another
 }
 // if millis() or timer wraps around, we'll just reset it
 if (timer > millis()) timer = millis();
 // approximately every 2 seconds or so, print out the current stats
 if (millis() - timer > 2000) {
 timer = millis(); // reset the timer


 Serial.print("Fix: "); Serial.print((int)GPS.fix);
 Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
 if (GPS.fix) {
 Serial.print("Location (in degrees, works with Google Maps): ");
 Serial.print(GPS.latitudeDegrees, 4);
BIT LALPUR Ext. CENTER | Batch 2014-17 K-Vehicle | 67
Written Report
 Serial.print(", ");
 Serial.println(GPS.longitudeDegrees, 4);
 Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
 currentLat = GPS.latitudeDegrees;
 currentLong = GPS.longitudeDegrees;

 if (GPS.lat == 'S') // make them signed
 currentLat = -currentLat;
 if (GPS.lon == 'W')
 currentLong = -currentLong;

 // update the course and distance to waypoint based on our new position
 distanceToWaypoint();
 courseToWaypoint();
 Serial.print(" \n Current Latitude ");
 Serial.print(currentLat );
 Serial.print(", "); Serial.print(", \n Current Longtitude ");
 Serial.print(currentLong );
 Serial.print(", \n");
 }
 }
}
/* converts lat/long from Adafruit degree-minute format to decimal-degrees;
requires <math.h> library
double convertDegMinToDecDeg (float degMin)
{
 double min = 0.0;
 double decDeg = 0.0;
 //get the minutes, fmod() requires double
 min = fmod((double)degMin, 100.0);
 //rebuild coordinates in decimal degrees
 degMin = (int) ( degMin / 100 );
 decDeg = degMin + ( min / 60 );
 return decDeg;
}*/
BIT LALPUR Ext. CENTER | Batch 2014-17 K-Vehicle | 68
Written Report
void nextWaypoint(void)
{
 waypointNumber++;
 targetLat = waypointList[waypointNumber].getLong();
 targetLong = waypointList[waypointNumber].getLat();

 if ((targetLat == 0 && targetLong == 0) || waypointNumber >=
NUMBER_WAYPOINTS) // last waypoint reached?
 {
 motor.run(RELEASE); // make sure we stop
 motork.run(RELEASE);
 Serial.println("* LAST WAYPOINT *");
 loopForever();
 }

 processGPS();
 distanceToTarget = originalDistanceToTarget = distanceToWaypoint();
 courseToWaypoint();

} // nextWaypoint()
// returns distance in meters between two positions, both specified
// as signed decimal-degrees latitude and longitude. Uses great-circle
// distance computation for hypothetical sphere of radius 6372795 meters.
// Because Earth is no exact sphere, rounding errors may be up to 0.5%.
// copied from TinyGPS library
int distanceToWaypoint()
{

 float delta = radians(currentLong - targetLong);
 float sdlong = sin(delta);
 float cdlong = cos(delta);
 float lat1 = radians(currentLat);
 float lat2 = radians(targetLat);
 float slat1 = sin(lat1);
 float clat1 = cos(lat1);
BIT LALPUR Ext. CENTER | Batch 2014-17 K-Vehicle | 69
Written Report
 float slat2 = sin(lat2);
 float clat2 = cos(lat2);
 delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
 delta = sq(delta);
 delta += sq(clat2 * sdlong);
 delta = sqrt(delta);
 float denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
 delta = atan2(delta, denom);
 distanceToTarget = delta * 6372795;

 // check to see if we have reached the current waypoint
 if (distanceToTarget <= WAYPOINT_DIST_TOLERANE)
 nextWaypoint();
 Serial.print(", \n Printing From DistanceToWayPt. ");
 Serial.print(distanceToTarget);

 return distanceToTarget;
} // distanceToWaypoint()
// returns course in degrees (North=0, West=270) from position 1 to position 2,
// both specified as signed decimal-degrees latitude and longitude.
// Because Earth is no exact sphere, calculated course may be off by a tiny fraction.
// copied from TinyGPS library
int courseToWaypoint()
{
 float dlon = radians(targetLong-currentLong);
 float cLat = radians(currentLat);
 float tLat = radians(targetLat);
 float a1 = sin(dlon) * cos(tLat);
 float a2 = sin(cLat) * cos(tLat) * cos(dlon);
 a2 = cos(cLat) * sin(tLat) - a2;
 a2 = atan2(a1, a2);
 if (a2 < 0.0)
 {
 a2 += TWO_PI;
 }
 Serial.print(", \n Printing From CourseToWayPt. ");
BIT LALPUR Ext. CENTER | Batch 2014-17 K-Vehicle | 70
Written Report
 Serial.print(degrees(a2));

 targetHeading = degrees(a2);
 return targetHeading;
} // courseToWaypoint()
////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////Module For Compass///////////////////////////////////////////////////
// Output the data down the serial port.
void Output(MagnetometerRaw raw, MagnetometerScaled scaled, float heading,
float headingDegrees)
{
 Serial.print(" \tHeading:\t");
 Serial.print(heading);
 Serial.print(" Radians \t");
 Serial.print(headingDegrees);
 Serial.println(" Degrees \t");
}
int readCompass(void)
{
 // Retrive the raw values from the compass (not scaled).
 MagnetometerRaw raw = compass.ReadRawAxis();
 // Retrived the scaled values from the compass (scaled to the configured scale).
 MagnetometerScaled scaled = compass.ReadScaledAxis();

 // Values are accessed like so:
 int MilliGauss_OnThe_XAxis = scaled.XAxis;// (or YAxis, or ZAxis)
 // Calculate heading when the magnetometer is level, then correct for signs of
axis.
 float heading = atan2(scaled.YAxis, scaled.XAxis);

 // Once you have your heading, you must then add your 'Declination Angle',
which is the 'Error' of the magnetic field in your location.
 // Find yours here: http://www.magnetic-declination.com/
BIT LALPUR Ext. CENTER | Batch 2014-17 K-Vehicle | 71
Written Report
 // Mine is: 2ï¿½? 37' W, which is 2.617 Degrees, or (which we need)
0.0456752665 radians, I will use 0.0457
 // If you cannot find your Declination, comment out these two lines, your
compass will be slightly off.
 float declinationAngle = 0.0005585;
 heading += declinationAngle;

 // Correct for when signs are reversed.
 if(heading < 0)
 heading += 2*PI;

 // Check for wrap due to addition of declination.
 if(heading > 2*PI)
 heading -= 2*PI;

 // Convert radians to degrees for readability.
 float headingDegrees = heading * 180/M_PI;
 return ((int)headingDegrees);
 // Output the data via the serial port.
 Output(raw, scaled, heading, headingDegrees);
 // Normally we would delay the application by 66ms to allow the loop
 // to run at 15Hz (default bandwidth for the HMC5883L).
 // However since we have a long serial out (104ms at 9600) we will let
 // it run at its natural speed.
 // delay(66);m m

}
void calcDesiredTurn(void)
{
 // calculate where we need to turn to head to destination
 headingError = targetHeading - currentHeading;

 // adjust for compass wrap
 if (headingError < -180)
 headingError += 360;
BIT LALPUR Ext. CENTER | Batch 2014-17 K-Vehicle | 72
Written Report
 if (headingError > 180)
 headingError -= 360;

 // calculate which way to turn to intercept the targetHeading
 if (abs(headingError) <= HEADING_TOLERANCE){ // if within tolerance,
don't turn
 turnDirection = straight;
 Serial.print(" Turning Straight "); }
 else if (headingError < 0){
 Serial.print(" Turning Left ");
 turnDirection = left;}
 else if (headingError > 0){
 Serial.print(" Turning Right ");
 turnDirection = right;}
 else{
 Serial.print(" Going Straight ");
 turnDirection = straight;
 }
} // calcDesiredTurn()
/////////////////////////////////////////////////////-------------------------------------------///////////////////////////////////////////
////////////////////////////////////////////////////// Check Sonar & Obstacle AviodanceSystem/////////////////////////////////////////////////////////////////////////
void checkSonar(void)
{
 delay(50); // Wait 50ms between pings (about 20 pings/sec). 29ms
should be the shortest delay between pings.
 Serial.print("Ping: ");
 Serial.print(sonar.ping_cm()); // Send ping, get distance in cm and print result (0
= outside set distance range)
 Serial.println("cm");
 sonarDistance = sonar.ping_cm();

}
void checkTurn (void)
BIT LALPUR Ext. CENTER | Batch 2014-17 K-Vehicle | 73
Written Report
{
if (turnDirection == right)
{
if (SR >= SSD)
 {
exit;}
else{
motork.run(left);}
}
if (turnDirection == left)
{
if (SL >= SSD)
{ exit;}
else
{motork.run(right);}
}
}
void moveAndAvoid(void)
{

 if (sonarDistance >= SAFE_DISTANCE) // no close objects in front of car
 {
 if (turnDirection == straight)
{checkStraight ();
 speed = FAST_SPEED;
} else
 speed = TURN_SPEED;
 motor.setSpeed(speed);
 motor.run(FORWARD);
checkTurn ();
 motork.run(turnDirection);
 Serial.print("\nMoving Forward");
 return;
 }
if (sonarDistance > TURN_DISTANCE && sonarDistance < SAFE_DISTANCE)
// not yet time to turn, but slow down
 {
BIT LALPUR Ext. CENTER | Batch 2014-17 K-Vehicle | 74
Written Report
 if (turnDirection == straight)
{ checkStraight ();
 speed = NORMAL_SPEED;
} else
 {
checkTurn ();
 speed = TURN_SPEED;
 motork.run(turnDirection); // alraedy turning to navigate
 }
 motor.setSpeed(speed);
 motor.run(FORWARD);
 Serial.print("\nSlowing Down & Moving Forward");
 return;
 }

 if (sonarDistance < TURN_DISTANCE && sonarDistance >
STOP_DISTANCE) // getting close, time to turn to avoid object
 {
 speed = SLOW_SPEED;
 motor.setSpeed(speed); // slow down
 motor.run(FORWARD);
 switch (turnDirection)
 {
 case straight: // going straight currently, so start new turn
 {
checkStraight ();
 if (headingError <= 0)
 turnDirection = left;
 else
 turnDirection = right;
 motork.run(turnDirection); // turn in the new direction
 break;
 }
 case left: // if already turning left, try right
 {
checkTurn ();
 motork.run(TURN_RIGHT);
 break;
 }
 case right: // if already turning right, try left
BIT LALPUR Ext. CENTER | Batch 2014-17 K-Vehicle | 75
Written Report
 {
checkTurn ();
 motork.run(TURN_LEFT);
 break;
 }
 } // end SWITCH

 return;
 }

 if (sonarDistance < STOP_DISTANCE) // too close, stop and back up
 {
 motor.run(RELEASE); // stop
 motork.run(RELEASE); // straighten up
 turnDirection = straight;
 motor.setSpeed(NORMAL_SPEED); // go back at higher speet
if (BckDist >= SAFE_DISTANCE)
 {
motor.run(BACKWARD);
}
else
{
motor.run(RELEASE);
}
 while (sonarDistance < TURN_DISTANCE) // backup until we get safe
clearance
 {

 processGPS();
 currentHeading = readCompass(); // get our current heading
 calcDesiredTurn(); // calculate how we would optimatally turn,
without regard to obstacles
 checkSonar();

 delay(100);
 } // while (sonarDistance < TURN_DISTANCE)
 motor.run(RELEASE);

 // stop backing up
BIT LALPUR Ext. CENTER | Batch 2014-17 K-Vehicle | 76
Written Report
 Serial.print("\nBreaking & Moving Backward");
 return;
 } // end of IF TOO CLOSE

} // moveAndAvoid()
void checkStraight (void)
{
 if(turnDirection == straight && SR >= SSD && SL >= SSD)
 {exit;}
 else if(turnDirection == straight && SR >= SSD && SL < SSD)
 {speed = TURN_SPEED;
 motor.run(right);}
 else if(turnDirection == straight && SR < SSD && SL >= SSD)
 { speed = TURN_SPEED;
 motor.run(left);}
 else if(turnDirection == straight && SR < SSD && SL < SSD)
{
 motor.run(RELEASE); // stop
 motork.run(RELEASE); // straighten up
 turnDirection = straight;
 motor.setSpeed(NORMAL_SPEED); // go back at higher speet
 if (BckDist >= SAFE_DISTANCE)
 {
 motor.run(BACKWARD);
 }
 else
 {
 motor.run(RELEASE);
 }
 }
}
// end of program routine, loops forever
void loopForever(void)
{
 while (1)
 ;
}
BIT LALPUR Ext. CENTER | Batch 2014-17 K-Vehicle | 77
Written Report
//////////////////////////////////---------------End Of Code ------------/////////////////////////////////
