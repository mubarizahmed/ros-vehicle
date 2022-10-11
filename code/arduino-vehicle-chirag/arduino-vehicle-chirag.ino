#include <Servo.h>        // Include Servo Library
#include <NewPing.h>     // Include Newping Library

// L298N Control Pins
const int LeftMotorForward = 7;
const int LeftMotorBackward = 5;
const int RightMotorForward = 2;
const int RightMotorBackward = 4;

#define TRIGGER_PIN  A1  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     A2  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 250 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 250cm.

Servo servo_motor;  // Servo's name
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

boolean goesForward = false;
int distance = 100;



void setup()
{
  // Set L298N Control Pins as Output
  pinMode(RightMotorForward, OUTPUT);
  pinMode(LeftMotorForward, OUTPUT);
  pinMode(LeftMotorBackward, OUTPUT);
  pinMode(RightMotorBackward, OUTPUT);
  
  servo_motor.attach(11);   // Attachs the servo on pin 9 to servo object.
  servo_motor.write(90);   // Set at 90 degrees. 
  delay(2000);              // Wait for 2s.
  distance = readPing();    // Get Ping Distance.
  delay(100);               // Wait for 100ms.
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
}

void loop()
{  
  int distanceRight = 0;
  int distanceLeft = 0;
  delay(50);

  if (distance <= 20)
  {
    moveStop();
    delay(100);
    moveBackward();
    delay(100);
    moveStop();
    delay(100);
    distanceRight = lookRight();
    delay(100);
    distanceLeft = lookLeft();
    delay(100);

    if (distanceRight >= distanceLeft)
    {
      turnRight();
      delay(100);
      moveStop();
    }
    else
    {
      turnLeft();
      delay(100);
      moveStop();
    }
  
  }
  else
  {
    moveForward(); 
  }

    distance = readPing();
}

int lookRight()     // Look Right Function for Servo Motor
{  
  servo_motor.write(45);
  delay(100);
  int distance = readPing();
  delay(100);
  servo_motor.write(90);
  return distance;
}

int lookLeft()      // Look Left Function for Servo Motor 
{
  servo_motor.write(135);
  delay(100);
  int distance = readPing();
  delay(100);
  servo_motor.write(90);
  return distance;
}

int readPing()      // Read Ping Function for Ultrasonic Sensor.
{
  delay(100);                 // Wait 100ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
  int cm = sonar.ping_cm();   //Send ping, get ping distance in centimeters (cm).
  if (cm==0)
  {
    cm=250;
  }
  return cm;
}

void moveStop()       // Move Stop Function for Motor Driver.
{
  digitalWrite(RightMotorForward, LOW);
  digitalWrite(RightMotorBackward, LOW);
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(LeftMotorBackward, LOW);
}

void moveForward()    // Move Forward Function for Motor Driver.
{
    digitalWrite(RightMotorForward, HIGH);
    digitalWrite(RightMotorBackward, LOW);
    digitalWrite(LeftMotorForward, HIGH);
    digitalWrite(LeftMotorBackward, LOW);
}

void moveBackward()   // Move Backward Function for Motor Driver.
{
  digitalWrite(RightMotorForward, LOW);
  digitalWrite(RightMotorBackward, HIGH);
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(LeftMotorBackward, HIGH);
}

void turnRight()      // Turn Right Function for Motor Driver.
{
  digitalWrite(RightMotorForward, LOW);
  digitalWrite(RightMotorBackward, HIGH);
  digitalWrite(LeftMotorForward, HIGH);
  digitalWrite(LeftMotorBackward, LOW);
}

void turnLeft()       // Turn Left Function for Motor Driver.
{
  digitalWrite(RightMotorForward, HIGH);
  digitalWrite(RightMotorBackward, LOW);
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(LeftMotorBackward, HIGH);
}