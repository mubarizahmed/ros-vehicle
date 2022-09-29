#include <Servo.h>        // Include Servo Library
#include <NewPing.h>      // Include Newping Library

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

enum Motor { LEFT, RIGHT };

// Set motor speed: 255 full ahead, −255 full reverse , 0 stop
void go( enum Motor m, int speed)
{
digitalWrite (m == LEFT ? in1Pin : in3Pin, speed > 0 ? HIGH : LOW );
digitalWrite (m == LEFT ? in2Pin : in4Pin, speed <= 0 ? HIGH : LOW );
analogWrite(m == LEFT ? enAPin : enBPin, speed < 0 ? −speed : speed );
}

int readPing()      // Read Ping Function for Ultrasonic Sensor.
{
  int cm = sonar.ping_cm();   //Send ping, get ping distance in centimeters (cm).
  if (cm==0)
  {
    cm=250;
  }
  return cm;
}

void setup()
{
  // Set L298N Control Pins as Output
  pinMode(RightMotorForward, OUTPUT);
  pinMode(LeftMotorForward, OUTPUT);
  pinMode(LeftMotorBackward, OUTPUT);
  pinMode(RightMotorBackward, OUTPUT);
  
  servo_motor.attach(11);   // Attachs the servo on pin 9 to servo object.
  servo_motor.write(90);   // Set at 115 degrees. 
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
  int angle = {0,30,60,90,120,150,180};
  int farthest_dist = 0;
  int farthest_angle = 90;

  for (int i = 0; i < 7; i++){
    int dist = readPing();
        
  }
}




