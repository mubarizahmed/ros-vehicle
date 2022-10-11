#include <Servo.h>        // Include Servo Library
#include <NewPing.h>      // Include Newping Library

// L298N Control Pins
const int LeftMotorPWM = 6;
const int LeftMotorForward = 7;
const int LeftMotorBackward = 5;
const int RightMotorPWM = 3;
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
void go( enum Motor m, int speed){
  digitalWrite (m == LEFT ? LeftMotorForward : RightMotorForward, speed > 0 ? HIGH : LOW );
  digitalWrite (m == LEFT ? LeftMotorBackward : RightMotorBackward, speed <= 0 ? HIGH : LOW );
  analogWrite(m == LEFT ? LeftMotorPWM : RightMotorPWM, speed < 0 ? -speed : speed );
};

void reverse(){
  go(LEFT, -255);
  go(RIGHT, -255);
};

int readPing()      // Read Ping Function for Ultrasonic Sensor.
{
  int cm = sonar.ping_cm();   //Send ping, get ping distance in centimeters (cm).
  if (cm==0)
  {
    cm=250;
  }
  return cm;
};

void setup()
{
  // Set L298N Control Pins as Output
  pinMode(RightMotorForward, OUTPUT);
  pinMode(LeftMotorForward, OUTPUT);
  pinMode(LeftMotorBackward, OUTPUT);
  pinMode(RightMotorBackward, OUTPUT);
  pinMode(LeftMotorPWM,OUTPUT);
  pinMode(RightMotorPWM,OUTPUT);    
  
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
};


void loop()
{  
  int angle[] = {0,30,60,90,120,150,180};
  int farthest_dist = 0;
  int farthest_angle = 90;

  for (int i = 0; i < 8; i++){
    servo_motor.write(angle[i]);
    int dist = readPing();
    delay(200);
    if (dist > farthest_dist) {
      farthest_dist = dist;
      farthest_angle = angle[i];
    }    
  }

  if (farthest_dist < 20) {
    reverse();
  } else {
    drive(farthest_angle);
  }

};

void drive( int drive_angle){
  
  if ((drive_angle-90) < 0){
    //turn left
    go(RIGHT,255);
    go(LEFT,(1-(90-drive_angle)/90)*255);
  } else if ((drive_angle-90) > 0){
    //turn right
    go(LEFT,255);
    go(RIGHT,(1-(drive_angle-90)/90)*255);
  } else {
    //go straight
    go(RIGHT,255);
    go(LEFT,255);
  }
}


