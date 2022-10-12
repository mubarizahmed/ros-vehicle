#include <math.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

// #define LED_BUILTIN 2 // Remapping the built-in LED since the NodeMcu apparently uses a different one.
// #define LED_BUILTIN_RED 16 // If using a NodeMcu v1, then there's another red onboard led.
// The min amount of PWM the motors need to move. Depends on the battery, motors and controller.
// The max amount is defined by PWMRANGE in Arduino.h
#define PWM_MIN 0
#define PWMRANGE 255

// Declare functions
void setupPins();
void setupSerial();
// bool rosConnected();
void onTwist(const geometry_msgs::Twist &msg);
float mapPwm(float x, float out_min, float out_max);

// Pins
const uint8_t R_PWM = 3;
const uint8_t R_BACK = 4;
const uint8_t R_FORW = 2;
const uint8_t L_BACK = 5;
const uint8_t L_FORW = 7;
const uint8_t L_PWM = 6;


// ROS serial server
ros::NodeHandle node;
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &onTwist);

bool _connected = false;

void setup()
{
  setupPins();
  //setupSerial();
  
  // Connect to rosserial socket server and init node. (Using default port of 11411)
  node.getHardware()->setBaud(57600);
  node.initNode();
  node.subscribe(sub);

  // node.logerror(printf ("Connected: %s", node.connected() ? "true":"false"));

  node.logerror("log: Subscribed");  
}

void setupPins()
{
  // Status LED
  // pinMode(LED_BUILTIN, OUTPUT);
  // digitalWrite(LED_BUILTIN, HIGH);

  pinMode(L_PWM, OUTPUT);
  pinMode(L_FORW, OUTPUT);
  pinMode(L_BACK, OUTPUT);
  pinMode(R_PWM, OUTPUT);
  pinMode(R_FORW, OUTPUT);
  pinMode(R_BACK, OUTPUT);
  stop();
}

void setupSerial()
{
  Serial.begin(57600);
  Serial.println("arduino_vehicle - Serial Connected");
}


void stop()
{
  // node.logerror("72: stopped");
  digitalWrite(L_FORW, 0);
  digitalWrite(L_BACK, 0);
  digitalWrite(R_FORW, 0);
  digitalWrite(R_BACK, 0);
  analogWrite(L_PWM, 0);
  analogWrite(R_PWM, 0);
}

void onTwist(const geometry_msgs::Twist& msg)
{ 
  // if (!_connected){
  //   stop();
  //   node.logerror("ln81: Not connected");
  //   return;
  // }
  
  // Cap values at [-1 .. 1]
  float x = max(min(msg.linear.x, 1.0f), -1.0f);
  float z = max(min(msg.angular.z, 1.0f), -1.0f);

  // Calculate the intensity of left and right wheels. Simple version.
  // Taken from https://hackernoon.com/unicycle-to-differential-drive-courseras-control-of-mobile-robots-with-ros-and-rosbots-part-2-6d27d15f2010#1e59
  float l = (msg.linear.x - msg.angular.z) / 2;
  float r = (msg.linear.x + msg.angular.z) / 2;
  // node.logerror(printf("l = %f | r = %f", l, r));
  // Then map those values to PWM intensities. PWMRANGE = full speed, while PWM_MIN = the minimal amount of power at which the motors begin moving.
  uint16_t lPwm = mapPwm(fabs(l), PWM_MIN, PWMRANGE);
  uint16_t rPwm = mapPwm(fabs(r), PWM_MIN, PWMRANGE);

  // Set direction pins and PWM
  digitalWrite(L_FORW, l > 0);
  digitalWrite(L_BACK, l < 0);
  digitalWrite(R_FORW, r > 0);
  digitalWrite(R_BACK, r < 0);
  analogWrite(L_PWM, lPwm);
  analogWrite(R_PWM, rPwm);
}

void loop()
{
  // if (!rosConnected())
  //   stop();
  node.spinOnce();
  delay(10);
}

bool rosConnected()
{
  // If value changes, notify via LED and console.
  bool connected = node.connected();
  if (_connected != connected)
  {
    _connected = connected;
    // node.logerror(connected ? "ROS connected" : "ROS disconnected")   ; 
    // digitalWrite(LED_BUILTIN, !connected); // false -> on, true -> off
    Serial.println(connected ? "ROS connected" : "ROS disconnected");
  }
  return connected;
}

// Map x value from [0 .. 1] to [out_min .. out_max]
float mapPwm(float x, float out_min, float out_max)
{
  float y = x * (out_max - out_min) + out_min;
  // node.logerror(printf(" Max pwm: %f", y))  ;
  return y;
}