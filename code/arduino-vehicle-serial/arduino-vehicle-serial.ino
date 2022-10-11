#include <math.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>


#define PWM_MIN 300

// Declare functions
void setupPins();
void setupSerial();
bool rosConnected();
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
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &onTwist);

bool _connected = false;

void setup()
{
  setupPins();
  setupSerial();
  
  // Connect to rosserial socket server and init node. (Using default port of 11411)
  //node.getHardware()->setConnection(server);
  node.initNode();
  node.subscribe(sub);
}

void setupPins()
{
  // Status LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

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
  Serial.println();
}


void stop()
{
  digitalWrite(L_FORW, 0);
  digitalWrite(L_BACK, 0);
  digitalWrite(R_FORW, 0);
  digitalWrite(R_BACK, 0);
  analogWrite(L_PWM, 0);
  analogWrite(R_PWM, 0);
}

void onTwist(const geometry_msgs::Twist &msg)
{
    // Cap values at [-1 .. 1]
  float x = max(min(msg.linear.x, 1.0f), -1.0f);
  float z = max(min(msg.angular.z, 1.0f), -1.0f);

  // Calculate the intensity of left and right wheels. Simple version.
  // Taken from https://hackernoon.com/unicycle-to-differential-drive-courseras-control-of-mobile-robots-with-ros-and-rosbots-part-2-6d27d15f2010#1e59
  float l = (msg.linear.x - msg.angular.z) / 2;
  float r = (msg.linear.x + msg.angular.z) / 2;

  // Then map those values to PWM intensities. PWMRANGE = full speed, while PWM_MIN = the minimal amount of power at which the motors begin moving.
  uint16_t lPwm = mapPwm(fabs(l), 0, 255*2);
  uint16_t rPwm = mapPwm(fabs(r), 0, 510);

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
   node.spinOnce();
}

// bool rosConnected()
// {
//   // If value changes, notify via LED and console.
//   bool connected = node.connected();
//   if (_connected != connected)
//   {
//     _connected = connected;
//     digitalWrite(LED_BUILTIN, !connected); // false -> on, true -> off
//     Serial.println(connected ? "ROS connected" : "ROS disconnected");
//   }
//   return connected;
// }

// Map x value from [0 .. 1] to [out_min .. out_max]
float mapPwm(float x, float out_min, float out_max)
{
  return x * (out_max - out_min) + out_min;
}