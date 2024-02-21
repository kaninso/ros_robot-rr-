/*
 * rosserial Servo Control Example
 *
 * This sketch demonstrates the control of hobby R/C servos
 * using ROS and the arduiono
 * 
 * For the full tutorial write up, visit
 * www.ros.org/wiki/rosserial_arduino_demos
 *
 * For more information on the Arduino Servo Library
 * Checkout :
 * http://www.arduino.cc/en/Reference/Servo
 */

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h> 
#include <Encoder.h>
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>

ros::NodeHandle nh;

std_msgs::Bool switch_msg;
std_msgs::Int32 joint1_msg;
std_msgs::Int32 joint2_msg;

Servo servo1;
Servo servo2;

int Joint1gui=0;
int Joint2gui=0;

const int ENCODER_PIN_A = 2;
const int ENCODER_PIN_B = 3;
const int ENCODER_SW_PIN = 4; 

bool switch_state = false;
const int HOME_POSITION = 75; 
const int STEP_VALUE = 3;     
const int SERVO_PIN_1 = 9;    
const int SERVO_PIN_2 = 10;   

long oldPosition = -999;

const int POT_PIN = A0;
int servoAngle1 = HOME_POSITION;
int servoAngle2 = HOME_POSITION;
int mappedVal;

Encoder encoder(ENCODER_PIN_A, ENCODER_PIN_B);

void servo1_cb(const std_msgs::UInt16& cmd1_msg) {
  Joint1gui = cmd1_msg.data;
  if (switch_state) {
    servo1.write(cmd1_msg.data); //set servo angle, should be from 0-180  
    digitalWrite(13, HIGH - digitalRead(13));  //toggle led  
  }
}

void servo2_cb(const std_msgs::UInt16& cmd2_msg) {
  Joint2gui = cmd2_msg.data;
  if (switch_state) {
    servo2.write(cmd2_msg.data); 
    digitalWrite(13, HIGH - digitalRead(13)); 
  }
}

void switch_callback(const std_msgs::Bool& msg) {
  switch_state = msg.data;
  if (switch_state) {
    digitalWrite(13, HIGH);
  } else {
    digitalWrite(13, LOW);
  }
}

ros::Subscriber<std_msgs::UInt16> servo1_sub("servo1", servo1_cb);
ros::Subscriber<std_msgs::UInt16> servo2_sub("servo2", servo2_cb);
ros::Publisher Joint1_pub("positionJoint1", &joint1_msg);
ros::Publisher Joint2_pub("positionJoint2", &joint2_msg);
ros::Subscriber<std_msgs::Bool> switch_sub("mode", &switch_callback);

void setup() {
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(servo1_sub);
  nh.subscribe(servo2_sub);
  nh.subscribe(switch_sub);
  nh.advertise(Joint1_pub);
  nh.advertise(Joint2_pub);
  servo2.attach(SERVO_PIN_2);
  servo1.attach(SERVO_PIN_1); 
  pinMode(ENCODER_SW_PIN, INPUT_PULLUP);
}

void loop() {
  nh.spinOnce();
  
  if (!switch_state) {    
    MM_robot();
    joint1_msg.data = servoAngle1;
    joint2_msg.data = mappedVal;
  } else {
    joint1_msg.data = Joint1gui;
    joint2_msg.data = Joint2gui;
  }
  
  Joint1_pub.publish(&joint1_msg);
  Joint2_pub.publish(&joint2_msg);
  
  delay(1);
}

void MM_robot() {
  long newPosition = encoder.read();
  
  if (newPosition != oldPosition) {
    if (newPosition > oldPosition) {
      servoAngle1 += STEP_VALUE;
      if (servoAngle1 > 155)
        servoAngle1 = 155;
    } else {
      servoAngle1 -= STEP_VALUE;
      if (servoAngle1 < 0)
        servoAngle1 = 0;
    }
    servo1.write(servoAngle1);
    oldPosition = newPosition;
  }

  // Check if the switch is pressed to reset servo 1 position to home
  if (digitalRead(ENCODER_SW_PIN) == LOW) {
    servoAngle1 = HOME_POSITION;
    servo1.write(servoAngle1);
    delay(50); // Debouncing delay
  }

  // Control servo 2 with potentiometer
  int val = analogRead(POT_PIN); // Read the value of the potentiometer
  mappedVal = map(val, 0, 1023, 0, 175); // Map the potentiometer value to servo angle
  servo2.write(mappedVal); // Set servo 2 position based on potentiometer value
}
