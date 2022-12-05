#include <AccelStepper.h>
#include <MultiStepper.h>
#include <ros.h>
#include <Servo.h> 
#include <rafyu6dof_moveit/ArmJointState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <math.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt16.h>  
#include <std_msgs/Float64.h>

#define JOINT1_STEP_PIN        53
#define JOINT1_DIR_PIN         52

#define JOINT2_STEP_PIN        43
#define JOINT2_DIR_PIN         42

#define JOINT3_STEP_PIN        51
#define JOINT3_DIR_PIN         50

#define JOINT4_STEP_PIN        49
#define JOINT4_DIR_PIN         48

#define JOINT5_STEP_PIN        47
#define JOINT5_DIR_PIN         46

#define JOINT6_STEP_PIN        45
#define JOINT6_DIR_PIN         44

AccelStepper joint1   (1, JOINT1_STEP_PIN, JOINT1_DIR_PIN);
AccelStepper joint2   (1, JOINT2_STEP_PIN, JOINT2_DIR_PIN);
AccelStepper joint3   (1, JOINT3_STEP_PIN, JOINT3_DIR_PIN);
AccelStepper joint4   (1, JOINT4_STEP_PIN, JOINT4_DIR_PIN);
AccelStepper joint5   (1, JOINT5_STEP_PIN, JOINT5_DIR_PIN);
AccelStepper joint6   (1, JOINT6_STEP_PIN, JOINT6_DIR_PIN);

Servo gripper; 
int pos_gripper = 0;
int algo = 0;
MultiStepper steppers; 

int joint_step[7];//[joint1,joint2,joint3,joint4,joint5,joint6,servo]
int joint_status = 0;
int pos = 0;
int eff0 = 0; // Gripper close
int eff1 = 0; // Gripper open

float Sensibility = 0.185;

ros::NodeHandle nh; // Declaration of the NodeHandle with the nh instance
std_msgs::Int16 msg;
std_msgs::Float64 test;


void arm_cb(const rafyu6dof_moveit::ArmJointState& arm_steps){
  joint_status = 1;
  joint_step[0] = arm_steps.position1;
  joint_step[1] = arm_steps.position2;
  joint_step[2] = arm_steps.position3;
  joint_step[3] = arm_steps.position4;
  joint_step[4] = arm_steps.position5;
  joint_step[5] = arm_steps.position6;
  joint_step[6] = arm_steps.position7; // Gripper position <0-89>
}

void gripper_cb( const std_msgs::UInt16& cmd_msg){
  //gripper.write(msg_angulo.data);
   
  if(cmd_msg.data > 0)
  {
    for(pos = 0; pos < 90; pos += 1)   // Goes from 0 to 89° In steps of 1 degree
    {                                   
      gripper.write(pos);              // Tell the servo to acquire the pos variable 
      delay(5);                        // Wait 5ms for the servo to reach the position 
    }
  }
  
  if(cmd_msg.data == 0)
  {
    for(pos = 90; pos>=1; pos-=1)      // Go from 89 to 0° 
    {                                
      gripper.write(pos);              // Tell the servo to acquire the pos variable
      delay(5);                        // Wait 5ms for the servo to reach the position
    }
  }
}

/*------------------definition of subscriber objects------------------*/
// arm_cb function is executed when there is a message in the topic joint_steps
ros::Subscriber<rafyu6dof_moveit::ArmJointState> arm_sub("joint_steps",arm_cb);

//arm_cb function is executed when there is a message in the topic gripper_angle
ros::Subscriber<std_msgs::UInt16> gripper_sub("gripper_angle", gripper_cb); 

void setup() {
  //Serial.begin(57600);
  joint_status = 1;

  // node initialization to use serial port communication
  nh.initNode(); 
  
  // Initialize subscribers
  nh.subscribe(arm_sub); 
  nh.subscribe(gripper_sub);

  // Assignment of maximum speed value for each motor
  joint1.setMaxSpeed(500);
  joint2.setMaxSpeed(500);
  joint3.setMaxSpeed(500);
  joint4.setMaxSpeed(500);
  joint5.setMaxSpeed(500);
  joint6.setMaxSpeed(500);

  // Add motors to the MultiStepper library
  steppers.addStepper(joint1);
  steppers.addStepper(joint2);
  steppers.addStepper(joint3);
  steppers.addStepper(joint4);
  steppers.addStepper(joint5);
  steppers.addStepper(joint6);

  // Assign PWM port 4 to the Gripper
  gripper.attach(7);

}

void loop() {
  if (joint_status == 1){ // If arm_cb is being called set the joint_state to 1
    
    long positions[7];

    positions[0] = -joint_step[0];
    positions[1] = joint_step[1];
    positions[2] = -joint_step[2];
    positions[3] = -joint_step[3];
    positions[4] = -joint_step[4];
    positions[5] = -joint_step[5];

    steppers.moveTo(positions);
    nh.spinOnce();
    steppers.runSpeedToPosition();
    
    if(joint_step[6] > 0){
      if(eff1 == 0){
        for(pos = 5; pos < 95; pos += 1){  // Go from 0 to 89° In steps of 1 degree                                   
          gripper.write(pos);              // Tell the servo to acquire the pos variable 
          delay(5);                        // Wait 5ms for the servo to reach the position 
        }        
      }
      eff1++;
      eff0 = 0;
    }

    if(joint_step[6] == 0){
      if(eff0 == 0){
        for(pos = 95; pos>=5; pos-=1){     // Go from 89 to 0°                               
          gripper.write(pos);              // Tell the servo to acquire the pos variable
          delay(5);                        // Wait 5ms for the servo to reach the position
        }
      }
      eff0++;
      eff1 = 0;
    }    
  }
  joint_status = 0;

  nh.spinOnce();
  delay(1);
}
