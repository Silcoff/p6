#include <Stepper.h>

// This is a simple manual controller, mostly useful for debugging.
//
// The controller is able to move one joint at a time, with the 
// movement being described in either degrees on the joint, or by 
// motor steps directly. 
//
// Commands can also be sent as absolute angle to move to, or as 
// relative angles to move irregardless of the current position
// 
// The controller features a homing sequence, but the user must 
// ensure that the base joint is able to move freely before
// executing the homing sequence. This may require manually 
// moving the second joint up so the manipulator is no longer 
// touching the ground.
//
// Enjoy.
//
//  - Jonas Thorhauge


//Holds the active joint (1-6)
int joint = 1;

//Speed of each joint, in percent
float spd[] = {20,10,10,40,20,40};
//Pulse delay for each joint (depends on speed percentage)
float spdDelay[] = {500/(spd[0]/100),500/(spd[1]/100),500/(spd[2]/100),500/(spd[3]/100),500/(spd[4]/100),500/(spd[5]/100)};

//How many steps on the motor to turn the joint 1 deg
float stepsPerDeg[] = {47.0889, 52.6667, 59.5357, 44.8889, 21.0466, 21.3333};

//Pulse and direction pins for each motor controller
int pul[] = {0,2,4,6,8,10};
int dir[] = {1,3,5,7,9,11};
//Limit switch pin for each joint
int sw[] = {26,27,28,32,30,31};

//Which direction does the joint home in
bool home[] = {1,1,0,1,1,0};
//How many steps should the joint move after homing to return to home position
int homeSteps[] = {-8686, -2508, 3727, -7897, -2178, 3580};

//Positive and negative joint limits for each joint
int posLimit[] = {7063, 6319, 3727, 7180, 2178, 3580};
int negLimit[] = {-8686, -2508, -6547, -7897, -2178, -3642};

//Holds the total steps moved on each joints
long total[] = {0,0,0,0,0,0};

//Length of control pulse signal [microsecond]
float distDelay = 60;
//Minimum lenght of delay between pulse signals[microsecond]
float minSpeedDelay = 500;

//If true, move using degrees. If false, move using steps
bool moveDeg = false;
//If true, angles are given in absoulute angles. If false, angles are relative
bool moveAbs = false;

//This is not actually used, but is sets all the pinmodes correctly so i have not bothered to remove it and write them manually :)
Stepper Step1(stepsPerRevolution[0], pul[0],dir[0]);
Stepper Step2(stepsPerRevolution[1], pul[1],dir[1]);
Stepper Step3(stepsPerRevolution[2], pul[2],dir[2]);
Stepper Step4(stepsPerRevolution[3], pul[3],dir[3]);
Stepper Step5(stepsPerRevolution[4], pul[4],dir[4]);
Stepper Step6(stepsPerRevolution[5], pul[5],dir[5]);
Stepper Step[] = {Step1,Step2,Step3,Step4,Step5,Step6}; 

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(20);
  Serial.println("Joint " + String(joint) + ", at pins " + String(pul[joint-1]) + ", " + String(dir[joint-1]));
}

void loop() {
  //Loop constantly checks if a command is available
  if (Serial.available()){
    delay(2);
    String msg = Serial.readStringUntil(';');   //Read the command string until sepperating ';'
    if (msg[0] == 'J' || msg[0] == 'j'){        //Select active joint
      //This command sets the active joint to the one given by the number following the 'J'
      msg = msg[1];
      joint = msg.toInt();
      Serial.println("Joint " + String(joint) + ", at pins " + String(pul[joint-1]) + ", " + String(dir[joint-1]));
    } else if (msg[0] == 'M'|| msg[0] == 'm'){  //Control mode steps or degrees
      //Whenever the command is recieved, the controller switches between interpreting commands as degrees or motor steps
      moveDeg = !(moveDeg);
      if(moveDeg){
        Serial.println("Moving in DEGREES");
      } else {
        Serial.println("Moving in STEPS");
      }
    } else if (msg[0] == 'C'|| msg[0] == 'c'){  //Command type relative or absolute
      //Whenever the command is recieved, the controller switches between interpreting commands as absolute angles or relative angles
      moveAbs = !(moveAbs);
      if(moveAbs){
        Serial.println("Moving in ABSOLUTE angles");
      } else {
        Serial.println("Moving in RELATIVE angles");
      }
    } else if (msg[0] == 'Z'|| msg[0] == 'z'){  //Reset a joint counter for debugging
      //This command resets the step counter of the active joint. Only useful for debugging
      total[joint-1] = 0;
      Serial.println("JOINT " + String(joint) + " COUNTER RESET");
    } else if (msg[0] == 'H'|| msg[0] == 'h'){  //Home the robot
      //This command runs the homing sequence of the manipulator.
      Serial.println("Homing");

      //The homing sequence is run twice, in order to ensure that all joint are really at their end stops
      for (int run = 0; run<2; run++){

        //Each joint is homed in sequence
        for (int i = 0; i<6; i++){

          //set the active joint
          joint = i+1;

          //Set the direction of the homing procedure
          if (home[joint-1]){
            digitalWrite(dir[joint-1], LOW);
          } else {
            digitalWrite(dir[joint-1], HIGH);
          }

          //Runs until the joint is homed
          while (1){

            //Check and double-check, whether the end stop is pressed. If it is, break the loop.
            if(digitalRead(sw[joint-1])){
              delay(20);
              if(digitalRead(sw[joint-1])){
                break;
              }
            }

            //Otherwise, step the motor once
            digitalWrite(pul[joint-1], LOW);
            delayMicroseconds(distDelay);
            digitalWrite(pul[joint-1], HIGH);
            delayMicroseconds(spdDelay[joint-1] - distDelay);
          }
          Serial.println("J" + String(joint) + " homed");
        }
        Serial.println("Re-checking");
      }

      //Once all joint are homed move each joint to the home position
      Serial.println("Returning...");

      //Each joint is moved in sequence
      for (int i = 0; i<6; i++){

        //Set active joint
        joint = i+1;

        //Set the direction of the movement
        if (homeSteps[joint-1] > 0){
          digitalWrite(dir[joint-1], LOW);
        } else {
          digitalWrite(dir[joint-1], HIGH);
        }
        
        //Step the motor a number of times equal to the steps described in the homesteps[] array
        for (int i = 0; i <= abs(homeSteps[joint-1]); i++){
          digitalWrite(pul[joint-1], LOW);
          delayMicroseconds(distDelay);
          digitalWrite(pul[joint-1], HIGH);
          delayMicroseconds(spdDelay[joint-1] - distDelay);
        }
      }

      //Reset the step counter for each joint
      for (int i=0; i<6; i++){
        total[i] = 0;
      }
      Serial.println("Done!");

    } else if (msg[0] == 'S'|| msg[0] == 's'){  //Set the speed of a joint
      //This command sets the speed of the active joint and calculates the pulse delay
      msg = msg.substring(1);
      int tempSpd = msg.toInt();
      tempSpd = constrain(tempSpd,1,100);
      spd[joint-1] = tempSpd;
      spdDelay[joint-1] = 500/(spd[joint-1]/100);
      Serial.println("Joint " + String(joint) + " speed set to " + String(spd[joint-1]));
    } else {                                    //Move the joint
      //If no command type is given, the command is interpreted as a movement command
      float steps = msg.toFloat();
      
      //If giving commands in degrees, convert to motor steps
      if(moveDeg){
        steps = steps*stepsPerDeg[joint-1];
      }


      if (moveAbs) {
        //If moving in absolute angles...
        Serial.print("Moving to " + String(steps/stepsPerDeg[joint-1]) + "... ");

        //Limit the desired angle to be inbetween the positive and negative limits
        steps = constrain(steps, negLimit[joint-1], posLimit[joint-1]);

        //calculate the number of steps to move as the difference between the desired and the current position
        steps = steps - total[joint-1];

      } else {
        //If moving in relative angles...

        //Truncate the command angle to ensure it does not move the joint outside its limits
        if ((total[joint-1] + steps) > posLimit[joint-1]){
          steps = posLimit[joint-1] - total[joint-1];
        }
        if ((total[joint-1] + steps) < negLimit[joint-1]){
          steps = negLimit[joint-1] - total[joint-1];
        }
        Serial.print("Moving " + String(steps/stepsPerDeg[joint-1]) + "... ");
      }

      //Set the direction of the movement
      if (steps > 0){
        digitalWrite(dir[joint-1], LOW);
      } else {
        digitalWrite(dir[joint-1], HIGH);
      }

      //Step the motor a number of times based on the number of steps calculated for the desired movement
      for (int i = 0; i <= abs(steps); i++){
        digitalWrite(pul[joint-1], LOW);
        delayMicroseconds(distDelay);
        digitalWrite(pul[joint-1], HIGH);
        delayMicroseconds(spdDelay[joint-1] - distDelay);
      }
      
      //Confirm the completion of the movement and add the moved steps to the total
      Serial.println("Done!");
      total[joint-1] = total[joint-1] + steps;
      
      Serial.println("Joint at " + String(total[joint-1]/stepsPerDeg[joint-1]) + " deg");
    }
  } else {
    delay(2);
  }
}
