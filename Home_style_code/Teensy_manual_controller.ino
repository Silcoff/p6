#include <Stepper.h>

int joint = 1;
float spd[] = {20,10,10,40,20,40};
float spdDelay[] = {500/(spd[0]/100),500/(spd[1]/100),500/(spd[2]/100),500/(spd[3]/100),500/(spd[4]/100),500/(spd[5]/100)};

int stepsPerRevolution[] = {4000,20000,20000,6400,800,3841};
float stepsPerDeg[] = {47.0889, 52.6667, 59.5357, 44.8889, 21.0466, 21.3333};
int pul[] = {0,2,4,6,8,10};
int dir[] = {1,3,5,7,9,11};
int sw[] = {26,27,28,32,30,31};
bool home[] = {1,1,0,1,1,0};
int homeSteps[] = {-8686, -2508, 3727, -7897, -2178, 3580};

long total = 0;
float distDelay = 60;
float minSpeedDelay = 500;
float calcStepGap = 0;

char nl = 10;

bool moveDeg = false;

// Stepper myStepper(stepsPerRevolution[joint-1], pul[joint-1],dir[joint-1]);

Stepper Step1(stepsPerRevolution[0], pul[0],dir[0]);
Stepper Step2(stepsPerRevolution[1], pul[1],dir[1]);
Stepper Step3(stepsPerRevolution[2], pul[2],dir[2]);
Stepper Step4(stepsPerRevolution[3], pul[3],dir[3]);
Stepper Step5(stepsPerRevolution[4], pul[4],dir[4]);
Stepper Step6(stepsPerRevolution[5], pul[5],dir[5]);

Stepper Step[] = {Step1,Step2,Step3,Step4,Step5,Step6};

void setup() {

Step[0].setSpeed(spd[0]); //speed is in RPM;
Serial.begin(9600);
Serial.setTimeout(20);
Serial.println("Joint " + String(joint) + ", at pins " + String(pul[joint-1]) + ", " + String(dir[joint-1]));

}

void loop() {
  if (Serial.available()){
    delay(2);
    String msg = Serial.readString();
    if (msg[0] == 'J' || msg[0] == 'j'){
      msg = msg[1];
      joint = msg.toInt();
      Serial.println("Joint " + String(joint) + ", at pins " + String(pul[joint-1]) + ", " + String(dir[joint-1]));
    } else if (msg[0] == 'M'|| msg[0] == 'm'){
      moveDeg = !(moveDeg);
      if(moveDeg){
        Serial.println("Moving in degrees");
      } else {
        Serial.println("Moving in steps");
      }
    } else if (msg[0] == 'H'|| msg[0] == 'h'){

      Serial.println("Homing");
      for (int run = 0; run<2; run++){
        for (int i = 0; i<6; i++){
          joint = i+1;
          if (home[joint-1]){
            digitalWrite(dir[joint-1], LOW);
          } else {
            digitalWrite(dir[joint-1], HIGH);
          }
          while (1){
            if(digitalRead(sw[joint-1])){
              delay(20);
              if(digitalRead(sw[joint-1])){
                break;
              }
            }
            digitalWrite(pul[joint-1], LOW);
            delayMicroseconds(distDelay);
            digitalWrite(pul[joint-1], HIGH);
            delayMicroseconds(spdDelay[joint-1] - distDelay);
          }
          Serial.println("J" + String(joint) + " homed");
        }
        Serial.println("Re-checkJing");
      }
      Serial.println("Returning...");
      for (int i = 0; i<6; i++){
        joint = i+1;

        if (homeSteps[joint-1] > 0){
          digitalWrite(dir[joint-1], LOW);
        } else {
          digitalWrite(dir[joint-1], HIGH);
        }
        
        for (int i = 0; i <= abs(homeSteps[joint-1]); i++){
          digitalWrite(pul[joint-1], LOW);
          delayMicroseconds(distDelay);
          digitalWrite(pul[joint-1], HIGH);
          delayMicroseconds(spdDelay[joint-1] - distDelay);
        }
      }
      Serial.println("Done!");
    }else {
      float steps = msg.toFloat();
      if (steps == 0){
        total = 0;
        Serial.print("Total reset to ");
      } else{

        Serial.print("Moving " + String(steps) + "... ");

        if(moveDeg){
          steps = steps*stepsPerDeg[joint-1];
        }

        if (steps > 0){
          digitalWrite(dir[joint-1], LOW);
        } else {
          digitalWrite(dir[joint-1], HIGH);
        }
        
        for (int i = 0; i <= abs(steps); i++){
          digitalWrite(pul[joint-1], LOW);
          delayMicroseconds(distDelay);
          digitalWrite(pul[joint-1], HIGH);
          delayMicroseconds(spdDelay[joint-1] - distDelay);
        }
        Serial.println("Done!");
        total = total + steps;
      }
      Serial.println(total);
    }
  } else {
    delay(2);
  }
}
