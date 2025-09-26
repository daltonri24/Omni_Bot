#include "pio_encoder.h" //rp2040-encoder-library by Giovanni di Dio Bruno
#include "RP2040_PWM.h"
#include "PID_v1.h"


char cmd;
char arg0[8];
char arg1[8];
char arg2[8];


PioEncoder encoders[3] = {
  PioEncoder(16),
  PioEncoder(14),
  PioEncoder(4)
};


int pwmPINS[3] = {18, 11, 6};
RP2040_PWM motPWM[3] = {
  RP2040_PWM(pwmPINS[0], 10000, 0),
  RP2040_PWM(pwmPINS[1], 10000, 0),
  RP2040_PWM(pwmPINS[2], 10000, 0)
};


float motorForwardSlopes[3] = {0.01, 0.00942, 0.00979};
float motorForwardIntercepts[3] = {39, 37.1, 35.6};
float motorBackwardSlopes[3] = {0.00965, 0.00883, 0.00928};
float motorBackwardIntercepts[3] = {39.6, 35.6, 37.3};

int PID_Gap = 100;
int prevUpdate;
long PrevPID[3] = {0,0,0};

double PID_Setpoint[3] = {0,0,0};
double PID_Input[3] = {0,0,0};
double PID_Output[3] = {0,0,0};
long prevENC[3] = {0,0,0};

double Kp0=2, Ki0= 3, Kd0=0;
double Kp1=4, Ki1= 5, Kd1=0;
double Kp2=3, Ki2= 5, Kd2=0;

int motDirectionPINS[3][2] = {
  {19,20},
  {12,13},
  {7, 8}
};

PID pidController[3] = {
  PID(&PID_Input[0], &PID_Output[0], &PID_Setpoint[0], Kp0, Ki0, Kd0, DIRECT),
  PID(&PID_Input[1], &PID_Output[1], &PID_Setpoint[1], Kp1, Ki1, Kd1, DIRECT),
  PID(&PID_Input[2], &PID_Output[2], &PID_Setpoint[2], Kp2, Ki2, Kd2, DIRECT)
};

boolean newData = false;



#define pinPWM1 

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 10000);
  Serial.println("started");

  pinMode(motDirectionPINS[0][0], OUTPUT);
  digitalWrite(motDirectionPINS[0][0], LOW);
  pinMode(motDirectionPINS[0][1], OUTPUT);
  digitalWrite(motDirectionPINS[0][1], HIGH);

  pinMode(motDirectionPINS[1][0], OUTPUT);
  digitalWrite(motDirectionPINS[1][0], LOW);
  pinMode(motDirectionPINS[1][1], OUTPUT);
  digitalWrite(motDirectionPINS[1][1], HIGH);

  pinMode(motDirectionPINS[2][0], OUTPUT);
  digitalWrite(motDirectionPINS[2][0], LOW);
  pinMode(motDirectionPINS[2][1], OUTPUT);
  digitalWrite(motDirectionPINS[2][1], HIGH);

  encoders[0].begin();
  encoders[0].reset();
  prevENC[0] = encoders[0].getCount();
  pidController[0].SetMode(AUTOMATIC);
  pidController[0].SetOutputLimits(-1000, 1000);

  encoders[1].begin();
  encoders[1].reset();
  encoders[2].begin();
  encoders[2].reset();

  PrevPID[0] = millis();
  PrevPID[1] = millis();
  PrevPID[2] = millis();

  prevUpdate = millis();
}

void runCommand() {
  Serial.print(cmd);
  Serial.print(" ");
  Serial.print(atoi(arg0));
  Serial.print(" ");
  Serial.print(atoi(arg1));
  Serial.print(" ");
  Serial.print(atoi(arg2));
  Serial.print("\n");


  switch (cmd) {
      case 'e':
        Serial.print("E ");
        Serial.print(encoders[0].getCount());
        Serial.print(" ");
        Serial.print(encoders[1].getCount());
        Serial.print(" ");
        Serial.print(encoders[2].getCount());
        Serial.print("\n");
        break;
      case 'm':
        PID_Setpoint[0] = atoi(arg0);
        PID_Setpoint[1] = atoi(arg1);
        PID_Setpoint[2] = atoi(arg2);
        break;
  }

  newData = false; 
  memset(arg0, 0, sizeof(arg0));
  memset(arg1, 0, sizeof(arg1));
  memset(arg2, 0, sizeof(arg2));
}

void setMotor(int idx) {

  PID_Input[idx] = ((double)(encoders[idx].getCount() - prevENC[idx]) / (millis() - PrevPID[idx])) * 1000;
  pidController[idx].Compute();

  PrevPID[idx] = millis();
  prevENC[idx] = encoders[idx].getCount();

  float newSpeed = PID_Setpoint[idx] + PID_Output[idx];
  float motorDuty;

  if(newSpeed > 0) {
    digitalWrite(motDirectionPINS[idx][0], LOW);
    digitalWrite(motDirectionPINS[idx][1], HIGH);

    motorDuty = motorForwardSlopes[idx] * newSpeed + motorForwardIntercepts[idx];

  } else {
    digitalWrite(motDirectionPINS[idx][1], LOW);
    digitalWrite(motDirectionPINS[idx][0], HIGH);

    motorDuty = motorBackwardSlopes[idx] * (newSpeed * -1) + motorBackwardIntercepts[idx];

  }

  motPWM[idx].setPWM_Int(pwmPINS[idx], 10000, motorDuty * 1000);

}

void updatePID() {

  setMotor(0);
  setMotor(1);
  setMotor(2);

  Serial.print("Motor Speeds:");
  Serial.print(PID_Input[0]);
  Serial.print(" ");
  Serial.print(PID_Input[1]);
  Serial.print(" ");
  Serial.print(PID_Input[2]);
  Serial.print("\n");

}

void loop() {  // listen for commands
  static boolean recvInProgress = false;
  static byte idx = 0;
  static byte argNum = 0;

  char startMarker = '<';
  char endMarker = '>';
  char rc;
  
  while (Serial.available() > 0 && newData == false) {
      rc = Serial.read();

      if (recvInProgress == true) {
        if (rc == endMarker) {
          recvInProgress = false;
          idx = 0;
          argNum = 0;
          newData = true;
          runCommand();
        }
        else if (rc == ','){
          argNum++;
          idx = 0;
        }
        else{
          switch(argNum) {
            case 0:
              cmd = rc;
            case 1:
              arg0[idx % 8] = rc;
              idx++;
              break;
            case 2:
              arg1[idx % 8] = rc;
              idx++;
              break;

            case 3:
              arg2[idx % 8] = rc;
              idx++;
              break;

          }  
        }
      }
      else if (rc == startMarker) {
        recvInProgress = true;
      }
  }

  if(prevUpdate + PID_Gap < millis()){
    updatePID();
    prevUpdate = millis();
  }

}