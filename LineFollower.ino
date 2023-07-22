#include <MeMegaPi.h>
#include <Thread.h>

MeEncoderOnBoard rightMotor(SLOT1);
MeEncoderOnBoard leftMotor(SLOT2);
MeLineFollower lineSensor(PORT_6);
MeMegaPiDCMotor crane(PORT3B);
MeMegaPiDCMotor claw(PORT4B);

int startTime = 0;

bool shouldMove = false;
bool followLine = false;
bool runCorrections = true;

void isr_process_encoder1(void)
{
  if(digitalRead(rightMotor.getPortB()) == 0)
  {
    rightMotor.pulsePosMinus();
  }
  else
  {
    rightMotor.pulsePosPlus();;
  }
}

void isr_process_encoder2(void)
{
  if(digitalRead(leftMotor.getPortB()) == 0)
  {
    leftMotor.pulsePosMinus();
  }
  else
  {
    leftMotor.pulsePosPlus();
  }
}

void setup()
{
  attachInterrupt(rightMotor.getIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(leftMotor.getIntNum(), isr_process_encoder2, RISING);
  Serial.begin(115200);
  
  //Set PWM 8KHz
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);

  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);

  rightMotor.setPulse(9);
  leftMotor.setPulse(9);
  rightMotor.setRatio(39.267);
  leftMotor.setRatio(39.267);
  rightMotor.setPosPid(1.8,0,1.2);
  leftMotor.setPosPid(1.8,0,1.2);
  rightMotor.setSpeedPid(0.18,0,0);
  leftMotor.setSpeedPid(0.18,0,0);

  startTime = millis();

  //fwd 9.5 - 10
  claw.run(-255);
  delay(1000);
  moveForward(14);
  delay(1000);
  claw.run(250);
  delay(1000);
  claw.run(0);
  delay(10);
  runCorrections = false;
  while(!(lineSensor.readSensors() == S1_IN_S2_IN)) {
    rightMotor.setMotorPwm(200);
    leftMotor.setMotorPwm(200);
  }
  rightMotor.setMotorPwm(0);
  leftMotor.setMotorPwm(0);
  delay(250);
  //moveForward(9.5);
  followLine = true;
  shouldMove = true;
  runCorrections = true;
  //traverse the line
  /*delay(10);
  claw.run(-250);
  delay(10);
  claw.run(0);
  delay(10);*/
  reset();
}

void loop()
{
  if(!shouldMove) { return; }
  updateOdom();
  updateFollower();
  updateHeading();
  updateMotorSpeeds();

  rightMotor.loop();
  leftMotor.loop();
}

float wheelDiameter = 2.5;
float wheelBase = 6.5;
float pi = 3.141592653589798;
float turnSpeed = 10.0; //multiplier for turning speed modifications
int driveSpeed = 200;
int realDriveSpeed = 200;
float realTurnSpeed = 6.0;

float leftDistance = 0;
float rightDistance = 0;
float headingRad = 0;
float heading = 0;
float generalDesiredHeading = 0; //the general direction the robot goes disregarding oscillations
float desiredHeading = 0; //the heading that the robot will always be trying to point to

int lmotorSpeed = 0;
int rmotorSpeed = 0;

int lmotorTurnSpeed = 0;
int rmotorTurnSpeed = 0;

float prevHeading = 0;
float prevLeftDistance = 0;
float prevRightDistance = 0;

int whiteTimer = 1000000000;

void reset() {
  prevHeading = heading;
  prevLeftDistance = leftDistance;
  prevRightDistance = rightDistance;
  desiredHeading = heading;
  //whiteTimer = millis();
}

float encoderToInches(int degrees) {
  return (degrees / 360.0f) * pi * wheelDiameter; //wheel revolutions * pi * wheel diameter
}

void updateOdom() {
  rightMotor.updateSpeed();
  leftMotor.updateSpeed();
  rightMotor.updateCurPos();
  leftMotor.updateCurPos();
  leftDistance = -encoderToInches(leftMotor.getCurPos()) - prevLeftDistance;
  rightDistance = encoderToInches(rightMotor.getCurPos()) - prevRightDistance;

  headingRad = (leftDistance - rightDistance) / wheelBase;
  heading = prevHeading - headingRad * (180.0f / pi);
}

void moveForward(float inches) { //SYNC NOT ASYNC!!!
  shouldMove = true;
  updateOdom();
  float prevLeftDistance = leftDistance;
  float prevRightDistance = rightDistance;
  Serial.print("distance: ");
  Serial.println(abs(leftDistance - prevLeftDistance));
  while(abs(leftDistance - prevLeftDistance) < inches && abs(rightDistance - prevRightDistance) < inches) {
    shouldMove = true;
    //rmotorTurnSpeed = abs(rightDistance - inches) * turnSpeed * 2;
    //lmotorTurnSpeed = abs(leftDistance - inches) * turnSpeed * 2;

    desiredHeading = heading;
    updateOdom();
    updateHeading();
    updateMotorSpeeds();
  }
  shouldMove = false;
  updateOdom();
  updateHeading();
  updateMotorSpeeds();
}

void rotate(float degrees) { //SYNC NOT ASYNC!!!
  rightMotor.reset(SLOT1);
  leftMotor.reset(SLOT2);
  updateOdom();
  //driveSpeed = 100;
  //turnSpeed = 6.0;
  float prevHeading = heading;
  while(abs(heading - prevHeading) < degrees*1.675) {//while((leftDistance) < encoderToInches(degrees/2) && rightDistance < encoderToInches(degrees/2)) {
    shouldMove = true;
    //rmotorTurnSpeed = abs(rightDistance - inches) * turnSpeed * 2;
    //lmotorTurnSpeed = abs(leftDistance - inches) * turnSpeed * 2;

    desiredHeading = degrees;
    //Serial.println(heading);
    updateOdom();
    updateHeading();
    updateMotorSpeeds();
  }
  shouldMove = false;
  updateOdom();
  updateHeading();
  updateMotorSpeeds();
  //driveSpeed = realDriveSpeed;
  //turnSpeed = realTurnSpeed;
}

enum State {
  RIGHT,
  LEFT
};

State lastState = LEFT;

float lastBestHeading = 0;

bool whiteTrigger = false;
bool whiteTimerTrigger = false;

void updateFollower() {
  float oscillator = sin((millis() - startTime) / 125.0) * 10.0; //goes back and forth from -10 to 10

  //color sensors
  int sensorState = lineSensor.readSensors();

  Serial.println(heading);
  
  if(sensorState == S1_IN_S2_OUT) { //Sensor 1 reads black
    //generalDesiredHeading = heading - 10;
    desiredHeading = heading + 20;
    lastState = LEFT;
    //whiteTimer = millis();
    whiteTrigger = false;
  }

  if(sensorState == S1_OUT_S2_IN) { //Sensor 2 reads black
    //generalDesiredHeading = heading + 10;
    desiredHeading = heading - 20;
    lastState = RIGHT;
    //whiteTimer = millis();
    whiteTrigger = false;
  }

  if(sensorState == S1_IN_S2_IN) { //Both sensors read black
    //generalDesiredHeading = heading;
    desiredHeading = heading;
    lastBestHeading = heading;
    //whiteTimer = millis();
    whiteTrigger = false;
  }

  if(sensorState == S1_OUT_S2_OUT) {
    if(whiteTrigger == false && followLine) {
      if(abs(startTime - millis()) < 7500) {
        whiteTrigger = false; 
        return; 
      }
      whiteTrigger = true;
      whiteTimer = millis();
    }
    if(whiteTrigger == true) {
      if(abs(millis() - whiteTimer) > 2000) {
        whiteTimerTrigger = true;
        if(abs(startTime - millis()) < 7500) {
          //whiteTrigger = false; 
          return; 
        }
        //desiredHeading = lastBestHeading;
        runCorrections = false;
        rightMotor.setMotorPwm(200);
        leftMotor.setMotorPwm(200);
        delay(1375);
        rightMotor.setMotorPwm(0);
        leftMotor.setMotorPwm(0);
        delay(1000);
        claw.run(-255);
        delay(500);
        claw.run(0);
        shouldMove = false;
      }else {
        //whiteTrigger = false;
      }
    }
    
    if(whiteTimerTrigger == true) {
      delay(1000);
      rightMotor.setMotorPwm(0);
      leftMotor.setMotorPwm(0);
    }
    if(lastState == LEFT) {
      desiredHeading = heading + 20;
    }
    if(lastState == RIGHT) {
      desiredHeading = heading - 20;
    }
    //desiredHeading = heading + 360;
  }

  //desiredHeading = generalDesiredHeading + oscillator;
}

void updateHeading() { //also corrects regular drift supposedly
  lmotorTurnSpeed = 0;
  rmotorTurnSpeed = 0;
  if(desiredHeading - heading > 0) { //must turn left
    lmotorTurnSpeed = -abs(desiredHeading - heading) * turnSpeed;
    if(!followLine) {
      rmotorTurnSpeed = abs(desiredHeading - heading) * turnSpeed;
    }
  }
  if(desiredHeading - heading < 0) { //must turn right
    if(!followLine) {
      lmotorTurnSpeed = abs(desiredHeading - heading) * turnSpeed;
    }
    rmotorTurnSpeed = -abs(desiredHeading - heading) * turnSpeed;
  }
}

void updateMotorSpeeds() {
  if(!shouldMove) {
    leftMotor.setMotorPwm(0);
    rightMotor.setMotorPwm(0);
    return; 
  }
  if(!runCorrections) {
    return;
  }
  lmotorSpeed = driveSpeed;
  rmotorSpeed = driveSpeed;
  int lfinalMotorSpeed = lmotorSpeed + lmotorTurnSpeed;
  int rfinalMotorSpeed = rmotorSpeed + rmotorTurnSpeed;

  if(lfinalMotorSpeed > 255) { lfinalMotorSpeed = 255; }
  if(rfinalMotorSpeed > 255) { rfinalMotorSpeed = 255; }
  if(lfinalMotorSpeed < -255) { lfinalMotorSpeed = -255; }
  if(rfinalMotorSpeed < -255) { rfinalMotorSpeed = -255; }

  //Serial.print("rightsine: ");
  //Serial.println(rfinalMotorSpeed);

  leftMotor.setMotorPwm(-lfinalMotorSpeed);
  rightMotor.setMotorPwm(rfinalMotorSpeed);
}