#include <MeOrion.h>
#include <SoftwareSerial.h>
#include <Wire.h>

MeDCMotor rightMotor(M1); 
MeDCMotor leftMotor(M2);
MeGyro gyro(PORT_7);
MeUltrasonicSensor fultra(PORT_3);
MeUltrasonicSensor rultra(PORT_8);

int lmotorSpeed = 0;
int rmotorSpeed = 0;
int lmotorTurnSpeed = 0;
int rmotorTurnSpeed = 0;
float fdistance = 400;
float rdistance = 400;

int driveSpeed = 220.0;
int turnSpeed =  6.0;

float heading = 0;
float desiredHeading = 0;

float centimeterDistance = 30;

int startTime = 0;
int forwardTime = 0;

void setup() {
  Serial.begin(9600);
  gyro.begin();
  delay(250);
  startTime = millis();
  while(fultra.distanceCm() > 30)
  {
    delay(100);
  }
  delay(15000);
  gyro.begin();
  gyro.update();
  heading = gyro.getAngleZ();
  Serial.println("Will turn Right");
  int currentHeading = heading;
  int motorTurnSpeed = 0;
  while(abs(currentHeading - heading) < 90) {
    gyro.update();
    heading = gyro.getAngleZ();
    motorTurnSpeed = - abs(heading - currentHeading) * turnSpeed;
    rightMotor.run(-(driveSpeed - motorTurnSpeed));
    leftMotor.run(-(driveSpeed - motorTurnSpeed + 45));
    /*Serial.print("desired: ");
    Serial.print(currentHeading + 90);
    Serial.print("current: ");
    Serial.println(heading);*/
  }
  rightMotor.run(0);
  leftMotor.run(0);
  gyro.begin();
  gyro.update();
  heading = gyro.getAngleZ();
  Serial.println("Will turn Right");
  currentHeading = heading;
  motorTurnSpeed = 0;
  while(abs(currentHeading - heading) < 90) {
    gyro.update();
    heading = gyro.getAngleZ();
    motorTurnSpeed = - abs(heading - currentHeading) * turnSpeed;
    rightMotor.run(-(driveSpeed - motorTurnSpeed));
    leftMotor.run(-(driveSpeed - motorTurnSpeed + 45));
    /*Serial.print("desired: ");
    Serial.print(currentHeading + 90);
    Serial.print("current: ");
    Serial.println(heading);*/
  }
  rightMotor.run(0);
  leftMotor.run(0);
}

void loop() {
  gyro.update();
  heading = gyro.getAngleZ();
  
  goForward();

  if(abs(millis() - startTime) > 1000) {
    updateMaze();
    startTime = millis();
  }

  //Serial.print("current: ");
  //Serial.println(heading);

  //updateHeading();
  //updateSpeed();
}

void updateMaze() {
  fdistance = fultra.distanceCm();
  rdistance = rultra.distanceCm();

  Serial.print("F: ");
  Serial.print(fdistance);
  Serial.print(" R: ");
  Serial.println(rdistance);

  if(rdistance < centimeterDistance) { //wall is to the right
    if(fdistance < centimeterDistance) { //wall is forward
      turnLeft();
      //turn(90); //if wall is to the right and front, then turn left
    }else { //wall is to the right and not forward, then go forward
      //desiredHeading = heading;
      goForward();
    }
  }else { //if no wall to the right, turn right
    turnRight();
    //turn(-90);
  }

}

void turnRight() {
  gyro.begin();
  gyro.update();
  heading = gyro.getAngleZ();
  Serial.println("Will turn Right");
  int currentHeading = heading;
  int motorTurnSpeed = 0;
  while(abs(currentHeading - heading) < 90) {
    gyro.update();
    heading = gyro.getAngleZ();
    motorTurnSpeed = - abs(heading - currentHeading) * turnSpeed;
    rightMotor.run(-(driveSpeed - motorTurnSpeed));
    leftMotor.run(-(driveSpeed - motorTurnSpeed + 45));
    /*Serial.print("desired: ");
    Serial.print(currentHeading + 90);
    Serial.print("current: ");
    Serial.println(heading);*/
  }
  rightMotor.run(0);
  leftMotor.run(0);
  //if(fdistance > centimeterDistance) { //wall is not forward
    goForward();
    goForward();
  //}
}

void turnLeft() {
  gyro.begin();
  gyro.update();
  heading = gyro.getAngleZ();
  Serial.println("Will turn Left");
  int currentHeading = heading;
  int motorTurnSpeed = 0;
  while(abs(currentHeading - heading) < 90) {
    gyro.update();
    heading = gyro.getAngleZ();
    motorTurnSpeed = - abs(heading - currentHeading) * turnSpeed;
    rightMotor.run((driveSpeed - motorTurnSpeed));
    leftMotor.run((driveSpeed - motorTurnSpeed) + 45);
    /*Serial.print("desired: ");
    Serial.print(currentHeading - 90);
    Serial.print("current: ");
    Serial.println(heading);*/
  }
  rightMotor.run(0);
  leftMotor.run(0);

  goForward();
  goForward();
}

void goForward() {
  gyro.begin();
  gyro.update();
  heading = gyro.getAngleZ();
  Serial.println("Going Straight");
  forwardTime = millis();
  while(abs(forwardTime - millis()) < 1000) {
    fdistance = fultra.distanceCm();
    rdistance = rultra.distanceCm();
    if(fdistance < 20) {
      break;
    }
    int lmotorTurnSpeed = 0;
    int rmotorTurnSpeed = 0;
    if(heading > 0) { //must turn left
      lmotorTurnSpeed = -abs(heading) * turnSpeed;
      //rmotorTurnSpeed = abs(desiredHeading - heading) * turnSpeed;
    }
    if(heading < 0) { //must turn right
      //lmotorTurnSpeed = abs(desiredHeading - heading) * turnSpeed;
      rmotorTurnSpeed = -abs(heading) * turnSpeed;
    }
    rightMotor.run(driveSpeed - rmotorTurnSpeed);
    leftMotor.run(-(driveSpeed - lmotorTurnSpeed + 45));
  }
  rightMotor.run(0);
  leftMotor.run(0);
}

void updateHeading() {
  /*heading = gyro.getAngleZ();
  lmotorTurnSpeed = 0;
  rmotorTurnSpeed = 0;
  if(desiredHeading - heading > 0) { //must turn left
    lmotorTurnSpeed = -abs(desiredHeading - heading) * turnSpeed;
    //rmotorTurnSpeed = abs(desiredHeading - heading) * turnSpeed;
  }
  if(desiredHeading - heading < 0) { //must turn right
    //lmotorTurnSpeed = abs(desiredHeading - heading) * turnSpeed;
    rmotorTurnSpeed = -abs(desiredHeading - heading) * turnSpeed;
  }*/
}

void updateSpeed() {
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

  leftMotor.run(-lfinalMotorSpeed);
  rightMotor.run(rfinalMotorSpeed);
}

void turn(int degrees) {
  desiredHeading = heading + degrees;
  
  lmotorTurnSpeed = 0;
  rmotorTurnSpeed = 0;
  while(desiredHeading - heading > 5) { //must turn left
    gyro.update();
    heading = gyro.getAngleZ();
    lmotorTurnSpeed = -abs(desiredHeading - heading) * turnSpeed;
    updateSpeed();
    /*Serial.print("desired: ");
    Serial.print(desiredHeading);
    Serial.print("current: ");
    Serial.println(heading);*/
    //rmotorTurnSpeed = abs(desiredHeading - heading) * turnSpeed;
  }
  while(desiredHeading - heading < 5) { //must turn right
    gyro.update();
    heading = gyro.getAngleZ();
    //lmotorTurnSpeed = abs(desiredHeading - heading) * turnSpeed;
    rmotorTurnSpeed = -abs(desiredHeading - heading) * turnSpeed;
    updateSpeed();
    Serial.print("desired: ");
    Serial.print(desiredHeading);
    Serial.print("current: ");
    Serial.println(heading);
  }
}