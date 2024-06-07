#include <Servo.h> //Servo motor library
#define MOTOR_SPEED 190
boolean object;

//Right motor
int enableRightMotor=6;
int rightMotorPin1=7;
int rightMotorPin2=8;

//Left motor
int enableLeftMotor=5;
int leftMotorPin1=9;
int leftMotorPin2=10;

int right_din=12; //right IR sensor
#define L_S A4 //ir sensor Left

int left_din=13; //left IR sensor
#define R_S A5 //ir sensor Right

#define servo 11
int analogPin = A0; //Front IR sensor
float sensorVal = 0;
float sensorVolt = 0;
float Vr=5.0;
float sum=0;
float k1=58.38230958;
float k2=-1.01866501;
int cm = 0;
int distance = 0;

int Set=20;
int distance_L, distance_F, distance_R, distance_aa;


void setup(){ // put your setup code here, to run once
Serial.begin(9600); // start serial communication at 9600bps
TCCR0B = TCCR0B & B11111000 | B00000010 ;

pinMode(R_S, INPUT); // declare if sensor as input  
pinMode(L_S, INPUT); // declare ir sensor as input

  pinMode(enableRightMotor, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  
  pinMode(enableLeftMotor, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);

pinMode(servo, OUTPUT);

 for (int angle = 70; angle <= 140; angle += 5)  {
   servoPulse(servo, angle);  }
 for (int angle = 140; angle >= 0; angle -= 5)  {
   servoPulse(servo, angle);  }
 for (int angle = 0; angle <= 70; angle += 5)  {
   servoPulse(servo, angle);  }

distance_F = sharp_read();
delay(500);
}


void loop(){  
//==============================================
//     Line Follower and Obstacle Avoiding
//==============================================  
distance_F = sharp_read();
Serial.print("D F=");Serial.println(distance_F);

//if Right Sensor and Left Sensor are at White color then it will call forword function
 if((digitalRead(R_S) == 0)&&(digitalRead(L_S) == 0)){
  if(distance_F > Set){rotateMotor(MOTOR_SPEED, MOTOR_SPEED); }
                  else{Check_side();}  
 }  
//if Right Sensor is Black and Left Sensor is White then it will call turn Right function
else if((digitalRead(R_S) == 1)&&(digitalRead(L_S) == 0)){ 
if(distance_F > Set){rotateMotor(-MOTOR_SPEED-10, MOTOR_SPEED-10); }
                  else{Check_side();}  
 }    
//if Right Sensor is White and Left Sensor is Black then it will call turn Left function
else if((digitalRead(R_S) == 0)&&(digitalRead(L_S) == 1)){
  if(distance_F > Set){rotateMotor(MOTOR_SPEED-10, -MOTOR_SPEED-10); }
                  else{Check_side();}  
 }    
//if Right Sensor is Black and Left Sensor is Black then it will call turn Stop function
else if((digitalRead(R_S) == 1)&&(digitalRead(L_S) == 1)){rotateMotor(0, 0);} 
}

void servoPulse (int pin, int angle){
int pwm = (angle*11) + 500;      // Convert angle to microseconds
 digitalWrite(pin, HIGH);
 delayMicroseconds(pwm);
 digitalWrite(pin, LOW);
 delay(50); // Refresh cycle of servo
}


//**********************sharp convert V to distance****************************
long sharp_read(){ //
  sum=0;
  for (int i=0; i<100; i++)
  {
    sum=sum+float(analogRead(analogPin));  
  }
  sensorVal=sum/100;
  sensorVolt=sensorVal*Vr/1024;
  cm = pow(sensorVolt*(1/k1), 1/k2);
  Serial.println(cm);
  if (cm==0){
    cm=250;
  }
  return cm;
}

void compareDistance(){
  if(distance_L > distance_R){ //turn left
      object = true;
      turn();
  }
  else if (distance_L <30 && distance_R < 30){ //for objects for front, left and right make a big turn
        Serial.println("backwards");
        rotateMotor(-200, -200);
        delay(6000);

        rotateMotor(MOTOR_SPEED, -MOTOR_SPEED);
        delay(10000);
        rotateMotor(200, 200);
        delay(12000);
        rotateMotor(-MOTOR_SPEED, MOTOR_SPEED);    
        delay(11000);
        rotateMotor(200, 200);
        delay(18500);
        rotateMotor(-MOTOR_SPEED, MOTOR_SPEED);
        delay(8200);
        rotateMotor(200, 200);
        delay(9500);
        rotateMotor(MOTOR_SPEED, -MOTOR_SPEED);
        delay(4500);
   }
  else{
      object = false; //turn right
      turn();
   }
}


void Check_side(){ //makeing servo look left and right to view distance_left and distance_right
  delay(5000);
    rotateMotor(0, 0);
    delay(100); 
    for (int angle = 70; angle <= 140; angle += 5)  {
      servoPulse(servo, angle); }
  delay(300);
  distance_R = sharp_read();
  Serial.print("Distance Right=");Serial.println(distance_R);
  delay(100);
  for (int angle = 140; angle >= 0; angle -= 5)  {
      servoPulse(servo, angle);  }
  delay(500);
  distance_L = sharp_read();
  Serial.print("Distance Left=");Serial.println(distance_L);
  delay(100);
 for (int angle = 0; angle <= 70; angle += 5)  {
      servoPulse(servo, angle);  }
  delay(3000);
  compareDistance();
}

void rotateMotor(int rightMotorSpeed, int leftMotorSpeed){
  if (rightMotorSpeed < 0){
    digitalWrite(rightMotorPin1,LOW);
    digitalWrite(rightMotorPin2,HIGH);    
  }
  else if (rightMotorSpeed > 0){
    digitalWrite(rightMotorPin1,HIGH);
    digitalWrite(rightMotorPin2,LOW);      
  }
  else{
    digitalWrite(rightMotorPin1,LOW);
    digitalWrite(rightMotorPin2,LOW);      
  }

  if (leftMotorSpeed < 0){
    digitalWrite(leftMotorPin1,LOW);
    digitalWrite(leftMotorPin2,HIGH);    
  }
  else if (leftMotorSpeed > 0){
    digitalWrite(leftMotorPin1,HIGH);
    digitalWrite(leftMotorPin2,LOW);      
  }
  else {
    digitalWrite(leftMotorPin1,LOW);
    digitalWrite(leftMotorPin2,LOW);      
  }
  analogWrite(enableRightMotor, abs(rightMotorSpeed));
  analogWrite(enableLeftMotor, abs(leftMotorSpeed));    
}


void turn() { 
  if (object == true) { //turn left function
    rotateMotor(MOTOR_SPEED, -MOTOR_SPEED);
    delay(9000);
    rotateMotor(200, 200);
    delay(10000);
    rotateMotor(-MOTOR_SPEED, MOTOR_SPEED);    
    delay(8700);
    rotateMotor(200, 200);
    delay(13000);
    rotateMotor(-MOTOR_SPEED, MOTOR_SPEED);
    delay(7000);
    rotateMotor(200, 200);
    delay(9800);
    rotateMotor(MOTOR_SPEED, -MOTOR_SPEED);
    delay(4500);

    if (digitalRead(R_S) == 1) {
      loop();
    } else {
      rotateMotor(MOTOR_SPEED, MOTOR_SPEED);
    }
  }
  else { //turn right function
    rotateMotor(-MOTOR_SPEED, MOTOR_SPEED);
    delay(9000);
    rotateMotor(200, 200);
    delay(10000);
    rotateMotor(MOTOR_SPEED, -MOTOR_SPEED);
    delay(8500);
    rotateMotor(200, 200);
    delay(13000);
    rotateMotor(MOTOR_SPEED, -MOTOR_SPEED);
    delay(7000);
    rotateMotor(200, 200);
    delay(9300);
    rotateMotor(-MOTOR_SPEED, MOTOR_SPEED);
    delay(4500);

    if (digitalRead(L_S) == 1) {
      loop();
    } else {
    rotateMotor(MOTOR_SPEED, MOTOR_SPEED);
    }
  }
}