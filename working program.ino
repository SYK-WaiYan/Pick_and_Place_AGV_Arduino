#include <Servo.h>  // include servo library header file
//Jesus take the wheel
Servo servo1;   // create servo object to control servo
Servo servo2;

// declare variables for optical sensor input 
byte pinOP1 = 10;
byte pinOP2 = 11;
byte pinOP3 = 12;
byte pinOP4 = 13;
byte op1;
byte op2;
byte op3;
byte op4;
byte opAll;

//Arduino PWM speed and direction control
// Motor 1 = Left Motor
// Motor 2 = Right Motor
byte M1 = 4;  // Motor 1 direction control
byte E1 = 5;  // Motor 1 speed control
byte E2 = 6;  // Motor 2 speed control
byte M2 = 7;  // Motor 2 direction control
byte speedM1A = 120;  // A speed = fastest
byte speedM2A = 120;
byte speedM1B = 100;   // B speed = middle
byte speedM2B = 100;
byte speedM1C = 80;   // C speed = slowest
byte speedM2C = 80;

// after turning
byte speedM1D = 65;   // D speed = fast
byte speedM2D = 65;
byte speedM1E = 60;   // E speed = medium
byte speedM2E = 60;
byte speedM1F = 50;   // F speed = slpw
byte speedM2F = 50;

// servo motor PWM position in microseconds
int servo1GripClose = 30;
int servo1GripOpen = 150;
int servo2GripUp = 60;
int servo2GripDown = 110;
//1000 ms -> 1500 ms -> 2000 ms (0 deg -> 90 deg -> 180 deg)

int left_turn = 0; // number of left turn made
int left_turn_timer = 0; // number of loops made
int stage = 0; // stage system (0 -> 5)

void setup() {
  // put your setup code here, to run once:
  // Optical Sensor
  //int stage = 0;
  pinMode(pinOP1, INPUT);     // set pin as digital input (2, INPUT)
  pinMode(pinOP2, INPUT);     // (0, INPUT)
  pinMode(pinOP3, INPUT);     // (12, INPUT)
  pinMode(pinOP4, INPUT);     // (13, INPUT)
  Serial.begin(9600);         // set serial communication for serial monitor (9600 bps)

  //DC Motor
  pinMode(M1, OUTPUT);        // Output Pins
  pinMode(M2, OUTPUT);

  // Servo Motor
  servo1.attach(3);   // set servo motor output PWM signal pin (PIN 3) (Can use PIN 9 for 2nd servo motor)
  servo2.attach(9);   // set servo motor output PWM signal pin (PIN 9) (Can use PIN 9 for 2nd servo motor)
  servo2.write(servo2GripUp); // output servo position, Servo Motor moves to bring up the gripper
  delay(500); 
  servo1.write(servo1GripOpen); // output servo position, Servo Motor moves to open the gripper  
  delay(500);
}

void loop()
{
  // put your main code here, to run repeatedly:
  readOpticalSensor();
  //Serial.print("stage: ");   //display stage value
  //Serial.println(stage, BIN);

  if(stage == 0) //start line tracing
  {
   if (left_turn == 1 && left_turn_timer <=10) 
    { //if the bot has turned left once, start the timer
      left_turn_timer+=1;
    }
    else if (left_turn == 1 && left_turn_timer >= 8) 
    { //all black => turn left
      stage=1;
    }

    if (opAll == B0110 || opAll == B1001) 
    { // in centre of line => move forward
    motorPWM(HIGH, speedM1A, HIGH, speedM2A);
    }
   // Optical Sensor Detect Black Surface -> Value = 1
   // Optical Sensor Detect White / Reflective Surface -> Value = 0
    else if (opAll == B0011) 
    {    // in left-most side of the line => turn right
      motorPWM(HIGH, speedM1A, LOW, speedM2C);
      //motorPWM(HIGH, speedM1B, LOW, speedM2B);      
    }
    else if (opAll == B0111) 
    {    // in left side of the line => turn right
      motorPWM(HIGH, speedM1B, LOW, speedM2C);
      //motorPWM(HIGH, speedM1B, LOW, speedM2B);
    }
    else if (opAll == B1100) 
    {     // in right-most side of the line => turn left
      motorPWM(LOW, speedM1C, HIGH, speedM2A);
      left_turn += 1;
      //motorPWM(LOW, speedM1B, HIGH, speedM2B);
    }
    else if (opAll == B1110) 
    {     // in right side of the line => turn left
      motorPWM(LOW, speedM1C, HIGH, speedM2B);
      //motorPWM(LOW, speedM1B, HIGH, speedM2B);
    }
    else if (opAll == B1111 || opAll == B0111) 
    { //all black => turn left
      motorPWM(LOW, 10, HIGH, 145);
      left_turn = 1; 
    }
  }

  else if(stage == 1) //slower line tracing after turning
  {
    if (opAll == B0110 || opAll == B1001) 
    { // in centre of line => move forward
    motorPWM(HIGH, speedM1D, HIGH, speedM2D);
    }
  // Optical Sensor Detect Black Surface -> Value = 1
  // Optical Sensor Detect White / Reflective Surface -> Value = 0
    else if (opAll == B0011 || opAll == B0001)
    {   // in left-most side of the line => turn right
      motorPWM(HIGH,speedM1D , LOW, speedM2E);
    }
    else if (opAll == B0111) 
    {    // in left side of the line => turn right
      motorPWM(HIGH, speedM1D, LOW, speedM2F);
    }
    else if (opAll == B1100 || opAll == B1000)  
    {     // in right-most side of the line => turn left
      motorPWM(LOW, speedM1F, HIGH, speedM2E);
    }
    else if (opAll == B1111) 
    {    //all black => turn left
        motorPWM(LOW, speedM1F, HIGH, speedM2E);
    }
    else if (opAll == B1110) 
    {     // in right side of the line => turn left
      motorPWM(LOW, speedM1E, HIGH, speedM2D);
    }
    else if (opAll == B0111) 
    {     // in left side of the line => turn right
      motorPWM(LOW, speedM1D, HIGH, speedM2E);
    }
    else if (opAll == B0000) //all white --> move onto stage 2 to pick the ball
    {
      stage=2;
    }
    /*else {                        // other conditions => stop
      motorPWM(LOW, 0, LOW, 0);
    }*/
  }
  else if(stage == 2) //open gripper, down gripper, close gripper, up gripper
  {
    motorPWM(LOW, 0, LOW, 0); //STOP
    //delay(500);
    //servo1.write(servo1GripOpen);
    delay(500);
    servo2.write(servo2GripDown); // output servo position, Servo Motor moves to lower down the gripper
    delay(500);
    servo1.write(servo1GripClose); // output servo position, Servo Motor moves to close the gripper
    delay(500);
    servo2.write(servo2GripUp); // output servo position, Servo Motor moves to bring up the gripper
    stage=3;
  }
  else if(stage == 3) //spin until see BLACK
  {
    if (opAll == B0000) // if the optical sensors detect white -> keep spinning
    {                        
     motorPWM(LOW, 85, HIGH, 85);
    }
    else // other conditions -> stop the motor and move onto stage-4 for line tracing
    {
      motorPWM(LOW, 0, LOW, 0);
      stage=4;
    }    
  }
  else if(stage == 4) //line tracing - Fast
  {
    if (opAll == B0110 || opAll == B1001) 
    { // in centre of line => move forward
    motorPWM(HIGH, speedM1C, HIGH, speedM2C);
    }
    // Optical Sensor Detect Black Surface -> Value = 1
    // Optical Sensor Detect White / Reflective Surface -> Value = 0
    else if (opAll == B0011 || opAll == B0001)
    {    // in left-most side of the line => turn right
      motorPWM(HIGH, speedM1A, LOW, speedM2B);
    }
    else if (opAll == B0111) 
    {    // in left side of the line => turn right
      motorPWM(HIGH, speedM1A, LOW, speedM2C);
    }
    else if (opAll == B1100 || opAll == B1000)  
    {     // in right-most side of the line => turn left
      motorPWM(LOW, speedM1C, HIGH, speedM2B);
    }
    else if (opAll == B1111 || opAll == B0111) 
    { //all black => turn left
        motorPWM(LOW, 40, HIGH, 150);
    }
    else if (opAll == B1110) 
    {     // in right side of the line => turn left
      motorPWM(LOW, speedM1B, HIGH, speedM2A);
    }
    else if (opAll == B0000) //all white --> DROP THE DAMN BALL
    {
      motorPWM(LOW, 0, LOW, 0); //STOP

      delay(500);
      servo2.write(servo2GripDown); // output servo position, Servo Motor moves to lower down the gripper
      delay(500);
      servo1.write(servo1GripOpen); // output servo position, Servo Motor moves to open the gripper
      delay(500);
      servo2.write(servo2GripUp); // output servo position, Servo Motor moves to bring up the gripper
      stage=5;
    }
  }

  else if (stage == 5) 
  {
    motorPWM(LOW, 0, LOW, 0);
  }

  else {
    motorPWM(LOW, 0, LOW, 0);
  }
  
}

// Motor Control and Optical Sensor Functions
void motorPWM(bool motor1dir, byte motor1pwm, bool motor2dir, byte motor2pwm) {
  // Motor 1 = Left Motor
  // Motor 2 = Right Motor
  digitalWrite(M1, motor1dir); // motor 1 direction control, HIGH = forward, LOW = reverse
  analogWrite(E1, motor1pwm);  // motor 1 PWM speed control, 0 - 255 => 0 - 100% duty cycle
  digitalWrite(M2, motor2dir); // motor 2 direction control, HIGH = forward, LOW = reverse
  analogWrite(E2, motor2pwm);  // motor 2 PWM speed control, 0 -> 255 => 0 -> 100% duty cycle
}

void readOpticalSensor() {
  op1 = digitalRead(pinOP2);
  op2 = digitalRead(pinOP1);
  op3 = digitalRead(pinOP4);
  op4 = digitalRead(pinOP3);
  opAll = op1*8 + op2*4 + op3*2 + op4;

  Serial.print("optical sensor: ");   //display optical sensor value
  Serial.println(opAll, BIN); 
}
