/*
MAE 156A PD control of motor
Motor Control and encoder sampling
*/
#define DIRpin  8
#define PWMpin  9
#define encPinA 2
#define encPinB 3

int theta; // potentiometer position
int sampleTime = 20; // time step
int setpoint; // desired position
int error; // difference between position and setpoint
int prevError; // previous sampling error (for Derv control)

long prevTime = millis(); // time at previous sample
long prevPrintTime = millis(); // time at prev print
float Kp=0; // proportional control const

///////////////////////////////////////////////////////////////////////////////
void setup()
{
  Serial.begin(9600);
  // give user time to type constants
  Serial.setTimeout(3000);

  pinMode(PWMpin, OUTPUT);
  pinMode(DIRpin, OUTPUT);

  // get current position
  theta = map(analogRead(A3),0,1024,0,340);

  // set desired postion to 180 degrees past initial position
  setpoint = theta + 180;

  // correct setpoint if outside 0-340 degrees
  if (setpoint>340){setpoint = setpoint-340;}

  // Ask for Kp value
  while (Kp==0)
  {
    Serial.print("Enter an integer for Kp: ");
    Kp = (float) Serial.readStringUntil('\n').toFloat();
    Serial.println();
  }
  Serial.print("Kp = ");
  Serial.println(Kp);

  Serial.print("Current position: ");
  Serial.print(theta);
  Serial.println(" Degrees");
  Serial.print("Desired Position: ");
  Serial.print(setpoint);
  Serial.println(" Degrees");
  Serial.println();
  delay(2000);
}

///////////////////////////////////////////////////////////////////////////////
void loop(){

  //controller
  if(millis()-prevTime>sampleTime){
    // get current position
    theta = map(analogRead(A3),0,1024,0,340);

    // calc error
    error = setpoint-theta;

    // set motor output
    setMotor(Kp*error);

    // shift values
    prevError = error;
    prevTime = millis();
  }

  // read pot, send to serial slower than controller
  if(millis()-prevPrintTime>sampleTime*10){
    Serial.print(setpoint);
    Serial.print(' ');
    Serial.print(theta);
    Serial.print(' ');
    Serial.println(error);
    prevPrintTime = millis();
  }
}
///////////////////////////////////////////////////////////////////////////////
// setMotor
// Function to set speed and direction of motor.
// Has saturation protection
///////////////////////////////////////////////////////////////////////////////
int setMotor(int motorSpeed)
{
  // Set motor direction
  if (motorSpeed>0){digitalWrite(DIRpin,HIGH);}
  else{digitalWrite(DIRpin,LOW);}

  // Saturation protection
  if (motorSpeed>100){motorSpeed=100;}
  if (motorSpeed<-100){motorSpeed=-100;}

  // Return the actual controller output with saturation protection
  return motorSpeed;

  // convert 0-100% to 0-255 duty cycle
  int motorDuty = map(abs(motorSpeed),0,100,0,255);
  
  // send PWM to motor
  analogWrite(PWMpin,motorDuty);
}
