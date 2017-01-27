/*
MAE 156A PD control of motor
Motor Control and encoder sampling
*/
#define DIRpin = 8;
#define PWMpin = 9;
#define encPinA = 2;
#define encPinB = 3;

int deg; // potentiometer position
int sampleTime = 20; // time step
int setpoint; // desired position
int error; // difference between position and setpoint
int prevError; // previous sampling error (for Derv control)

long prevTime = millis(); // time at previous sample
long prevPrintTime = millis(); // time at prev print
float Kp=10; // proportional control const

///////////////////////////////////////////////////////////////////
void setup()
{
  Serial.begin(9600);
  // give user 10 seconds to type constants
  Serial.setTimeout(10000);

  pinMode(PWMpin, OUTPUT);
  pinMode(DIRpin, OUTPUT);

  // set desired postion to 180 degrees past initial position
  setpoint = map(analogRead(A3),0,1024,0,340) + 180;

  // correct setpoint if outside 0-340 degrees
  if (setpoint>340){setpoint = setpoint-340;}

  // Ask for Kp value
  //Serial.print("Enter an integer for Kp: ");
  if(Serial.available()>0){
    Kp = Serial.readStringUntil('\n').toFloat();
  }

  Serial.println();
  Serial.print("Current position: ");
  Serial.print(map(analogRead(A3),0,1024,0,340));
  Serial.println(" Degrees");
  Serial.print("Desired Position: ");
  Serial.print(setpoint);
  Serial.println(" Degrees");
  Serial.println();
  delay(2000);
}

/////////////////////////////////////////////////////////////////
void loop(){

  //controller
  if(millis()-prevTime>sampleTime){
    // get current position
    deg = map(analogRead(A3),0,1024,0,340);

    // calc error
    error = deg-setpoint;

    // set motor output
    setMotor(Kp*error);

    // shift values
    prevError = error;
    prevTime = millis();
  }

  // read pot, send to serial slower than controller
  if(millis()-prevPrintTime>sampleTime*10){
    Serial.println(deg);
    prevPrintTime = millis();
  }
}

void setMotor(int motorSpeed){
  // Set motor direction
  if (motorSpeed>0){digitalWrite(DIRpin,HIGH);}
  else{digitalWrite(DIRpin,LOW);}

  // Set motor speed
  if (motorSpeed>100){motorSpeed=100;}
  if (motorSpeed<-100){motorSpeed=-100;}

  // convert 0-100% to 0-255 duty cycle
  int motorDuty = map(motorSpeed,0,100,0,255);

  // send PWM to motor
  analogWrite(PWMpin,motorDuty);
}
