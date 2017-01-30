/*
MAE 156A PD control of motor
Motor Control and encoder sampling
*/
#define DIRpin  8
#define PWMpin  9
#define encPinA 2
#define encPinB 3

int sampleTime = 20; // time step in milliseconds
int serial_printing = 1; // verbose serial printing?
int print_time; // time between printing to serial
int toggle=0; // toggles setpoint

long prevTime = millis(); // time at previous sample
long prevPrintTime = millis(); // time at prev print
long toggleTime = millis(); // time at last toggle

float theta; // potentiometer position
float setpoint; // desired position
float error; // difference between position and setpoint
float prevError; // previous sampling error (for derivative control)
float deltaError; // change in error (for derivative control)
float dEdt; // error derivative (for derivative control)
float inteError; // integral of error since time = zero
float u; // output to motor
float Kp=0.5; // proportional control const
float Ki=0; // integral control const
float Kd=0.1; // derivative control const

///////////////////////////////////////////////////////////////////////////////
void setup()
{
  Serial.begin(115200);
  
  // give user time to type constants
  Serial.setTimeout(2000); // milliseconds

  pinMode(PWMpin, OUTPUT);
  pinMode(DIRpin, OUTPUT);

  // get current position
  theta = map(analogRead(A3),0,1024,0,340);

  // set desired postion to 180 degrees past initial position
  setpoint = theta + 180.0;

  // correct setpoint if outside 0-340 degrees
  if (setpoint>340.0){setpoint = setpoint-340.0;}
  
  if(serial_printing)
  {
    print_time = 400; // milliseconds
    Serial.print("Current position: ");
    Serial.print(theta);
    Serial.println(" Degrees");
    Serial.print("Desired Position: ");
    Serial.print(setpoint);
    Serial.println(" Degrees");
    Serial.println();
    delay(2000); // milliseconds
  }
  else{print_time = 50;} // milliseconds

}

///////////////////////////////////////////////////////////////////////////////
void loop()
{
  // alternate setpoint
  if(millis()-toggleTime>5000)
  {
    if(toggle){setpoint=90;toggle=0;}
    else{setpoint=180;toggle=1;}
    toggleTime = millis();
  }

  // If values sent through serial monitor, set PID constants
  if(Serial.available()>0)
  {
    switch(char(Serial.read()))
    {
      case 'P':
      case 'p':
        Kp = Serial.parseFloat();
        break;
      case 'I':
      case 'i':
        Ki = Serial.parseFloat();
        break;
      case 'D':
      case 'd':
        Kd = Serial.parseFloat();
        break;
      case 'S':
      case 's':
        setpoint = Serial.parseFloat();
        break;
    }
    if(serial_printing)
    {
      Serial.println();
      Serial.print("Kp =\t");
      Serial.println(Kp);
      Serial.print("Ki =\t");
      Serial.println(Ki);
      Serial.print("Kd =\t");
      Serial.println(Kd);
      Serial.println();
    }
    delay(1000);
  }

  //controller
  if(millis()-prevTime>sampleTime)
  {
    // get current position
    theta = map(analogRead(A3),0,1024,0,340); // degrees

    // calc error
    error = setpoint - theta; // degrees

    // derivative of error
    deltaError = error - prevError; // degrees
    dEdt = deltaError/sampleTime; // degrees/milliseconds

    // integral of error with saturation protection
    inteError += sampleTime*(prevError+error)/2.0;
    if(Ki*inteError>30){inteError=30/Ki;}
    else if(Ki*inteError<-30){inteError=-30/Ki;}

    // set motor output and return u with saturation limit
    u = Kp*error + Ki*inteError - Kd*dEdt;
    u = setMotor(u);

    // shift values
    prevError = error;
    prevTime = millis();
  }

  // read pot, send to serial slower than controller
  if(millis()-prevPrintTime>print_time)
  {
    Serial.print(setpoint);
    Serial.print("\t");
    Serial.println(theta);
    //Serial.print("\t");
    //Serial.print(error);
    //Serial.print("\t");
    //Serial.println(u);
    prevPrintTime = millis();
  }
}
///////////////////////////////////////////////////////////////////////////////
// setMotor
// Function to set speed and direction of motor.
// Has saturation protection
///////////////////////////////////////////////////////////////////////////////
float setMotor(float motorSpeed)
{
  // Set motor direction
  if (motorSpeed>0){digitalWrite(DIRpin,LOW);}
  else{digitalWrite(DIRpin,HIGH);}

  // Saturation protection
  if (motorSpeed>100){motorSpeed=100.0;}
  if (motorSpeed<-100){motorSpeed=-100.0;}

  // convert 0-100% to 0-255 duty cycle
  int motorDuty = map(abs(motorSpeed),0,100,0,255);
  
  // send PWM to motor
  analogWrite(PWMpin,motorDuty);

  // Return the actual controller output with saturation protection
  return motorSpeed;
}
