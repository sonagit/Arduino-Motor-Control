/*
MAE 156A Open Loop Challenge

Measure velocity and deal with the 'dead zone' of the potentiometer.

By Daniel Heideman and Stuart Sonatina

*/
#define DIRpin  8
#define PWMpin  9
#define encPinA 2
#define encPinB 3

int serial_printing = 1;    // verbose serial printing?


int sampleTime = 20;        // time step in milliseconds
int print_time;             // time between printing to serial

long prevTime = millis();   // time at previous sample
long prevPrintTime = millis(); // time at prev print
long nturns;                // number of turns since restart of arduino

float theta;                // potentiometer position
float omega;                // angular velocity
float u;                    // output to motor

///////////////////////////////////////////////////////////////////////////////
void setup()
{
  Serial.begin(115200);
  
  // give user time to type constants
  Serial.setTimeout(2000);  // milliseconds

  pinMode(PWMpin, OUTPUT);
  pinMode(DIRpin, OUTPUT);

  // get current position
  theta = map(analogRead(A3),0,1024,0,340); // degrees

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

  // If values sent through serial monitor, do stuff
  if(Serial.available()>0)
  {
    switch(char(Serial.read()))
    {
        case
    }
    delay(1000);
  }

  //controller
  if(millis()-prevTime>sampleTime)
  {
    // get current position
    theta = map(analogRead(A3),0,1024,0,340); // degrees
    
    // If position is nearing 'dead zone' with positive velocity,
    // count as a full positive turn
    if(theta > 335 && omega > 50)
    {
        nturns = nturns + 1;
        
        // wait until pot is out of 'dead zone'
        while (theta > 335 or theta < 1)
        {
            delay(1);
            // get current position
            theta = map(analogRead(A3),0,1024,0,340); // degrees
        }
    }
    
    // Same for opposite direction:
    // If position is nearing 'dead zone' with negative velocity,
    // count as a full negative turn
    if(theta < 5 && omega < -50)
    {
        nturns = nturns - 1;
        
        // wait until pot is out of 'dead zone'
        while (theta > 335 or theta < 1)
        {
            delay(1);
            // get current position
            theta = map(analogRead(A3),0,1024,0,340); // degrees
        }
    }
    
    // set motor output and return u with saturation limit
    u = setMotor(u);
  }

  // read pot, send to serial slower than controller
  if(millis()-prevPrintTime>print_time)
  {
    Serial.println(theta);
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