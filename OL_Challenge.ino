/*
MAE 156A Open Loop Challenge

Measure velocity and deal with the 'dead zone' of the potentiometer.

By Daniel Heideman and Stuart Sonatina

*/
#define DIRpin  8
#define PWMpin  9
#define encPinA 2
#define encPinB 3

int serialPrinting = 1;	// verbose serial printing?


int sampleTime = 20;		// time step in milliseconds
int printTime;			    // time between printing to serial

long prevTime = millis();   // time at previous sample
long prevPrintTime = millis(); // time at prev print
long nturns;				// number of turns since restart of arduino

float theta=0;				// potentiometer position
float omega;				// angular velocity

/******************************************************************************
 setup
******************************************************************************/
void setup()
{
    Serial.begin(115200);
    
    pinMode(PWMpin, OUTPUT);
    pinMode(DIRpin, OUTPUT);
    
    if(serialPrinting)
    {
    	printTime = 400; // milliseconds
    	Serial.print("Current position: ");
    	Serial.print(theta);
    	Serial.println(" Degrees");
    	Serial.print("Desired Position: ");
    	Serial.print(setpoint);
    	Serial.println(" Degrees");
    	Serial.println();
    	delay(2000); // milliseconds
    }
    else{printTime = 50;} // milliseconds

}

/******************************************************************************
 looooooooooop
******************************************************************************/
void loop()
{
    // If values are sent through serial monitor, do stuff
    if(Serial.available()>0)
    {
        switch(char(Serial.read()))
        {
        	case 'S':
        	case 's':
        	    setMotor(Serial.parseFloat());
        	    break;
        }
        delay(1000);
    }
    
    // Read position after sampleTime has elapsed
    if(millis()-prevTime>sampleTime)
    {
    	// get current position
    	theta = map(analogRead(A3),0,1024,0,340); // degrees
    	
    	// If position is nearing 'dead zone' with positive velocity,
    	// count as a full positive turn
    	if(theta > 335 && omega > 50){nturns = nturns + 1;}
    	
    	// Same for opposite direction:
    	// If position is nearing 'dead zone' with negative velocity,
    	// count as a full negative turn
    	if(theta < 5 && omega < -50){nturns = nturns - 1;}
    	
    	// wait until pot is out of 'dead zone'
    	while (theta > 335 or theta < 1)
		{
			delay(1);
			// get current position
			theta = map(analogRead(A3),0,1024,0,340); // degrees
		}
    }

    // send position to serial after printTime has elapsed
    if(millis()-prevPrintTime>printTime)
    {
        Serial.println(theta);
        prevPrintTime = millis();
    }
}

/******************************************************************************
 setMotor

 Function to set speed and direction of motor.
 Has saturation protection
******************************************************************************/

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