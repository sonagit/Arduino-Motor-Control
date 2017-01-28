/*
MAE 156A HW2
Motor Control and encoder sampling
*/
#define DIRpin  8
#define PWMpin  9
#define encPinA 2
#define encPinB 3

void setup()
{
	Serial.begin(115200);
	pinMode(PWMpin, OUTPUT);
	pinMode(DIRpin, OUTPUT);
	pinMode(encPinA, INPUT);
	pinMode(encPinB, INPUT);
}

void loop()
{
	// Set motor speed
	setMotor(100);
	delay(2000);

	setMotor(0);
	delay(2000);

}

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
