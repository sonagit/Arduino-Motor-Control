/*
MAE 156A HW2
Motor Control and encoder sampling
*/
#define DIRpin = 8;
#define PWMpin = 9;
#define encPinA = 2;
#define encPinB = 3;

int samples=500;

void setup()
{
	Serial.begin(115200);
	pinMode(PWMpin, OUTPUT);
	pinMode(DIRpin, OUTPUT);
	pinMode(encPinA, INPUT);
	pinMode(encPinB, INPUT);

	// Set motor direction
	digitalWrite(DIRpin,HIGH);
}

void loop()
{
	// begin!
	Serial.println("----------------------------------------------------------");
	Serial.println("Begin!");

	// Set motor speed and start timer
	analogWrite(PWMpin,255*1);
	float start=micros();

	// read encoder 'samples' times
	for(int i=0; i<samples; i++)
	{
		Serial.print(digitalRead(encPin));
		Serial.print(',');
		Serial.println((micros()-start)/1000);
	}

	// Turn motor off
	analogWrite(PWMpin,255*0);

	delay(10000);
}


