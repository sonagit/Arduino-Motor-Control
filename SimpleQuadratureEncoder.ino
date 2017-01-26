/*  
 *  MAE 156A - Interrupt Function for Measuring No-Load Speed
 *  Motor: Pololu Motor 3260
 *  Spec No-Load Speed: 5600 RPM
 *  For No-Load Speed
 *  
 *  Janaury 19th, 2017
 *  
 */
 
#define motor_pwm 9       //define PWM pin number for motor pwm
#define motor_dir 8       //define digital pin number for motor direction
#define encA 2            //define Encoder A channel pin
#define encB 3            //define Encoder B channel pin
 
int encAnow = LOW;        //define current state of Encoder A
int encAprev = LOW;       //define previous state of Encoder A
int count = LOW;          //define a variable if the new encoder signal has been detected
 
long encPOS;              //encoder counting variable
unsigned long t;          //interrupt timing and counting variables
 
void setup()
{
	Serial.begin(250000);              //serial output transmission rate (bps)  
	pinMode(encA, INPUT);              //Encoder A on motor encoder
	pinMode(encB, INPUT);              //Encoder B on motor encoder
	// Using the interrupt on Digital Pin #2
	attachInterrupt(digitalPinToInterrupt(encA),encoder,CHANGE);
	 
	pinMode(motor_dir, OUTPUT);        //define pin for motor direction (HIGH or LOW)
	digitalWrite(motor_dir, HIGH);     //set motor direction
}
 
void loop()
{
	Serial.println("~~~~~~~~Begin!~~~~~~~~");
	analogWrite(motor_pwm, 255);       //set motor speed in duty cycle (0-255)
	long start=micros();
	while(((long) micros()-start) < 500)
	{
		if (count == HIGH)
		{
			 Serial.print((long) micros() - start); //time base for speed data
			 Serial.print(",");                //data separator character
			 Serial.println(encPOS);           //counts for position and velocity calculation
			 count = LOW;
		}
	}
	analogWrite(motor_pwm, 0);       //set motor speed in duty cycle (0-255)
	delay(10000);
}
 
void encoder() {                     //interrupt handler code
 encAnow = digitalRead(encA);
	 if (count == LOW){
		 // Check if Encoder A is transitioning from 'LOW' to 'HIGH' State
		 if ((encAprev == LOW) && (encAnow == HIGH)) {
				// Check if at that state the Encoder B is 'LOW' (CCW)
				if (digitalRead(encB) == LOW) {
						encPOS--;
						count = HIGH;
				}
				// If Encoder B is 'HIGH', then the direction must be CW
				else {
						encPOS++;
						count = HIGH;
				}
		 }
	}
}
