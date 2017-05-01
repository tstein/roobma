/*
* Dustin Sanders, Matt Nubbe, Ted Stein
*
*/

#include <AccelStepper.h>
#include <MultiStepper.h>
#include "quaternionFilters.h"
#include "MPU9250.h"

// The left Stepper pins
// Timer 1
#define LEFT_DIR_PIN 16
#define LEFT_STEP_PIN 17
// Timer 0
// The right stepper pins
#define RIGHT_DIR_PIN 21
#define RIGHT_STEP_PIN 20

#define PWM_RES 8
#define DUTY 127
// AccelStepper left(AccelStepper::DRIVER, LEFT_STEP_PIN, LEFT_DIR_PIN);
// AccelStepper right(AccelStepper::DRIVER, RIGHT_STEP_PIN, RIGHT_DIR_PIN);
MPU9250 myIMU;
int max_freq = 1600;
int min_freq = 10;
int freq_step = 8;
// int gospeed = 2400;
// int acc = 1600;
int myLed  = 13;  // Set up pin 13 led for toggling
#define SerialDebug true  // Set to true to get Serial output for debugging

void setup() {
  Wire.begin();
  Serial.begin(38400);

  // right.setMaxSpeed(step_speed);
  // right.setAcceleration(acc);
  
  // left.setMaxSpeed(step_speed);
  // left.setAcceleration(acc);

  pinMode(myLed, OUTPUT);
  pinMode(LEFT_DIR_PIN, OUTPUT);
  pinMode(RIGHT_DIR_PIN, OUTPUT);
  digitalWrite(myLed, HIGH);
  analogWriteResolution(PWM_RES);
  analogWriteFrequency(RIGHT_STEP_PIN, min_freq);
  analogWriteFrequency(LEFT_STEP_PIN, min_freq);
  analogWrite(RIGHT_STEP_PIN, DUTY);
  analogWrite(LEFT_STEP_PIN, DUTY); 


  // Read the WHO_AM_I register, this is a good test of communication
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX);
  Serial.print(" I should be "); Serial.println(0x71, HEX);

  if (c == 0x71) // WHO_AM_I should always be 0x68
  {
    Serial.println("MPU9250 is online...");

    // Start by performing self test and reporting values
    myIMU.MPU9250SelfTest(myIMU.SelfTest);
    Serial.print("x-axis self test: acceleration trim within : ");
    Serial.print(myIMU.SelfTest[0],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: acceleration trim within : ");
    Serial.print(myIMU.SelfTest[1],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: acceleration trim within : ");
    Serial.print(myIMU.SelfTest[2],1); Serial.println("% of factory value");
    Serial.print("x-axis self test: gyration trim within : ");
    Serial.print(myIMU.SelfTest[3],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: gyration trim within : ");
    Serial.print(myIMU.SelfTest[4],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: gyration trim within : ");
    Serial.print(myIMU.SelfTest[5],1); Serial.println("% of factory value");

    // Calibrate gyro and accelerometers, load biases in bias registers
    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);

    myIMU.initMPU9250();
    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
    Serial.println("MPU9250 initialized for active data mode....");

    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX);
    Serial.print(" I should be "); Serial.println(0x48, HEX);

    // Get magnetometer calibration from AK8963 ROM
    myIMU.initAK8963(myIMU.magCalibration);
    // Initialize device for active mode read of magnetometer
    Serial.println("AK8963 initialized for active data mode....");
    if (SerialDebug)
    {
      //  Serial.println("Calibration values: ");
      Serial.print("X-Axis sensitivity adjustment value ");
      Serial.println(myIMU.magCalibration[0], 2);
      Serial.print("Y-Axis sensitivity adjustment value ");
      Serial.println(myIMU.magCalibration[1], 2);
      Serial.print("Z-Axis sensitivity adjustment value ");
      Serial.println(myIMU.magCalibration[2], 2);
    }
	}  // if (c == 0x71)
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);
    while(1) ; // Loop forever if communication doesn't happen
  }
}

void loop() {
	static int dir = 1;
	static int inc = 1;
	static int current_step = 10;
  digitalWrite(myLed, HIGH);
	delay(10); 
  digitalWrite(myLed, LOW);
	current_step += (inc)? freq_step:-freq_step;
	// zero crossing
	if (current_step < min_freq)
	{
		if (SerialDebug) { Serial.println("hit min speed"); }
		current_step = min_freq;
		// change directions
		if (dir) {
			dir = 0;
			if (SerialDebug) { Serial.println("going backwards"); }
		} else {
			dir = 1;
			if (SerialDebug) { Serial.println("going forwards"); }
		}
		// start incrementing again
		inc = 1;
	}
	// hitting max, reverse
	if (current_step > max_freq)
	{
		if (SerialDebug) { Serial.println("hit max speed"); }
		inc = 0;
		current_step = max_freq;
	}
	goServos(current_step, dir);

  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {  
    myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values
    myIMU.getAres();
    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    myIMU.ax = (float)myIMU.accelCount[0]*myIMU.aRes; // - accelBias[0];
    myIMU.ay = (float)myIMU.accelCount[1]*myIMU.aRes; // - accelBias[1];
    myIMU.az = (float)myIMU.accelCount[2]*myIMU.aRes; // - accelBias[2];

    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values
    myIMU.getGres();
    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    myIMU.gx = (float)myIMU.gyroCount[0]*myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1]*myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2]*myIMU.gRes;

    // myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values
    // myIMU.getMres();

		// if (myIMU.ax > 0) {
		// 	right.setSpeed(gospeed);
  //     //Serial.println("Right Forwards!");
		// } else {
		// 	right.setSpeed(-gospeed);
  //     //Serial.println("Right Backwards!");
		// }
		// if (myIMU.ax > 0) {
		// 	left.setSpeed(-gospeed);
  //     //Serial.println("Left Backwards!");
		// } else {
		// 	left.setSpeed(gospeed);
  //     //Serial.println("Left Forwards!");
		// }
		// right.runSpeed();
		// left.runSpeed();
//
//		if(SerialDebug)
//	  {
//   
//	    // Print acceleration values in milligs!
//	    Serial.print("X-acceleration: "); Serial.print(1000*myIMU.ax);
//	    Serial.print(" mg ");
//	    Serial.print("Y-acceleration: "); Serial.print(1000*myIMU.ay);
//	    Serial.print(" mg ");
//	    Serial.print("Z-acceleration: "); Serial.print(1000*myIMU.az);
//	    Serial.println(" mg ");
//      
//	    // Print gyro values in degree/sec
//	    Serial.print("X-gyro rate: "); Serial.print(myIMU.gx, 3);
//	    Serial.print(" degrees/sec ");
//	    Serial.print("Y-gyro rate: "); Serial.print(myIMU.gy, 3);
//	    Serial.print(" degrees/sec ");
//	    Serial.print("Z-gyro rate: "); Serial.print(myIMU.gz, 3);
//	    Serial.println(" degrees/sec");
//
//	    myIMU.tempCount = myIMU.readTempData();  // Read the adc values
//	    // Temperature in degrees Centigrade
//	    myIMU.temperature = ((float) myIMU.tempCount) / 333.87 + 21.0;
//	    // Print temperature in degrees Centigrade
//	    Serial.print("Temperature is ");  Serial.print(myIMU.temperature, 1);
//	    Serial.println(" degrees C");
//	  }
  }
  // right.runSpeed();
  // left.runSpeed();
}

// updates both motors, setting the 2nd motor of the 2 opposite
int goServos(int freqnecy, int dir)
{
	analogWriteFrequency(RIGHT_STEP_PIN, freqnecy);
	analogWriteFrequency(LEFT_STEP_PIN, freqnecy);
	digitalWrite(RIGHT_DIR_PIN, dir);
  Serial.print("direction"); Serial.println(dir);
	digitalWrite(LEFT_DIR_PIN, -dir);

	return 1;
}
