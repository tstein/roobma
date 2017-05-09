/*
 * Dustin Sanders, Matthew Nubbe, Ted Stein
 */

#include "math.h"

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

#define SerialDebug true

const uint32_t timer_Hz = 24000000;
const uint32_t interrupt_Hz = 100;
const uint32_t interrupt_period_cycles = timer_Hz / interrupt_Hz;
volatile byte interrupt_flag = 0;
volatile uint32_t interrupt_count = 0;

MPU9250 myIMU;

const int max_freq = 1600;
const int min_freq = 10;
const int freq_step = 8;

// attitude-keeping constants
const float gravity = 9800.0;           // mm/s^2
// gyro weight in commanded acceleration
const float k_derivative = 4500.0;        // mm/s^2 / rad/s
// angle estimation weight
const float k_proportional = 25000.0;    // mm/s^2 / rad
// speed feedback weight
const float k_integral = 5.0;          // mm/s^2 / mm/s
// position error feedback weight
const float k_double_integral = 4.0;   // mm/s^2 / mm
// gyro/commanded accel balance
// gyro is weighted by this value, commanded accel is the complement
const float gyro_weight = 0.96;
// commanded accel coefficient
const float accel_coefficient = (1 - gyro_weight) / gravity;

const float deg_to_rad = M_PI / 180.0;

const float velocity_soft_limit = 50.0; // mm/s
const float velocity_hard_limit = 90.0; // mm/s
const float loop_period = 1 / (float) interrupt_Hz;

const float stepper_freq_ref = 1200;
const float pulley_circumference = 42.2;  // mm
const float speed_to_freq = stepper_freq_ref / pulley_circumference;

typedef struct {
    // sensed
    float gyro_x = 0.0;                 // rad/s
    // computed
    float angle_estimate = 0.0;         // rad
    float speed = 0.0;                  // mm/s
    float position_error = 0.0;         // mm
    // outputs
    float demanded_speed = 0.0;         // mm/s
    float accel = 0.0;                  // mm/s^2
} attitude_state;

attitude_state tick_state;
attitude_state tock_state;

typedef bool phase_t;
const phase_t TICK = true;
const phase_t TOCK = false;
phase_t phase = TICK;

inline float clamp(float min, float val, float max) {
  val = fmin(val, max);
  val = fmax(val, min);
  return val;
}

void setup() {
  Wire.begin();
  Serial.begin(38400);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LEFT_DIR_PIN, OUTPUT);
  pinMode(RIGHT_DIR_PIN, OUTPUT);


  // Read the WHO_AM_I register, this is a good test of communication
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX);
  Serial.print(" I should be "); Serial.println(0x71, HEX);

  if (c == 0x71) // WHO_AM_I should always be 0x68 (it should?!?)
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
  digitalWrite(LED_BUILTIN,1);
  delay(3000);//Wait 3 s to position upright
  digitalWrite(LED_BUILTIN,0);

  analogWriteResolution(PWM_RES);
  analogWriteFrequency(RIGHT_STEP_PIN, min_freq);
  analogWriteFrequency(LEFT_STEP_PIN, min_freq);
  analogWrite(RIGHT_STEP_PIN, DUTY);
  analogWrite(LEFT_STEP_PIN, DUTY);
  
  // Periodic Interal Timer (PIT) setup
  SIM_SCGC6 |= SIM_SCGC6_PIT;
  PIT_MCR = 0x00;
  NVIC_ENABLE_IRQ(IRQ_PIT);
  PIT_LDVAL0 = interrupt_period_cycles;
  PIT_TCTRL0 = PIT_TCTRL_TIE;
  PIT_TCTRL0 |= PIT_TCTRL_TEN;
  PIT_TFLG0 |= 1;
}

void loop() {
  if (interrupt_flag) {
    interrupt_flag = 0;
    if (interrupt_count++ % interrupt_Hz == 0) {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
    on_int();
  }
}

void on_int() {
  //
  // math happens here
  //
  attitude_state* new_state = phase ? &tick_state : &tock_state;
  attitude_state* old_state = phase ? &tock_state : &tick_state;
  phase = !phase;

  // read gyro data
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {
      myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values
      myIMU.getGres();
      // Calculate the gyro value into actual degrees per second
      // This depends on scale being set
      float gx_deg_s = (float)myIMU.gyroCount[0] * myIMU.gRes;

      new_state->gyro_x = gx_deg_s * deg_to_rad;
  }

  // update angle estimation
  float new_angle_estimate = old_state->angle_estimate;
  new_angle_estimate += new_state->gyro_x * loop_period;
  new_angle_estimate *= gyro_weight;
  new_angle_estimate += old_state->accel * accel_coefficient;
  new_state->angle_estimate = new_angle_estimate;

  // update speed
  new_state->speed = 
    clamp(-velocity_hard_limit,
          old_state->speed + (old_state->accel * loop_period),
          velocity_hard_limit);

  // update demanded speed
  new_state->demanded_speed = old_state->demanded_speed;

  // update position error
  new_state->position_error = old_state->position_error +
    ((new_state->speed - new_state->demanded_speed) * loop_period);

  // update commanded accel
  float suggested_speed =
    (new_state->position_error * k_double_integral / k_integral)
    + new_state->demanded_speed;
  suggested_speed = clamp(-velocity_soft_limit, suggested_speed, velocity_soft_limit);

  new_state->accel =
    (new_state->gyro_x * k_derivative) +
    (new_state->angle_estimate * k_proportional) +
    ((new_state->speed + suggested_speed) * k_integral);
  new_state->accel = clamp(-10000, new_state->accel, 10000);


  //
  // electricty happens here
  //
  int freq = speed_to_freq * fabs(new_state->speed);
  int dir = new_state->speed > 0 ? 1 : 0;
  goServos(freq, dir);
}

// updates both motors, setting the 2nd motor of the 2 opposite
int goServos(int frequency, int dir)
{
  analogWriteFrequency(RIGHT_STEP_PIN, frequency);
  analogWriteFrequency(LEFT_STEP_PIN, frequency);
  digitalWrite(RIGHT_DIR_PIN, !dir);
  digitalWrite(LEFT_DIR_PIN, dir);
  return 1;
}

// PIT handler. The LC has one handler for both PITs.
void pit_isr() {
  // Reset the flag so we get interrupted again later.
  PIT_TFLG0 = 1;
  interrupt_flag = 1;
}
