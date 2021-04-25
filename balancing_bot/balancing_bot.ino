#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <SparkFun_TB6612.h>
#include <Wire.h>

#define DEBUG 0
#define LOG_FILTER_INPUTS 0
#define COMP_FILTER 1

Adafruit_MPU6050 mpu;
static volatile uint32_t sys_tick = 0;
static volatile uint32_t r_encoder_counts = 0;
static volatile uint32_t l_encoder_counts = 0;
static bool borked = false;   // Set true once we've toppled over, requries reset to clear

// Controller values
static const float dt = 0.005;
static const float MAX_PHI_FOR_CONTROL = 45.0f;
static const float KP_ANGLE = 45.0f;
static const float KI_ANGLE = 25.0f;
static const float KD_ANGLE = 0.0f;

#if COMP_FILTER
static const float alpha = 0.98;
static const float phi_dot_bias = 2.5f;
static float phi_deg = 0.0f;
static float phi_dot_dps = 0.0f;
#else
// Kalman filter start
static float phi_deg = 0.0f;        // State x(0)
static float phi_dot_bias = 2.5f;   // State x(1)
static float phi_dot_dps = 0.0f;
static const float Q_angle = 0.002;
static const float Q_bias = 0.05;
static const float R_angle =  0.5;
static const float dt_2 = dt * dt;
static float P_00 = 1e3;
static float P_01 = 0.0f;
static float P_10 = 0.0f;
static float P_11 = 1e3;;
static float K_0 = 0.0f;
static float K_1 = 0.0f;
// Kalman filter end
#endif

// Wheel encoders
static const uint8_t R_ENCODER_PIN = 2;
static const uint8_t L_ENCODER_PIN = 3;
static const uint16_t PULSES_PER_REV = 192;
static const uint16_t COUNTS_PER_REV = PULSES_PER_REV * 2;  // Triggered on rising and falling

// Motors
static const uint8_t MAX_PWM = 255;
static const uint8_t AIN1 = 12;
static const uint8_t BIN1 = 9;
static const uint8_t AIN2 = 13;
static const uint8_t BIN2 = 8;
static const uint8_t PWMA = 5;
static const uint8_t PWMB = 6;
static const uint8_t STBY = 10;
static const int8_t OFFSET_A = -1;  // either 1 or -1
static const int8_t OFFSET_B = -1;  // either 1 or -1
Motor r_motor = Motor(AIN1, AIN2, PWMA, OFFSET_A, STBY);
Motor l_motor = Motor(BIN1, BIN2, PWMB, OFFSET_B, STBY);

void initSysTicker(void);
uint32_t getSysTick(void);
void calculatePhi(void);
int16_t runAngleController(void);
void rightEncoderIsr(void);
void leftEncoderIsr(void);
uint32_t getRightEncoderCounts(void);
uint32_t getLeftEncoderCounts(void);
void clearEncoderCounts(void);
void initFailedloop(const String failure_msg);

void setup() {
  Serial.begin(115200);

  while (!Serial) {
    delay(10);
  }

  // Initialize the MPU6050
  if (!mpu.begin()) {
    initFailedloop("Failed to find MPU6050 chip");
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  if (MPU6050_RANGE_2_G != mpu.getAccelerometerRange()) {
    initFailedloop("Failed to set MPU6050 accelerometer range");
  }

  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  if (MPU6050_RANGE_500_DEG != mpu.getGyroRange()) {
    initFailedloop("Failed to set MPU6050 gyro range");
  }

  mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
  if (MPU6050_BAND_260_HZ != mpu.getFilterBandwidth()) {
    initFailedloop("Failed to set MPU6050 filter bandwidth");
  }

  Serial.println("");
  delay(100);

  // Initialize the wheel encoder inputs
  pinMode(R_ENCODER_PIN, INPUT_PULLUP);
  pinMode(L_ENCODER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(R_ENCODER_PIN), rightEncoderIsr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(L_ENCODER_PIN), leftEncoderIsr, CHANGE);

  // Initialize the system ticker
  initSysTicker();
}

void loop() {
  static uint32_t task_5ms_trigger_ticks = 2;  // Delay this task by 2ms
  static uint32_t task_100ms_trigger_ticks = 0;

  const uint32_t cur_sys_tick = getSysTick();

  if (cur_sys_tick > task_100ms_trigger_ticks) {
    task_100ms_trigger_ticks += 100;
#if DEBUG
    // Print the tilt angle in degrees
    Serial.print("phi = ");
    Serial.println(phi_deg);
  
    // Print the tilt angle rate in degrees / second
    Serial.print("phi_dot = ");
    Serial.println(phi_dot_dps);

    Serial.println("");
#endif
  }

  if (cur_sys_tick > task_5ms_trigger_ticks) {
    task_5ms_trigger_ticks += 5;

    // Sample and filter phi and phi_dot
    calculatePhi();

    const int16_t angle_pwm = runAngleController();
    const float phi_deg_abs = fabsf(phi_deg);
    if (phi_deg_abs < MAX_PHI_FOR_CONTROL && !borked) {
#if !LOG_FILTER_INPUTS && !DEBUG
      r_motor.drive(angle_pwm);
      l_motor.drive(angle_pwm);
#endif
    }
    else {
      r_motor.brake();
      l_motor.brake();
      borked = true;
    }
  }
}

void initSysTicker(void) {
  TCCR1A = 0;  // set entire TCCR0A register to 0
  TCCR1B = 0;  // same for TCCR0B
  TCNT1  = 0;  // initialize counter value to 0

  // Set compare match register for 1khz increments
  OCR1A = 249;  // = (16e6) / (1000 * 64) - 1 (must be < 256)

  // Turn on CTC mode
  TCCR1A |= (1 << WGM01);

  // Set CS01 and CS00 bits for 64 prescaler
  TCCR1B |= (1 << CS01) | (1 << CS00);

  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE0A);
}

ISR(TIMER1_COMPA_vect) {
  sys_tick++;
}

uint32_t getSysTick(void) {
  cli();  // disable interrupts
  const uint32_t temp = sys_tick;
  sei();  // enable interrupts
  return temp;
}

void calculatePhi(void)
{
  // Get new sensor events with the readings
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
    
  // Radial rotation angle calculation formula; 
  // The negative sign indicates the direction.
  // Convert radians to degrees.
  const float phi_deg_m = -atan2f(a.acceleration.y , a.acceleration.z) * (180.0f / PI);
  const float phi_dot_dps_m = (-g.gyro.x * (180.0f / PI));

#if LOG_FILTER_INPUTS
  Serial.print(getSysTick());
  Serial.print(", ");
  Serial.print(phi_deg_m);
  Serial.print(", ");
  Serial.print(phi_dot_dps_m);
  Serial.println("");
#else
#if COMP_FILTER
  phi_dot_dps = phi_dot_dps_m - phi_dot_bias;
  const float gyro_angle = phi_dot_dps * dt;
  phi_deg = (alpha * (phi_deg + gyro_angle)) + ((1 - alpha) * phi_deg_m);
#else
  phi_deg += dt * (phi_dot_dps_m - phi_dot_bias);
  P_00 += (-dt * (P_10 + P_01)) + (P_11 * dt_2) + Q_angle;
  P_01 -= dt * P_11;
  P_10 -= dt * P_11;
  P_11 += Q_bias;

  const float y = phi_deg_m - phi_deg;
  const float S = P_00 + R_angle;
  K_0 = P_00 / S;
  K_1 = P_10 / S;

  phi_deg += K_0 * y;
  phi_dot_bias += K_1 * y;
  phi_dot_dps = phi_dot_dps_m - phi_dot_bias;
  P_00 -= K_0 * P_00;
  P_01 -= K_0 * P_01;
  P_10 -= K_1 * P_00;
  P_11 -= K_1 * P_01;
#endif
#endif
}

int16_t runAngleController(void)
{
  static float phi_deg_last = 0.0f;
  static float I = 0.0f;

  I += phi_deg;
  I = constrain(I, -300, 300);

  const int16_t u = (int16_t)constrain((KP_ANGLE * phi_deg) + (KI_ANGLE * I * dt) + (KD_ANGLE * ((phi_deg - phi_deg_last) / dt)), -MAX_PWM, MAX_PWM);
  phi_deg_last = phi_deg;
  return u;
}

void rightEncoderIsr(void)
{
  r_encoder_counts++;
}

void leftEncoderIsr(void)
{
  l_encoder_counts++;
}

uint32_t getRightEncoderCounts(void)
{
  cli();  // disable interrupts
  const uint32_t temp = r_encoder_counts;
  sei();  // enable interrupts
  return temp;
}

uint32_t getLeftEncoderCounts(void)
{
  cli();  // disable interrupts
  const uint32_t temp = l_encoder_counts;
  sei();  // enable interrupts
  return temp;
}

void clearEncoderCounts(void)
{
  cli();  // disable interrupts
  r_encoder_counts = 0;
  l_encoder_counts = 0;
  sei();  // enable interrupts
}

void initFailedloop(const String failure_msg) {
  Serial.println(failure_msg);
  while (1) {
    delay(10);
  }
}
