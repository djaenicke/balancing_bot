#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <SparkFun_TB6612.h>
#include <Wire.h>

#define DEBUG 0
#define TUNE_KALMAN_FILTER 0

Adafruit_MPU6050 mpu;
static volatile uint32_t sys_tick = 0;
static volatile uint32_t r_encoder_counts = 0;
static volatile uint32_t l_encoder_counts = 0;

static float phi_deg = 0.0f;
static float phi_dot_dps = 0.0f;
static const float phi_dot_bias_dps = 2.5f;

// Controller values
static const float MIN_PHI_FOR_CONTROL = 2.0f;
static const float MAX_PHI_FOR_CONTROL = 45.0f;
static const float KP_ANGLE = 30.0f;
static const float KD_ANGLE = 0.5f;

// Kalman filter start
static const float dt = 0.005;
static const float dt_2 = dt * dt;
static float P[2][2] = { { 1e3, 0 }, { 0 , 1e3 } };          // State estimate covariance matrix
static const float Q[2][2] = { { 1e-3, 0 }, { 0 , 5e-4 } };  // Covariance of the process noise
static const float R[2][2] = { { 0.5, 0 }, { 0 , 1e-3 } };   // Covariance of the observation noise
static float K[2][2] = { { 0, 0 }, { 0, 0 } };
static float C[2][2] = { { 0, 0 }, { 0, 0 } };
static float D[2][2] = { { 0, 0 }, { 0, 0 } };
// Kalman filter end

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
void mul2x2(const float A[2][2], float B[2][2], float C[2][2]);
void inv2x2(const float A[2][2], float B[2][2]);

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
  
    // Print the angular velocity in degrees per second
    Serial.print("phi_dot_dps = ");
    Serial.println(phi_dot_dps);

    // Print the encoder counts
    Serial.print("r encoder counts = ");
    Serial.println(getRightEncoderCounts());

    Serial.print("l encoder counts = ");
    Serial.println(getLeftEncoderCounts());

    Serial.println("");
#endif
  }

  if (cur_sys_tick > task_5ms_trigger_ticks) {
    task_5ms_trigger_ticks += 5;

    // Sample and filter phi and phi_dot
    calculatePhi();

    const int16_t angle_pwm = runAngleController();
    const float phi_deg_abs = fabsf(phi_deg);
    if (phi_deg_abs > MIN_PHI_FOR_CONTROL && phi_deg_abs < MAX_PHI_FOR_CONTROL) {
      r_motor.drive(angle_pwm);
      l_motor.drive(angle_pwm);
    }
    else {
      r_motor.brake();
      l_motor.brake();
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
  const float phi_dot_dps_m = (-g.gyro.x * (180.0f / PI)) - phi_dot_bias_dps;

  // Predition step
  // xk = F * x
  // F = [1 dt; 0 1]
  // x = [phi; phi_dot]
  phi_deg = phi_deg + (dt * phi_dot_dps);

  // P = (F * P * F^T) + Q
  P[0][0] += (P[1][0] * dt) + (P[0][1] * dt) + (P[1][1] * dt_2) + Q[0][0];
  P[0][1] += (P[1][1] * dt) + Q[0][1];
  P[1][0] += (P[1][1] * dt) + Q[1][0];
  P[1][1] += Q[1][1];

  // Update step
  // K = P * H^T * ((H * P * H^T) + R)^-1
  // H = I (both states are measured)
  // K = P * (P + R)^-1
  // Let C = (P + R)
  // Led D = (P + R)^-1
  C[0][0] = P[0][0] + R[0][0];
  C[0][1] = P[0][1];
  C[1][0] = P[1][0];
  C[1][1] = P[1][1] + R[1][1];
  inv2x2(C, D);
  mul2x2(P, D, K);

  // P = (I - (K * H)) * P;
  // H = I (both states are measured)
  // P = (I - K) * P
  // Let C = I - K
  C[0][0] = 1 - K[0][0];
  C[0][1] = -K[0][1];
  C[1][0] = -K[1][0];
  C[1][1] = 1 - K[1][1];

  D[0][0] = P[0][0];
  D[0][1] = P[0][1];
  D[1][0] = P[1][0];
  D[1][1] = P[1][1];

  mul2x2(C, D, P);

  // x = x + K * ([phi_deg_m; phi_dot_dps_m] - (H * F * x));
  // Let y = ([phi_deg_m; phi_dot_dps_m] - (H * F * x));
  float y[2] = { 0, 0 };
  y[0] = phi_deg_m - (phi_deg + (dt * phi_dot_dps));
  y[1] = phi_dot_dps_m - phi_dot_dps;

  phi_deg += K[0][0] * y[0] + K[0][1] * y[1];
  phi_dot_dps += K[1][0] * y[0] + K[1][1] * y[1];
}

int16_t runAngleController(void)
{
  return (int16_t)constrain((KP_ANGLE * phi_deg) + (KD_ANGLE * phi_dot_dps), -MAX_PWM, MAX_PWM);
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

void mul2x2(const float A[2][2], float B[2][2], float C[2][2])
{
  for (uint8_t i = 0; i < 2; i++)
  {
    for (uint8_t j = 0; j < 2; j++)
    {
      C[i][j] = 0;
      for (uint8_t k = 0; k < 2; k++)
      {
        C[i][j] += A[i][k] * B[k][j];
      }
    }
  }
}

void inv2x2(const float A[2][2], float B[2][2])
{
  const float det = (A[0][0] * A[1][1]) - (A[0][1] * A[1][0]);
  const float det_inv = 1 / det;
  B[0][0] = det_inv * A[1][1];
  B[0][1] = det_inv * -A[0][1];
  B[1][0] = det_inv * -A[1][0];
  B[1][1] = det_inv * A[0][0];
}
