#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <SparkFun_TB6612.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;
static volatile uint32_t sys_tick = 0;
static volatile uint32_t r_encoder_counts = 0;
static volatile uint32_t l_encoder_counts = 0;

static float phi_deg = 0.0f;
static float phi_dot_dps = 0.0f;

// Controller values
static const float MIN_PHI_FOR_CONTROL = 2.0f;
static const float KP_ANGLE = 40.0f;
static const float KD_ANGLE = 0.0f;

// Kalman filter start
float Q_angle = 0.001;  // Covariance of gyroscope noise
float Q_gyro = 0.003;   // Covariance of gyroscope drift noise
float R_angle = 0.5;    // Covariance of accelerometer
uint8_t C_0 = 1;
float dt = 0.005; // The filter sampling time
float K1 = 0.05; 
float K_0, K_1, t_0, t_1;
float q_bias; // Gyroscope drift 
float Pdot[4] = { 0, 0, 0, 0};
float P[2][2] = {{ 1, 0 }, { 0, 1 }};
float PCt_0, PCt_1, E;
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

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  if (MPU6050_BAND_21_HZ != mpu.getFilterBandwidth()) {
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
  static uint32_t task_1s_trigger_ticks = 0;

  const uint32_t cur_sys_tick = getSysTick();

  if (cur_sys_tick > task_1s_trigger_ticks) {
    task_1s_trigger_ticks += 1000;

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
  }

  if (cur_sys_tick > task_5ms_trigger_ticks) {
    task_5ms_trigger_ticks += 5;

    // Sample and filter phi and phi_dot
    calculatePhi();

    const int16_t angle_pwm = runAngleController();
  
    if (fabsf(phi_deg) > MIN_PHI_FOR_CONTROL) {
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
  const float phi_deg_m = -atan2(a.acceleration.y , a.acceleration.z) * (180.0f / PI);
  const float phi_dot_dps_m = -g.gyro.x * (180.0f / PI);

  // Prior estimate
  phi_deg += (phi_dot_dps_m - q_bias) * dt;          
  const float angle_err = phi_deg_m - phi_deg;
 
  // Differential of azimuth error covariance
  Pdot[0] = Q_angle - P[0][1] - P[1][0];    
  Pdot[1] = - P[1][1];
  Pdot[2] = - P[1][1];
  Pdot[3] = Q_gyro;
 
  // The integral of the covariance differential of the prior estimate error
  P[0][0] += Pdot[0] * dt;    
  P[0][1] += Pdot[1] * dt;
  P[1][0] += Pdot[2] * dt;
  P[1][1] += Pdot[3] * dt;
   
  // Intermediate variable of matrix multiplication
  PCt_0 = C_0 * P[0][0];
  PCt_1 = C_0 * P[1][0];
   
  // Denominator
  E = R_angle + C_0 * PCt_0;
   
  // Gain value
  K_0 = PCt_0 / E;
  K_1 = PCt_1 / E;
 
  // Intermediate variable of matrix multiplication
  t_0 = PCt_0;  
  t_1 = C_0 * P[0][1];
 
  // Posterior estimation error covariance
  P[0][0] -= K_0 * t_0;     
  P[0][1] -= K_0 * t_1;
  P[1][0] -= K_1 * t_0;
  P[1][1] -= K_1 * t_1;
 
  // Posterior estimation
  q_bias += K_1 * angle_err;    
 
  // The differential value of the output value; work out the optimal angular velocity
  phi_dot_dps = phi_dot_dps_m - q_bias;   
 
  // Posterior estimation; work out the optimal angle
  phi_deg += K_0 * angle_err; 
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
