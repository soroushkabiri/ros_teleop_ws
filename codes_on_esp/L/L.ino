#include <WiFi.h>
#include <WebServer.h>
#include "driver/mcpwm.h"
#include <Wire.h>
#include <MPU6050.h>

// Encoder pins for Motor 1
#define ENC_A 21
#define ENC_B 19
volatile int encoderCount1 = 0;
volatile int currentVelocity1 = 0;

// Encoder pins for Motor 2
#define ENC_A2 34
#define ENC_B2 35
volatile int encoderCount2 = 0;
volatile int currentVelocity2 = 0;

// Motor 1 pins
#define PWM_PIN      5
#define DIR_PIN_1    16
#define DIR_PIN_2    4

// Motor 2 pins
#define PWM_PIN_2    33
#define DIR_PIN_3    26
#define DIR_PIN_4    14

//motor specific constant
const int pulse_per_rev = 5000;
const float wheel_radius = 0.03;

// Access Point credentials
const char* ssid = "Irancell-TF-i60-437E_1";
const char* password = "kabiri123456";
WebServer server(80);

// imu object
MPU6050 mpu;
#define SDA_PIN 22  
#define SCL_PIN 23

const float ACCEL_SCALE = 16384.0;  // ±2g
const float GY_LRO_SCALE = 131.0;     // ±250°/s

// Raw sensor variables
int16_t ax_L, ay_L, az_L, gx_L, gy_L, gz_L;

// Offset variables
float ax_L_offset = 0, ay_L_offset = 0, az_L_offset = 0;
float gx_L_offset = 0, gy_L_offset = 0, gz_L_offset = 0;

//temporary for serial printing
float ax_L_print=0.0,ay_L_print=0.0,az_L_print=0.0,gx_L_print=0.0,gy_L_print=0.0,gz_L_print=0.0;

// Kalman filter states for each gy_Lro ax_Lis
struct KalmanGy_Lro {
  float rate = 0.0;
  float bias = 0.0;
  float P[2][2] = {{1, 0}, {0, 1}};
} kalX, kalY, kalZ;

float Q_angle = 0.001;
float Q_bias = 0.003;
float R_measure = 0.03;

//direction
bool direction_m1_forward = true;
bool direction_m2_forward = true;

//encoder
volatile int lastEncoderCount1 = 0;
volatile int lastEncoderCount2 = 0;

//desired speed
float desired_linear_speed_L = 0.0;
float desired_angular_speed_L = 0.0;
float desired_speed_motor1 = 0.0;
float desired_speed_motor2 = 0.0;

// PID control variables
float desired_speed = 0.0;
//set initial value for pid parameters
float Kp = 40.0, Ki = 0.01, Kd = 40.0 ;
//initial pwm
int initial_pr=64;
int initial_pl=64;


float error_m1 = 0.0;
float error_m2 = 0.0;
float pid_integral_m1 = 0.0;
float pid_integral_integral_m1 = 0.0;
float pid_integral_m2 = 0.0;
float pid_integral_integral_m2 = 0.0;
float last_error_m1 = 0.0;
float last_error_m2 = 0.0;




//odometry parameters
float x = 0.0;
float y = 0.0;
float theta = 0.0;
const float wheel_base = 0.4;

//all the timers
unsigned long lastEncoderCountTime = 0;
unsigned long last_pid_time = 0;//last time pid controller activated
unsigned long prevTime;
unsigned long last_odom_time = 0;

void IRAM_ATTR encoderISR1() {
  bool b = digitalRead(ENC_B);
  encoderCount1 += (b ? -1 : 1);
}

void IRAM_ATTR encoderISR2() {
  bool b = digitalRead(ENC_B2);
  encoderCount2 += (b ? -1 : 1);
}

void setup() {
  Serial.begin(115200);
  // Connect to existing Wi-Fi network
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  Wire.begin(SDA_PIN, SCL_PIN);
  delay(100);

  mpu.initialize();
  calibrateMPU();

  prevTime = millis();

  pinMode(DIR_PIN_1, OUTPUT);
  pinMode(DIR_PIN_2, OUTPUT);
  pinMode(DIR_PIN_3, OUTPUT);
  pinMode(DIR_PIN_4, OUTPUT);

  digitalWrite(DIR_PIN_1, HIGH);
  digitalWrite(DIR_PIN_2, LOW);
  digitalWrite(DIR_PIN_3, HIGH);
  digitalWrite(DIR_PIN_4, LOW);

  // Encoder Motor 1
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR1, RISING);

  // Encoder Motor 2
  pinMode(ENC_A2, INPUT_PULLUP);
  pinMode(ENC_B2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A2), encoderISR2, RISING);

  // Motor 1 MCPWM
  mcpwm_config_t pwm_config1;
  pwm_config1.frequency = 5000;
  pwm_config1.cmpr_a = 0.0;
  pwm_config1.counter_mode = MCPWM_UP_COUNTER;
  pwm_config1.duty_mode = MCPWM_DUTY_MODE_0;
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config1);

  // Motor 2 MCPWM
  mcpwm_config_t pwm_config2;
  pwm_config2.frequency = 5000;
  pwm_config2.cmpr_a = 0.0;
  pwm_config2.counter_mode = MCPWM_UP_COUNTER;
  pwm_config2.duty_mode = MCPWM_DUTY_MODE_0;
  mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_0, &pwm_config2); 

  // Setup PWM pins
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PWM_PIN);     // Motor 1
  mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0A, PWM_PIN_2);   // Motor 2
   
  Serial.println("ESP32 IP address: ");
  Serial.println(WiFi.localIP()); // Use this IP in your browser/PC scripts
  server.on("/v_desired_L",HTTP_GET, handle_v_desired_L);
  server.on("/status_L",HTTP_GET, handleStatus_L);
  server.on("/set_pid",HTTP_GET, handlePIDParams);
  server.on("/set_pwm_L",HTTP_GET, handlePwm_L);
  server.on("/imu_L",HTTP_GET,handle_imu_L); 
  server.begin();
}

void loop() {

  mpu.getMotion6(&ax_L, &ay_L, &az_L, &gx_L, &gy_L, &gz_L);
  
  // Time delta
  unsigned long currTime = millis();
  float dt = (currTime - prevTime) / 1000.0;
  prevTime = currTime;

  // Calibrated and scaled accelerometer values
  float ax_L_corr = (ax_L - ax_L_offset) / ACCEL_SCALE;
  float ay_L_corr = (ay_L - ay_L_offset) / ACCEL_SCALE;
  float az_L_corr = (az_L - az_L_offset) / ACCEL_SCALE;

  // Raw gy_Lro rates in °/s after calibration
  float gx_L_corr = (gx_L - gx_L_offset) / GY_LRO_SCALE;
  float gy_L_corr = (gy_L - gy_L_offset) / GY_LRO_SCALE;
  float gz_L_corr = (gz_L - gz_L_offset) / GY_LRO_SCALE;

  // Kalman filtered gy_Lro
  float filt_gx_L = kalman1D(gx_L_corr, dt, kalX);
  float filt_gy_L = kalman1D(gy_L_corr, dt, kalY);
  float filt_gz_L = kalman1D(gz_L_corr, dt, kalZ);

  float temp_ax_L = ax_L_corr;
  float temp_ay_L = ay_L_corr;
  ax_L_corr = -temp_ay_L;
  ay_L_corr = temp_ax_L;

  float temp_gx_L = filt_gx_L;
  float temp_gy_L = filt_gy_L;
  filt_gx_L = -temp_gy_L;
  filt_gy_L = temp_gx_L;

  ax_L_print=ax_L_corr;
  ay_L_print=ay_L_corr;
  az_L_print=az_L_corr;
  gx_L_print=filt_gx_L;
  gy_L_print=filt_gy_L;
  gz_L_print=filt_gz_L;
  
  server.handleClient();
  encoder_update();
  pid_control_motor();
  unsigned long now = millis();
  float dt_1 = (now - last_odom_time) / 1000.0; // dt in seconds
  //change for increasing speed  
  //if (dt_1 >= 0.1) {
  if (dt_1 >= 1/40) {
    float v_r = calc_vel_motor(currentVelocity1); // motor 1 = right
    float v_l = calc_vel_motor(currentVelocity2); // motor 2 = left
    update_odometry(v_l, v_r, dt_1);
    last_odom_time = now;
  }
  delay(10);
}

void encoder_update() {
  //change for increasing speed
  //if (millis() - lastEncoderCountTime >= 100) {

  if (millis() - lastEncoderCountTime >= 25) {
    int currentCount1 = encoderCount1;
    int delta1 = currentCount1 - lastEncoderCount1;
    currentVelocity1 = delta1 * 40;

    int currentCount2 = encoderCount2;
    int delta2 = currentCount2 - lastEncoderCount2;
    currentVelocity2 = delta2 * 40;

    lastEncoderCount1 = currentCount1;
    lastEncoderCount2 = currentCount2;
    lastEncoderCountTime = millis();
  }
}
float calc_vel_motor(int currentVelocity){
  //calculate velocity mtor (m/s)
  float linear_vel=(currentVelocity*2*3.1415*wheel_radius)/pulse_per_rev;
  return linear_vel;
}

//vr is motor 1 and vl is motor2
void update_odometry(float v_l, float v_r, float dt) {
  float v = (v_r + v_l) / 2.0;
  float omega = (v_r - v_l) / wheel_base;

  x += v * cos(theta) * dt;
  y += v * sin(theta) * dt;
  theta += omega * dt;

  // Normalize theta to [-π, π]
  if (theta > PI) theta -= 2 * PI;
  if (theta < -PI) theta += 2 * PI;
}

void pid_control_motor() {
  unsigned long now = millis();
  //change to increase speed
  //if (now - last_pid_time >= 100) {  // Run at 10 Hz note that in calculating integral terms we use *0.1

  if (now - last_pid_time >= 25) {  // Run at 40 Hz note that in calculating integral terms we use *0.1/4
    float measured_speed_m1 = calc_vel_motor(currentVelocity1); // m/s
    float measured_speed_m2 = calc_vel_motor(currentVelocity2); // m/s

    bool desired_m1_forward=(desired_speed_motor1 >= 0);
    bool desired_m2_forward=(desired_speed_motor2 >= 0);

    error_m1 = desired_speed_motor1 - measured_speed_m1;
    error_m2 = desired_speed_motor2 - measured_speed_m2;

    // PID calculations
    // ***************important *************
    pid_integral_m1 += error_m1*0.1/4; //this term is related to frequency   
    pid_integral_m2 += error_m2*0.1/4;//this term is related to frequency   
    
    // ***************important *************

    pid_integral_integral_m1+=pid_integral_m1*0.1/4;//this term is related to frequency   
    pid_integral_integral_m2+=pid_integral_m2*0.1/4;//this term is related to frequency   
    pid_integral_m1 = constrain(pid_integral_m1, -40.0, 40.0);  // tune bounds as needed
    pid_integral_m2 = constrain(pid_integral_m2, -40.0, 40.0);  // tune bounds as needed
    pid_integral_integral_m1 = constrain(pid_integral_integral_m1, -40.0, 40.0);  // tune bounds as needed
    pid_integral_integral_m2 = constrain(pid_integral_integral_m2, -40.0, 40.0);  // tune bounds as needed
    //float derivative = (error - last_error)/0.1;
    float output_m1 = Kp * pid_integral_m1 + Ki * pid_integral_integral_m1+ Kd * error_m1;
    float output_m2 = Kp * pid_integral_m2 + Ki * pid_integral_integral_m2+ Kd * error_m2;
 
    if (direction_m1_forward==true and desired_m1_forward==false){
      if (abs(measured_speed_m1)<0.05){
        direction_m1_forward = !direction_m1_forward;
        digitalWrite(DIR_PIN_1, direction_m1_forward ? HIGH : LOW);
        digitalWrite(DIR_PIN_2, direction_m1_forward ? LOW : HIGH);
        pid_integral_m1=0.0;
        pid_integral_integral_m1=0.0;
      }
      else{output_m1=0.0;}
    }
    if (direction_m1_forward==true and desired_m1_forward==true){
      output_m1=output_m1+initial_pr;
    }
    if (direction_m1_forward==false and desired_m1_forward==false){
      output_m1=-output_m1+initial_pr;
    }
    if (direction_m1_forward==false and desired_m1_forward==true){
      if (abs(measured_speed_m1)<0.05){
        direction_m1_forward = !direction_m1_forward;
        digitalWrite(DIR_PIN_1, direction_m1_forward ? HIGH : LOW);
        digitalWrite(DIR_PIN_2, direction_m1_forward ? LOW : HIGH);
        pid_integral_m1=0.0;
        pid_integral_integral_m1=0.0;
      }
      else{output_m1=0.0;}
    }    
    if (direction_m2_forward==true and desired_m2_forward==false){
      if (abs(measured_speed_m2)<0.05){
        direction_m2_forward = !direction_m2_forward;
        digitalWrite(DIR_PIN_3, direction_m2_forward ? HIGH : LOW);
        digitalWrite(DIR_PIN_4, direction_m2_forward ? LOW : HIGH);
        pid_integral_m2=0.0;
        pid_integral_integral_m2=0.0;
      }
      else{output_m2=0.0;}
    }
    if (direction_m2_forward==true and desired_m2_forward==true){
      output_m2=output_m2+initial_pr;
    }
    if (direction_m2_forward==false and desired_m2_forward==false){
      output_m2=-output_m2+initial_pl;
    }
    if (direction_m2_forward==false and desired_m2_forward==true){
      if (abs(measured_speed_m2)<0.05){
        direction_m2_forward = !direction_m2_forward;
        digitalWrite(DIR_PIN_3, direction_m2_forward ? HIGH : LOW);
        digitalWrite(DIR_PIN_4, direction_m2_forward ? LOW : HIGH);
        pid_integral_m2=0.0;
        pid_integral_integral_m2=0.0;
      }
      else{output_m2=0.0;}
    }   
    if (desired_speed_motor1==0.0){output_m1=0.0;}
    if (desired_speed_motor2==0.0){output_m2=0.0;}
    
    //output_m1=output_m1+initial_pr;
    //output_m2=output_m2+initial_pl;

    // Clamp output between 0 and 0.75 (max_L duty)
    output_m1 = constrain(output_m1, 0.0, 100.0);
    output_m2 = constrain(output_m2, 0.0, 100.0);
 
    // Update PWM
    float duty_cycle_m1 = output_m1 ;
    float duty_cycle_m2 = output_m2 ;
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty_cycle_m1);
    mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, duty_cycle_m2);
    last_error_m1 = error_m1;
    last_error_m2 = error_m2;
    last_pid_time = now;
  }
}

void calibrateMPU() {
  long ax_L_sum = 0, ay_L_sum = 0, az_L_sum = 0;
  long gx_L_sum = 0, gy_L_sum = 0, gz_L_sum = 0;

  for (int i = 0; i < 20000; i++) {
    mpu.getMotion6(&ax_L, &ay_L, &az_L, &gx_L, &gy_L, &gz_L);
    ax_L_sum += ax_L; ay_L_sum += ay_L; az_L_sum += az_L;
    gx_L_sum += gx_L; gy_L_sum += gy_L; gz_L_sum += gz_L;
    delay(5);
  }

  ax_L_offset = ax_L_sum / 20000.0;
  ay_L_offset = ay_L_sum / 20000.0;
  az_L_offset = (az_L_sum / 20000.0) - ACCEL_SCALE; // Should be 1g
  gx_L_offset = gx_L_sum / 20000.0;
  gy_L_offset = gy_L_sum / 20000.0;
  gz_L_offset = gz_L_sum / 20000.0;
}

float kalman1D(float newRate, float dt, KalmanGy_Lro &kal) {
  // Predict
  kal.rate -= dt * kal.bias;
  kal.P[0][0] += dt * (dt * kal.P[1][1] - kal.P[0][1] - kal.P[1][0] + Q_angle);
  kal.P[0][1] -= dt * kal.P[1][1];
  kal.P[1][0] -= dt * kal.P[1][1];
  kal.P[1][1] += Q_bias * dt;

  // Update
  float S = kal.P[0][0] + R_measure;
  float K0 = kal.P[0][0] / S;
  float K1 = kal.P[1][0] / S;

  float y = newRate - kal.rate;
  kal.rate += K0 * y;
  kal.bias += K1 * y;

  float P00_temp = kal.P[0][0];
  float P01_temp = kal.P[0][1];

  kal.P[0][0] -= K0 * P00_temp;
  kal.P[0][1] -= K0 * P01_temp;
  kal.P[1][0] -= K1 * P00_temp;
  kal.P[1][1] -= K1 * P01_temp;

  return kal.rate;
}
//server handlers
void handle_v_desired_L(){
  if (server.hasArg("desired_linear_velocity_L")){
    desired_linear_speed_L=server.arg("desired_linear_velocity_L").toFloat();
  }
  if (server.hasArg("desired_angular_velocity_L")){
    desired_angular_speed_L=server.arg("desired_angular_velocity_L").toFloat();
  }
  desired_speed_motor1=desired_linear_speed_L+(desired_angular_speed_L*wheel_base)/2.0;
  desired_speed_motor2=desired_linear_speed_L-(desired_angular_speed_L*wheel_base)/2.0;
  server.send(200, "text/plain", "Speed updated");
}

void handlePIDParams() {
  if (server.hasArg("kp")) {Kp = server.arg("kp").toFloat();}
  if (server.hasArg("ki")) {Ki = server.arg("ki").toFloat();}
  if (server.hasArg("kd")) {Kd = server.arg("kd").toFloat();}
  server.send(200, "text/plain", "PID parameters updated");
}

void handlePwm_L() {
  if (server.hasArg("initial_pr_L")) {initial_pr = server.arg("initial_pr_L").toFloat();}
  if (server.hasArg("initial_pl_L")) {initial_pl = server.arg("initial_pl_L").toFloat();}
  server.send(200, "text/plain", "PID parameters updated");
}

void handle_imu_L(){
  String json = "{";
  json += "\"ax_L\":" + String(ax_L_print, 3) + ",";
  json += "\"ay_L\":" + String(ay_L_print, 3) + ",";
  json += "\"az_L\":" + String(az_L_print, 3) + ",";
  json += "\"gx_L\":" + String(gx_L_print, 2) + ",";
  json += "\"gy_L\":" + String(gy_L_print, 2) + ",";
  json += "\"gz_L\":" + String(gz_L_print, 2) ;
  json += "}";
  server.send(200, "application/json", json);
}

void handleStatus_L() {
  String json = "{";
  json += "\"desired_speed_m1_L\":" + String(desired_speed_motor1) + ",";
  json += "\"actual_speed_m1_L\":" + String(calc_vel_motor(currentVelocity1)) + ",";
  json += "\"desired_speed_m2_L\":" + String(desired_speed_motor2) + ",";
  json += "\"actual_speed_m2_L\":" + String(calc_vel_motor(currentVelocity2)) + ",";
  json += "\"direction_m1\":\"" + String(direction_m1_forward ? "forward" : "reverse") + "\",";
  json += "\"direction_m2\":\"" + String(direction_m2_forward ? "forward" : "reverse") + "\",";
  json += "\"kp\":" + String(Kp) + ",";
  json += "\"ki\":" + String(Ki) + ",";
  json += "\"kd\":" + String(Kd);
  json += ",\"error_m1\":" + String(error_m1);
  json += ",\"int_m1\":" + String(pid_integral_m1);
  json += ",\"int_int_m1\":" + String(pid_integral_integral_m1);
  json += ",\"error_m2\":" + String(error_m2);
  json += ",\"int_m2\":" + String(pid_integral_m2);
  json += ",\"int_int_m2\":" + String(pid_integral_integral_m2);
  json += ",\"x\":" + String(x);
  json += ",\"y\":" + String(y);
  json += ",\"theta\":" + String((theta/3.14)*180);
  json += "}";
  server.send(200, "application/json", json);
}

