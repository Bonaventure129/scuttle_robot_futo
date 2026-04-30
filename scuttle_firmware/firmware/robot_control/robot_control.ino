#include <ams_as5048b.h>
#include <Wire.h>

// --- ARDUINO UNO PINS (HW-231) ---
// LEFT Motor (PWM Pins 3 & 9)
const int L_IN1 = 9;  
const int L_IN2 = 3;

// RIGHT Motor (PWM Pins 11 & 10)
const int R_IN1 = 10;  
const int R_IN2 = 11;

// --- SENSORS ---
AMS_AS5048B encL(0x40);
AMS_AS5048B encR(0x41);
const float GEAR_RATIO = 0.5;
#define U_DEG 3

// --- TUNING VALUES ---
// Tamed the PID to prevent violent jerking and wheel slip
double Kp_L = 1.2, Ki_L = 0.01, Kd_L = 0.05;
double Kp_R = 1.2, Ki_R = 0.01, Kd_R = 0.05;

// --- UNIT CONVERSION ---
const float RPM_TO_RADS = 0.104719755;
const float RADS_TO_RPM = 9.54929658;

// --- SERIAL PARSING STATE ---
bool is_right_wheel_cmd = false;
bool is_left_wheel_cmd = false;
bool is_right_wheel_forward = true;
bool is_left_wheel_forward = true;
char value[] = "00.00";
uint8_t value_idx = 0;
bool is_cmd_complete = false;

// --- CONTROL VARIABLES ---
// Targets in Radians/Sec (From ROS)
double target_rads_L = 0.0;
double target_rads_R = 0.0;

// Internal PID variables
unsigned long prevTime = 0;
double prevAngL=0, prevTotL=0; long rotL=0;
double prevAngR=0, prevTotR=0; long rotR=0;
double integL=0, lastErrL=0;
double integR=0, lastErrR=0;

// Smoothing Buffers
const int AVG=4;
double bufL[AVG]={0}, bufR[AVG]={0}; 
int idxL=0, idxR=0; 

void setup() {
  Serial.begin(115200);
  
  // Prevent Arduino from freezing when reading ROS commands
  Serial.setTimeout(10); 
  
  pinMode(L_IN1, OUTPUT); pinMode(L_IN2, OUTPUT);
  pinMode(R_IN1, OUTPUT); pinMode(R_IN2, OUTPUT);
  
  // Stop Motors initially
  digitalWrite(L_IN1, LOW); digitalWrite(L_IN2, LOW);
  digitalWrite(R_IN1, LOW); digitalWrite(R_IN2, LOW);
  
  encL.begin(); encR.begin();
  encL.setClockWise(true); encR.setClockWise(false);
  
  encL.updateMovingAvgExp(); encR.updateMovingAvgExp();
  prevAngL = encL.angleR(U_DEG, false);
  prevAngR = encR.angleR(U_DEG, false);
  prevTotL = prevAngL; prevTotR = prevAngR;
}

void loop() {
  // -------------------------------------------------
  // 1. SERIAL COMMAND PARSER (FIXED)
  // -------------------------------------------------
  // Use 'while' to drain the entire buffer instantly
  while (Serial.available()) {
    char chr = Serial.read();
    
    // CRITICAL: Ignore invisible characters that corrupt the math!
    if (chr == '\n' || chr == '\r' || chr == ' ') continue;
    
    if(chr == 'r') {
      is_right_wheel_cmd = true; is_left_wheel_cmd = false;
      value_idx = 0; is_cmd_complete = false;
    }
    else if(chr == 'l') {
      is_right_wheel_cmd = false; is_left_wheel_cmd = true;
      value_idx = 0;
    }
    else if(chr == 'p') {
      if(is_right_wheel_cmd) is_right_wheel_forward = true;
      else if(is_left_wheel_cmd) is_left_wheel_forward = true;
    }
    else if(chr == 'n') {
      if(is_right_wheel_cmd) is_right_wheel_forward = false;
      else if(is_left_wheel_cmd) is_left_wheel_forward = false;
    }
    else if(chr == ',') {
      if(is_right_wheel_cmd) {
        target_rads_R = atof(value);
        if(!is_right_wheel_forward) target_rads_R *= -1.0;
      }
      else if(is_left_wheel_cmd) {
        target_rads_L = atof(value);
        if(!is_left_wheel_forward) target_rads_L *= -1.0;
        is_cmd_complete = true;
      }
      // Reset buffer
      value_idx = 0;
      memset(value, 0, sizeof(value));
      strcpy(value, "00.00");
    }
    else {
      if(value_idx < 5) {
        value[value_idx] = chr;
        value_idx++;
      }
    }
  }

  // -------------------------------------------------
  // 2. CONTROL LOOP (50ms / 20Hz)
  // -------------------------------------------------
  unsigned long now = millis();
  if(now - prevTime >= 50) { 
    double dt = (now - prevTime)/1000.0;
    prevTime = now;

    // --- A. READ SENSORS (Get RPM) ---
    double rpmL = getRPM(dt, &encL, &prevAngL, &prevTotL, &rotL, bufL, &idxL);
    double rpmR = getRPM(dt, &encR, &prevAngR, &prevTotR, &rotR, bufR, &idxR);

    // --- B. CONVERT TARGETS (Rad/s -> RPM) ---
    double targetRPM_L = target_rads_L * RADS_TO_RPM;
    double targetRPM_R = target_rads_R * RADS_TO_RPM;

    // --- C. RUN PID ---
    double pwmL = runPID(targetRPM_L, rpmL, dt, &integL, &lastErrL, Kp_L, Ki_L, Kd_L);
    double pwmR = runPID(targetRPM_R, rpmR, dt, &integR, &lastErrR, Kp_R, Ki_R, Kd_R);

    // --- D. APPLY TO MOTORS ---
    if(target_rads_L == 0) { setHW231Motor(L_IN1, L_IN2, 0); integL = 0; }
    else setHW231Motor(L_IN1, L_IN2, pwmL);

    if(target_rads_R == 0) { setHW231Motor(R_IN1, R_IN2, 0); integR = 0; }
    else setHW231Motor(R_IN1, R_IN2, pwmR);

    // --- E. SEND FEEDBACK TO ROS (RPM -> Rad/s) ---
    double meas_rads_L = rpmL * RPM_TO_RADS;
    double meas_rads_R = rpmR * RPM_TO_RADS;

    // DEADBAND FILTER to stop sensor noise
    if (abs(meas_rads_L) < 0.05) meas_rads_L = 0.0;
    if (abs(meas_rads_R) < 0.05) meas_rads_R = 0.0;

    // Corrected positive/negative string logic
    String r_sign = (meas_rads_R >= 0) ? "p" : "n";
    String l_sign = (meas_rads_L >= 0) ? "p" : "n";

    // Format: "rp03.14,ln02.50,"
    Serial.print("r");
    Serial.print(r_sign);
    if (abs(meas_rads_R) < 10.0) Serial.print("0");
    Serial.print(abs(meas_rads_R), 2);
    
    Serial.print(",l");
    Serial.print(l_sign);
    if (abs(meas_rads_L) < 10.0) Serial.print("0");
    Serial.print(abs(meas_rads_L), 2);
    Serial.println(",");
  }
}

// -------------------------------------------------
// HELPERS
// -------------------------------------------------
double getRPM(double dt, AMS_AS5048B *enc, double *prevAng, double *prevTot, long *rot, double *buf, int *wheel_idx) {
  enc->updateMovingAvgExp();
  double cur = enc->angleR(U_DEG, false);
  
  if(cur < 90 && *prevAng > 270) (*rot)++;
  else if(cur > 270 && *prevAng < 90) (*rot)--;
  
  *prevAng = cur;
  double tot = (*rot * 360.0) + cur;
  double delta = (tot - *prevTot) * GEAR_RATIO;
  *prevTot = tot;
  
  double raw = (delta / 360.0) / (dt / 60.0);
  
  buf[*wheel_idx] = raw;
  *wheel_idx = (*wheel_idx + 1) % AVG;
  
  double avg = 0;
  for(int i=0; i<AVG; i++) avg += buf[i];
  return avg / AVG;
}

double runPID(double target, double current, double dt, double *integral, double *lastErr, double Kp, double Ki, double Kd) {
  double error = target - current;
  *integral += error * dt;
  *integral = constrain(*integral, -255, 255); 
  double deriv = (error - *lastErr) / dt;
  *lastErr = error;
  return (Kp * error) + (Ki * *integral) + (Kd * deriv);
}

void setHW231Motor(int pinA, int pinB, double pwmInput) {
  int pwm = constrain((int)abs(pwmInput), 0, 255);
  
  // --- THE STICTION FIX ---
  // Lowered slightly to 65 to prevent the robot from jumping and slipping
  if (pwm > 0 && pwm < 65) {
      pwm = 65; 
  }
  // ------------------------

  if (pwmInput > 0) { 
    analogWrite(pinA, pwm); digitalWrite(pinB, LOW); 
  } else if (pwmInput < 0) { 
    digitalWrite(pinA, LOW); analogWrite(pinB, pwm); 
  } else { 
    digitalWrite(pinA, LOW); digitalWrite(pinB, LOW); 
  }
}