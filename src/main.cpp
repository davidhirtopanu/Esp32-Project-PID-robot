#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <I2Cdev.h>

//Objects
MPU6050 mpu;


//Pins 
const int LEDpin = 4;
const int Buttonpin = 13;
//Motor pins
const int ENA = 14;
const int IN1 = 26;
const int IN2 = 27;

const int ENB = 18;
const int IN3 = 33;
const int IN4 = 19;

const int pwmChannelA = 0;
const int pwmChannelB = 1;

//Variables

float gyro_pitch = 0;
float gyro_roll = 0;
boolean isOn = false;
int16_t ax, ay, az, gx, gy, gz;

//Kalman filter variables
float KalmanPitch = 0;
float KalmanRoll = 0;
float KalmanPitchUnc = 2*2;
float KalmanRollUnc = 2*2;
float KalmanOutputs[] = {0, 0};

//PID variables - Roll
float target_roll = 7.5; //7.5
float Kp_roll = 6.0; //6.0
float Ki_roll = 0.15; //0.15
float Kd_roll = 0.015; //0.015

//PID variables - Yaw rate
/*
float target_yaw_rate = 0.0;
float Kp_yaw = 0.0;
float Ki_yaw =0;
float Kd_yaw = 0;
*/
unsigned long lastTime = 0;



//Structures:
struct PIDcontroller{
  float Kp, Ki, Kd;
  float lastError = 0;
  float integral = 0;
  bool firstRun = true;
  float P = 0, I = 0, D = 0;


    float calculate(float target, float current, float dt){
      float error = target - current;
      P = Kp * error;
      integral += error *dt;
      
      I = Ki * integral;
      if (firstRun){
        lastError = error;
        firstRun = false;
      }
      D = Kd * (error - lastError)/dt;

      
      lastError = error;
      return P + I + D;
    }
  
  void reset(){
    lastError = 0;
    integral = 0;
    firstRun = true;
  }

};

  PIDcontroller rollPID;
  //PIDcontroller yawRatePID;


//Function declarations
void KalmanFilter(float KalmanState, float KalmanUnc, float KalmanInput, float KalmanMeasurement, float dt);
void driveMotor(float leftMotor, float rightMotor);
float map_float(float value, float min_in, float max_in, float min_out, float max_out);

void setup() {

  rollPID.Kp = Kp_roll;
  rollPID.Ki = Ki_roll;
  rollPID.Kd = Kd_roll;
  //yawRatePID.Kp = Kp_yaw;
  //yawRatePID.Ki = Ki_yaw;
  //yawRatePID.Kd = Kd_yaw;


  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  digitalWrite(ENA, LOW);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(ENB, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  ledcSetup(pwmChannelA, 2000, 11);
  ledcAttachPin(ENA, pwmChannelA);
  ledcSetup(pwmChannelB, 2000, 11);
  ledcAttachPin(ENB, pwmChannelB);

  Serial.begin(115200);
  Wire.begin(21, 22);
  Wire.setClock(400000); 

  pinMode(LEDpin, OUTPUT);
  pinMode(Buttonpin, INPUT_PULLUP);
  digitalWrite(LEDpin, LOW);

  mpu.initialize();
  


  if(mpu.testConnection()){
    Serial.println("MPU connected");
  }else{
    Serial.println("MPU not connected");
  }

  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_1000);
    
  lastTime = micros();
}

void loop() {
  
  static boolean lastButtonState = HIGH;
  boolean currentButtonState = digitalRead(Buttonpin);

  // Detect button press transition (HIGH to LOW)
  if (currentButtonState == LOW && lastButtonState == HIGH) {
    isOn = !isOn; // Flip state
    digitalWrite(LEDpin, isOn ? HIGH : LOW);

    if (isOn){
      lastTime = micros();

      rollPID.reset();
      //yawRatePID.reset();
    }
  }

  lastButtonState = currentButtonState;

   mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    unsigned long currentTime = micros();
    float dt = (currentTime - lastTime) / 1000000.0f;
    lastTime = currentTime;

    float accelx = ((float)ax)/4096.0;  
    float accely = ((float)ay)/4096.0;
    float accelz = ((float)az)/4096.0;

    float gyrox = (((float)gx)/32.8f)+1;
    float gyroy = (((float)gy)/32.8f) + 2.5; 
    float gyroz = (((float)gz)/32.8f);

    float pitch_A = (atan2(accely , sqrt(accelx * accelx + accelz * accelz)) * 180.0 / PI) - (-1.638);
    float roll_A = (-atan2(accelx ,sqrt(accely * accely + accelz * accelz)) * 180.0 / PI) - (-1.805);
    gyro_roll += gyroy * dt; 
    gyro_pitch += gyrox * dt;

    //Pitch angle calculation
    KalmanFilter(KalmanPitch, KalmanPitchUnc, gyrox, pitch_A, dt);
    KalmanPitch = KalmanOutputs[0];
    KalmanPitchUnc = KalmanOutputs[1];

    //Roll angle calculation
    KalmanFilter(KalmanRoll, KalmanRollUnc, gyroy, roll_A, dt);
    KalmanRoll = KalmanOutputs[0];
    KalmanRollUnc = KalmanOutputs[1];
  
  if (isOn) {

    //minimum PWM value for motors is 1700, but aim for higher value for reliablility (about 1800)

    //Serial.printf("Accel Pitch: %.2f, Accel Roll: %.2f\t", pitch_A, roll_A);
    //Serial.printf("Gyro Pitch: %.2f, Gyro Roll: %.2f\n", gyro_pitch, gyro_roll);    

    float rollOutput = rollPID.calculate(target_roll, KalmanRoll, dt);
    //float yawRateOutput = yawRatePID.calculate(target_yaw_rate, gyroz, dt);
    //Serial.printf("dt: %.4f | Roll P: %.2f, I: %.2f, D: %.2f\n", dt, rollPID.P, rollPID.I, rollPID.D);

    //Forward -> Negative KalmanRoll, IN1 HIGH, IN2 LOW, IN3 LOW, IN4 HIGH
    //Clockwise yaw -> Positive gyroz, IN1 LOW, IN2 HIGH, IN3 LOW, IN4 HIGH

    float leftMotor = rollOutput; //- yawRateOutput;
    float rightMotor = rollOutput; //+ yawRateOutput;

    driveMotor(leftMotor, rightMotor);


    //Serial.printf("Kalman Pitch: %.2f, Kalman Roll: %.2f, Yaw rate: %.2f\n", KalmanPitch, KalmanRoll, gyroz);
    
  }else{
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    ledcWrite(pwmChannelA, 0);
    ledcWrite(pwmChannelB, 0);

    rollPID.reset();
    //yawRatePID.reset();
  }
 
} 

void KalmanFilter(float KalmanState, float KalmanUnc, float KalmanInput, float KalmanMeasurement, float dt){
  /*
  KalmanState = The angle
  KalmanUnc = The uncertainty of the angle
  KalmanInput = The gyro rate
  KalmanMeasurement = The angle from the accelerometer
  dt = The time difference between the last and current measurement
  */
  KalmanState += dt*KalmanInput;
  KalmanUnc += dt*dt*4*4;


  float KalmanGain = KalmanUnc/(KalmanUnc + 3*3);
  KalmanState += KalmanGain*(KalmanMeasurement - KalmanState);
  KalmanUnc *= (1 - KalmanGain);

  KalmanOutputs[0] = KalmanState;
  KalmanOutputs[1] = KalmanUnc;
}


void driveMotor(float leftMotor, float rightMotor){
  if (rightMotor > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else if (rightMotor < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }else{
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }

  if (leftMotor > 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else if (leftMotor < 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }else{
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }

  //Serial.printf("Left Motor: %.2f, Right Motor: %.2f\n", leftMotor, rightMotor);
  

  if (abs(leftMotor) < 1){
    ledcWrite(pwmChannelB, 0);
  }else{
    leftMotor = map_float(abs(leftMotor), 1, 100, 1800, 2047);
    ledcWrite(pwmChannelB, constrain(leftMotor, 1800, 2047));
  }
  if (abs(rightMotor) < 1){
    ledcWrite(pwmChannelA, 0);
  }else{
    rightMotor = map_float(abs(rightMotor), 1, 100, 1800, 2047);
    ledcWrite(pwmChannelA, constrain(rightMotor, 1800, 2047));
  }
  
 // Serial.printf("PWM Left: %lu, PWM Right: %lu\n", ledcRead(pwmChannelB), ledcRead(pwmChannelA));

}


float map_float(float value, float min_in, float max_in, float min_out, float max_out){

  float result = (max_out - min_out) * ((value - min_in)/(max_in-min_in)) + min_out;
  return result;
}


