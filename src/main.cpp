#include <Arduino.h>
#include <PID_v1.h>

// Pins
const int SENSORPIN = 5;    
const int PWMPIN    = 10;   
const int PIN1A     = 11;   
const int PIN2A     = 12;   

//  Tolerances  for wheel slot detection
const int lightThresh = 2500;               
const int darkThresh  = 1500;            

const unsigned long stall      = 1000000;   // Motor stall timeout
const unsigned long sampleTime = 100;       

const int WHEELSLOTS = 10;

unsigned long pulseCount = 0;
unsigned long pulseLast  = 0;

int  sensorRaw = 0;
bool slotSight = false;


// Get sensor data
void sense() {
  sensorRaw = analogRead(SENSORPIN);
  if (!slotSight && sensorRaw > lightThresh) {
    slotSight = true;            
    pulseCount++;
    pulseLast = micros();
  } else if (slotSight && sensorRaw < darkThresh) {
    slotSight = false;          
  }
}

unsigned long prevCount = 0;
unsigned long pulsePrev = 0;

float rpm = 0;

// Calculate RPM from sensor data
float findRPM() {
  unsigned long pulseNew = pulseCount - prevCount;
  if (pulseNew > 0) {
    unsigned long dt = pulseLast - pulsePrev;
    if (dt > 0 && pulsePrev != 0) {
      float revolutions = pulseNew / (float)WHEELSLOTS;
      rpm = revolutions * 60000000.0f / dt;
    }
    prevCount = pulseCount;
    pulsePrev = pulseLast;
  } else if (micros() - pulseLast > stall) {
    rpm = 0;
  }
  return rpm;
}

// Speed setting function
void motorRun(int pwm) {
  pwm = constrain(pwm, 0, 255);
  digitalWrite(PIN1A, HIGH);     
  digitalWrite(PIN2A, LOW);
  analogWrite(PWMPIN, pwm);
}

double setPoint = 0;
double input    = 0;
double output   = 0;
double kP = 0.6;
double kI = 2.0;
double kD = 0.0;
PID pid(&input, &output, &setPoint, kP, kI, kD, DIRECT);

// SETUP ////////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(20);

  pinMode(PIN1A, OUTPUT);
  pinMode(PIN2A, OUTPUT);
  pinMode(PWMPIN, OUTPUT);
  motorRun(0);

  analogReadResolution(12);
  pid.SetSampleTime(sampleTime);
  pid.SetOutputLimits(0, 255);
  pid.SetMode(AUTOMATIC);
}

// LOOP ////////////////////////////////////////////////////////////////////
void loop() {
  unsigned long now = millis();

  // Take reading every 1ms
  static unsigned long lastSense = 0;
  if (now - lastSense >= 1) {
    lastSense = now;
    sense();
  }

  // Enter speed by serial prompt
  if (Serial.available()) {
    setPoint = Serial.readStringUntil('\n').toFloat();
  }

  // Measure speed and adjust as needed
  static unsigned long lastSample = 0;
  if (now - lastSample >= sampleTime) {
    lastSample = now;
    input = findRPM();

    if (setPoint <= 0) {  // Motor stopped
      pid.SetMode(MANUAL);
      output = 0;
    } else {                     // PID control
      pid.SetMode(AUTOMATIC);
      pid.Compute();
    }
    motorRun((int)output);

    Serial.print("Setpoint:");
    Serial.println(setPoint);
    Serial.print("RPM:");
    Serial.println(input);
    Serial.print("Sensor:");
    Serial.println(sensorRaw);
    Serial.print("PWM:");
    Serial.println(output);
  }
}