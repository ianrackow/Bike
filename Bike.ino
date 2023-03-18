#include <PID_v1.h>

float ypr[3];
float turn;

int pwmPin = 5;
int in1Pin = 6;
int in2Pin = 7;

int minVal = 265;
int maxVal = 402;

double x;
double y;
double z;

double kp = 50;
double ki = 40;
double kd = 5;

double setpoint, input, output;

double accum;

PID pid(&input, &output, &setpoint, kp, ki, kd, DIRECT);


void setup(){

  Serial.begin(115200);
  while (!Serial);

  mpu_setup();

  pinMode(A0, INPUT);

  pinMode(pwmPin, OUTPUT);
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, LOW);

  setpoint = 0.0;
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-255, 255);
  pid.SetSampleTime(10);
  input = 0;

  accum = 0;

}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void read_turn(){

  int potRaw = analogRead(A0);
  Serial.print("Pot: ");
  Serial.println(potRaw);
  // Remap this on bike
  float rads = mapfloat(potRaw, 0.0, 1023.0, -30.0, 30.0);
  turn = rads;

}

void loop(){

  read_angles(); // global ypr populated

  double ang = ypr[0] * 180.0/M_PI;

  input = ang;

  setpoint = 0 - accum;
  accum = accum - (output / 150);
  accum = constrain(accum,-0.6,0.6);

  pid.Compute();

  if (output > 0) {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
    analogWrite(pwmPin, abs(output));
    accum -= 0.05;
  } else {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
    analogWrite(pwmPin, abs(output));
    accum += 0.05;
  }


  //read_turn(); 
  Serial.println(ang);
  //delay(10);
}
