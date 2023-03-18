#include <PID_v1.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

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


#define BNO055_SAMPLERATE_DELAY_MS (50)
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);


void setup(){

  Serial.begin(115200);
  while (!Serial);

  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");

  bno.setExtCrystalUse(true);

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

  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  double ang = euler.y();

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
