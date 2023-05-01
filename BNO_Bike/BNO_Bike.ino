#include <PID_v1.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <LiquidCrystal.h>

// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// these constants won't change.  But you can change the size of
// your LCD using them:
const int numRows = 2;
const int numCols = 16;


enum state {CALIBRATE, ON};
state cur_state = CALIBRATE;
int level = 0;
int weight = 150;

int inc_tgl = 0;
int dec_tgl = 0;
int on_tgl = 0;

int toggle_pin = 43;
int increase_pin = 45;
int decrease_pin = 47;

byte block[8] = {
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111
};

float ypr[3];
float turn;

int pwmPin = 31;
int in1Pin = 33;
int in2Pin = 35;

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
Adafruit_BNO055 bno2 = Adafruit_BNO055(-1, 0x29, &Wire);
int bno2_ADR = 23;

void setup(){

  Serial.begin(115200);
  while (!Serial);

  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  pinMode(bno2_ADR, OUTPUT);
  digitalWrite(bno2_ADR, HIGH);

  if(!bno2.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 2 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  bno.setExtCrystalUse(true);


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

  lcd.begin(numCols, numRows);
  pinMode(toggle_pin, INPUT);
  pinMode(increase_pin, INPUT);
  pinMode(decrease_pin, INPUT);

  attachInterrupt(digitalPinToInterrupt(18), handleIncrease, RISING);
  attachInterrupt(digitalPinToInterrupt(19), handleDecrease, RISING);

  Serial.println("Here");
  lcd.createChar(0, block);
  renderScreen();
}

void handleIncrease(){
  if (cur_state == CALIBRATE) {
    if (weight < 250){
      weight++;
    }
  } else if (cur_state == ON) {
    if (level < 5){
      level++;
    }
  }
  renderScreen();
}

void handleDecrease(){
  if (cur_state == CALIBRATE) {
    if (weight > 0){
      weight--;
    }
  } else if (cur_state == ON) {
    if (level > 0){
      level--;
    }
  }
  renderScreen();
}

void renderScreen(){
  lcd.clear();
  if (cur_state == CALIBRATE) {
    lcd.setCursor(0, 0);
    lcd.print("Status: OFF");
    lcd.setCursor(0, 1);
    lcd.print("Set weight: " + String(weight));

  } else if (cur_state == ON) {
    lcd.setCursor(0, 0);
    lcd.print("Status: ON");
    lcd.setCursor(0, 1);
    lcd.print("Level: " + String(level) + " [");
    for (int i = 0; i < 5; i++){
      if (i < level){
        lcd.write(byte(0));
      } else {
        lcd.print(" ");
      }
    }
    lcd.print("]");
  }
}

bool primary_idx = 0;
Adafruit_BNO055 sensors[2] = {bno, bno2};

void balance(){

  double ang;

  imu::Vector<3> euler = sensors[primary_idx].getVector(Adafruit_BNO055::VECTOR_EULER);
  ang = euler.z();
  if (ang == 0.0){
    euler = sensors[1 - primary_idx].getVector(Adafruit_BNO055::VECTOR_EULER);
    ang = euler.y();
    primary_idx = 1 - primary_idx;
    Serial.println("swap");
    Serial.println(ang);
  }
  
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
  Serial.print(ang);
  Serial.print(" ");
  Serial.println(primary_idx);
  //delay(10);
}

void loop() {
  //Serial.println("loop");
  // put your main code here, to run repeatedly:
  int toggle_val = digitalRead(toggle_pin);

  if (cur_state == CALIBRATE) {
    if (toggle_val){
      cur_state = ON;
      /// Swicting back to off, turn motor off here
      level = 0;
      renderScreen();
      return;
    }

  } else if (cur_state == ON) {
    if (toggle_val){
      cur_state = CALIBRATE;
      renderScreen();
      return;
    }
    //Serial.println("Balance");
    balance();
  }
}
