

float ypr[3];
float turn;


void setup(){

  Serial.begin(115200);
  while (!Serial);

  mpu_setup();

  pinMode(A0, INPUT);

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

  //read_angles();
  read_turn(); // global ypr populated
  //Serial.println(ypr[1]);
  delay(100);

}