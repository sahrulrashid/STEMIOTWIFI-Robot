
#define MAKERLINE_AN  A0

#define MAX_SPEED 550

int PWMA = 5; //motor kiri
int PWMB = 4; //motor kanan
int DA = 0; //motor kiri
int DB = 2; //motor kanan

int motorB ; // motor kanan
int motorA ; // motor kiri

float Kp = 1.5; //ubah
float Kd = 5.0; //ubah

int adcMakerLine = 0;
int adcSetPoint = 0;
int proportional = 0;
int lastProportional = 0;
int derivative = 0;
int powerDifference = 0;
int motorLeft = 0;
int motorRight = 0;
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;
const int interval = 10;

void setup()
{
  pinMode(MAKERLINE_AN, INPUT);
  // analogReadResolution(10);

  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(DA, OUTPUT);
  pinMode(DB, OUTPUT);

  analogWrite(PWMA, 0);
  digitalWrite(DA, 0);
  analogWrite(PWMB, 0);
  digitalWrite(DB, 0);

  Serial.begin(115200);
  Serial.println("NodeMCU PD Line Following Robot with Maker Line");

  // Place robot at the center of line
  adcSetPoint = analogRead(MAKERLINE_AN);
  delay(2000);
}

void loop() {

  //--------LFR--------------

  adcMakerLine = analogRead(MAKERLINE_AN);

  proportional = adcMakerLine - adcSetPoint;

  derivative = proportional - lastProportional;

  lastProportional = proportional;

  powerDifference = (proportional * Kp) + (derivative * Kd);




  motorLeft = MAX_SPEED + powerDifference;
  motorRight = MAX_SPEED - powerDifference;

  motorLeft = constrain(motorLeft, 0, MAX_SPEED);
  motorRight = constrain(motorRight, 0, MAX_SPEED);

  digitalWrite(DA, LOW); //FORWARD
  analogWrite(PWMA, motorLeft);
  digitalWrite(DB, LOW); //FORWARD
  analogWrite(PWMB, motorRight);

}
