
#define MAKERLINE_AN  A0

#define MAX_SPEED 550

int PWMA = 5; //motor kiri
int PWMB = 4; //motor kanan
int DA = 0; //motor kiri
int DB = 2; //motor kanan

int motorB ; // motor kanan
int motorA ; // motor kiri


int adcMakerLine = 0;
int adcSetPoint = 0;
int proportional = 0.11;
int lastProportional = 0;
int derivative = 8;
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

void loop()
{
  currentMillis = millis();
  if (currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;

    adcMakerLine = analogRead(MAKERLINE_AN);

  /*  if (adcMakerLine < 51) { // Out of line
      robotMove(0, 0);
    }
    else */ if (adcMakerLine > 972) { // Detects cross line
      robotMove(MAX_SPEED - 25, MAX_SPEED - 25);
    }
    else {
      proportional = adcMakerLine - adcSetPoint;
      derivative = proportional - lastProportional;
      lastProportional = proportional;

      powerDifference = (proportional * 1.5) + (derivative * 5);

      if (powerDifference > MAX_SPEED) {
        powerDifference = MAX_SPEED;
      }
      if (powerDifference < -MAX_SPEED) {
        powerDifference = -MAX_SPEED;
      }

      if (powerDifference < 0) {
        motorLeft = MAX_SPEED + powerDifference;
        motorRight = MAX_SPEED;
      }
      else {
        motorLeft = MAX_SPEED;
        motorRight = MAX_SPEED - powerDifference;
      }

      robotMove(motorLeft, motorRight);

      Serial.print("ADC:\t");
      Serial.print(adcMakerLine);
      Serial.print("\tMotor Left:\t");
      Serial.print(motorLeft);
      Serial.print("\tMotor Right:\t");
      Serial.println(motorRight);
      Serial.print("\tPD:\t");
      Serial.println(powerDifference);
            Serial.print("\tST:\t");
      Serial.println(adcSetPoint);
    }
  }
}

void robotMove(int speedLeft, int speedRight)
{
  speedLeft = constrain(speedLeft, -1023, 1023);
  speedRight = constrain(speedRight, -1023, 1023);

  if (speedLeft > 0) {
    digitalWrite(DA, LOW); //FORWARD
  }
  else {
    digitalWrite(DA, HIGH); //REVERSE
  }

  if (speedRight > 0) {
    digitalWrite(DB, LOW); //FORWARD
  }
  else {
    digitalWrite(DB, HIGH); // REVERSE
  }
  analogWrite(PWMA, abs(speedLeft));
  analogWrite(PWMB, abs(speedRight));
}
