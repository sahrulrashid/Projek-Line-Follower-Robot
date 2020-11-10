

#include <LiquidCrystal_I2C.h> // Library for LCD
LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 16, 2); // Change to (0x27,16,2) for 16x2 LCD.

#define MAKERLINE_AN  A0 // MAKERLINE AN pin ke A0 Nano

#define MAX_SPEED 150 // kalau ubah ni, kena ubah semula Kp dan Kd

//left wheel
int pwm1 = 10;
int in1 = 9;
int in2 = 8;
//right wheel
int pwm2 = 5;
int in3 = 7;
int in4 = 6;


//PID SETTING
float Kp = 0; //ubah
float Kd = 0; //ubah

//potentiomwter
int pinPP = A7;    //pin Analog 0 for the input of the potentiometer
int pinPD = A6;    //pin Analog 0 for the input of the potentiometer

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
const int interval = 500;

void setup()
{
  pinMode(MAKERLINE_AN, INPUT);

  //potentiomwter
  pinMode (pinPP, INPUT);   //Pin Setup for potentiometer
  pinMode (pinPD, INPUT);   //Pin Setup for potentiometer

  //left wheel
  pinMode(pwm1, OUTPUT); //PWM-Pin
  pinMode(in1, OUTPUT); //IN1-Pin
  pinMode(in2, OUTPUT); //IN2-Pin
  //right wheel
  pinMode(pwm2, OUTPUT); //PWM-Pin
  pinMode(in3, OUTPUT); //IN1-Pin
  pinMode(in4, OUTPUT); //IN2-Pin

  Serial.begin(9600);
  Serial.println("PD Line Following Robot with Maker Line");
  lcd.init(); // initialize the lcd
  lcd.backlight();

  lcd.setCursor(0, 0);         // move cursor to   (0, 0)
  lcd.print("PD LFR by");        // print message at (0, 0)
  lcd.setCursor(2, 1);         // move cursor to   (2, 1)
  lcd.print("Sahrul Rashid"); // print message at (2, 1)
  // lcd.clear();

  // Place robot at the center of line
  adcSetPoint = analogRead(MAKERLINE_AN);
  delay(2000);
  lcd.clear();

}

void LCDRefresh() {

  currentMillis = millis();
  if (currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;

    // Serial.print("Kd: ");
    // Serial.println(Kd);

    //  Serial.print("Kp: ");
    // Serial.print(Kp);

    // Serial.print("\t");

    //Serial.print("ADC:\t");
    // Serial.print(adcMakerLine);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Kp: ");
    lcd.print(Kp);
    lcd.setCursor(0, 1);
    lcd.print("Kd:  ");
    lcd.print(Kd);
  }
}

void loop() {

  LCDRefresh();

  Kp = analogRead(pinPP);
  Kp = Kp / 100;
  Kd = analogRead(pinPD);
  Kd = Kd / 50;



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

  analogWrite(pwm1, motorLeft);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);

  analogWrite(pwm2, motorRight);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

}