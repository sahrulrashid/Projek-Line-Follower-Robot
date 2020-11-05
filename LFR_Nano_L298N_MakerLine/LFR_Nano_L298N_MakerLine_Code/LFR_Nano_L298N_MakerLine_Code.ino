
#define MAKERLINE_AN  A0 // MAKERLINE AN pin ke A0 Nano

#define MAX_SPEED 550 // kalau ubah ni, kena ubah semula Kp dan Kd

//left wheel
int pwm1 = 10;
int in1 = 9;
int in2 = 8;
//right wheel
int pwm2 = 5;
int in3 = 7;
int in4 = 6;


//PID SETTING
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

  //left wheel
  pinMode(pwm1, OUTPUT); //PWM-Pin
  pinMode(in1, OUTPUT); //IN1-Pin
  pinMode(in2, OUTPUT); //IN2-Pin
  //right wheel
  pinMode(pwm2, OUTPUT); //PWM-Pin
  pinMode(in3, OUTPUT); //IN1-Pin
  pinMode(in4, OUTPUT); //IN2-Pin

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

  analogWrite(pwm1, motorLeft);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);

  analogWrite(pwm2, motorRight);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

}
