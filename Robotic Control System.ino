#include <Wire.h>
#define CMPS14_Address 0x60
#include <LiquidCrystal.h>
LiquidCrystal lcd(37, 36, 35, 34, 33, 32);
int joystickButtonPin = 19;
bool buttonPressed = false;
#define JOY_X_PIN A0
#define JOY_Y_PIN A1
#define Motor_forward 1
#define Motor_return 0
#define Motor_L_dir_pin 7
#define Motor_R_dir_pin 8
#define Motor_L_pwm_pin 9
#define Motor_R_pwm_pin 10
#define ENC_RIGHT_A 23
#define ENC_RIGHT_B 2
#define ENC_LEFT_A 24
#define ENC_LEFT_B 3
#define PPR 176
#define WHEEL_RADIUS_CM 2.1
int iL = 0, iR = 0, pulseCount = 0, distance = 0;
int intDist = 0;
float pulsesForStatcm = 0.0;
float wheelDistanceRevolution = 2.0 * PI * WHEEL_RADIUS_CM;
byte raw;
byte target_raw = 0;
int old_degree = 0;
int new_degree = 0;
bool flag = false;
int degree;
int pwm_R = 0;
int pwm_L = 0;

void setup() {
  Wire.begin();
  lcd.begin(20, 4);

  pinMode(Motor_L_dir_pin, OUTPUT);
  pinMode(Motor_R_dir_pin, OUTPUT);
  pinMode(Motor_L_pwm_pin, OUTPUT);
  pinMode(Motor_R_pwm_pin, OUTPUT);
  pinMode(ENC_RIGHT_A, INPUT);
  pinMode(ENC_RIGHT_B, INPUT);
  pinMode(ENC_LEFT_A, INPUT);
  pinMode(ENC_LEFT_B, INPUT);
  pinMode(JOY_X_PIN, INPUT);
  pinMode(JOY_Y_PIN, INPUT);
  pinMode(joystickButtonPin, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(joystickButtonPin), buttonhPressedISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_B), pulseCountForLeftWheel, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_B), pulseCountForRightWheel, FALLING);
  Serial.begin(9600);
}

void loop() {
  pwm_R = 0;
  pwm_L = 0;
  // Read compass data and display direction
  readCompass();

  // Control the robot with joystick or serial commands
  if (buttonPressed) {
    controlWithJoystick();
  } else {
    controlWithWebBrowser();
  }

  // Write PWM values to motors
  analogWrite(Motor_L_pwm_pin, pwm_L);
  analogWrite(Motor_R_pwm_pin, pwm_R);
}

void readCompass() {
  // Request and read data from compass sensor
  Wire.beginTransmission(CMPS14_Address);
  Wire.write(1);
  Wire.endTransmission();
  Wire.requestFrom(CMPS14_Address, 1, true);

  if (Wire.available() >= 1) {
    raw = Wire.read();  // Read the 8-bit bearing value
    displayToLcd(raw);  // Display direction on LCD
  }
}

void displayToLcd(byte raw) {
  // Clear LCD and set cursor to start
  lcd.clear();
  lcd.setCursor(0, 0);
  //display direction
  if (raw > 175 && raw < 210) {
    lcd.print("Direction: N ");
  } else if (raw > 225 && raw < 240) {
    lcd.print("Direction: NE");
  } else if (raw >= 0 && raw < 10) {
    lcd.print("Direction: E");
  } else if (raw > 35 && raw < 50) {
    lcd.print("Direction: SE");
  } else if (raw > 55 && raw < 75) {
    lcd.print("Direction: S");
  } else if (raw > 90 && raw < 105) {
    lcd.print("Direction: SW");
  } else if (raw > 120 && raw < 140) {
    lcd.print("Direction: W");
  } else if (raw > 155 && raw < 175) {
    lcd.print("Direction: NW");
  }

  //display degreed
  lcd.setCursor(0, 1);
  degree = (int(raw) / 255.0) * 360;
  lcd.print("Degreed: ");
  if (degree + 90 == 361) {
    lcd.print(0);
    lcd.print("\xDF");
    lcd.print("/360");
  } else if (degree + 90 > 360)
    lcd.print(degree - 270);
  else {
    lcd.print(degree + 90);
  }
  lcd.print("\xDF");

  //display pulsecount for both wheels
  lcd.setCursor(0, 2);
  lcd.print("Pulse Count: ");
  lcd.print(pulseCount);

  //display distance
  distance = (pulseCount * wheelDistanceRevolution) / PPR;
  lcd.setCursor(0, 3);
  lcd.print("Distance: ");
  lcd.print(distance);
}

void controlWithJoystick() {
  int joyX = analogRead(JOY_X_PIN);
  int joyY = analogRead(JOY_Y_PIN);
  if (joyY > 600) {
    pwm_R = map(joyY, 600, 1023, 0, 255);
    pwm_L = map(joyY, 600, 1023, 0, 255);
    digitalWrite(Motor_R_dir_pin, Motor_forward);
    digitalWrite(Motor_L_dir_pin, Motor_forward);
  } else if (joyY < 400) {
    pwm_R = map(joyY, 400, 0, 0, 255);
    pwm_L = map(joyY, 400, 0, 0, 255);
    digitalWrite(Motor_R_dir_pin, Motor_return);
    digitalWrite(Motor_L_dir_pin, Motor_return);
  }

  if (joyX > 600) {
    pwm_L = map(joyX, 600, 1023, 0, 255);
    digitalWrite(Motor_L_dir_pin, Motor_forward);
  } else if (joyX < 400) {
    pwm_R = map(joyX, 400, 0, 0, 255);
    digitalWrite(Motor_R_dir_pin, Motor_forward);
  }
}

void controlWithWebBrowser() {
  if (Serial.available() > 0) {
    String message = Serial.readStringUntil('\n');
    Serial.print("Message received, content: ");
    Serial.println(message);
    int dist = message.indexOf("Move");
    int dire = message.indexOf("Turn");

    if (dist > -1) {
      Serial.println("Command = Move ");
      dist = message.indexOf(":");

      if (dist > -1) {
        String stat = message.substring(dist + 1);
        intDist = stat.toInt();
        pulsesForStatcm = (intDist / wheelDistanceRevolution) * PPR;
        Serial.println(intDist);
      }
    } else if (dire > -1) {
      Serial.println("Command = Turn ");
      dire = message.indexOf(":");

      if (dire > -1) {
        String stat = message.substring(dire + 1);
        new_degree = stat.toInt();
        target_raw = (new_degree * 255 / 360 + 255) % 255;  // Define final direction
      }
    } else {
      Serial.println("No number found, try typing Move/Turn:Number");
    }
  }

  //Control forward and return
  if (intDist > 0) {
    if (iL < pulsesForStatcm) {
      Serial.println(iL);
      Serial.println(iR);
      Serial.println(pulseCount);
      flag = true;
      pwm_R = 255;
      pwm_L = 255;
      digitalWrite(Motor_R_dir_pin, Motor_forward);
      digitalWrite(Motor_L_dir_pin, Motor_forward);
    } else {
      intDist = 0;
      iL = 0;
      flag = false;
    }
  } else if (intDist < 0) {
    if (iL < (pulsesForStatcm / -1)) {
      flag = true;
      pwm_R = 255;
      pwm_L = 255;
      digitalWrite(Motor_R_dir_pin, Motor_return);
      digitalWrite(Motor_L_dir_pin, Motor_return);
    } else {
      intDist = 0;
      iL = 0;
      flag = false;
    }
  }

  //control final direction
  if (new_degree > old_degree) {
    pwm_L = 155;
    digitalWrite(Motor_L_dir_pin, Motor_forward);
  } else if (new_degree < old_degree) {
    pwm_R = 155;
    digitalWrite(Motor_R_dir_pin, Motor_forward);
  }

  //when the final direction(raw) have been reached, stop the car
  if (!flag) {
    if ((raw + 255 - target_raw) % 255 <= 10 || (target_raw + 255 - raw) % 255 <= 10) {
      // stop turning
      pwm_R = 0;
      pwm_L = 0;
      old_degree = new_degree;
    }
  }
}

void pulseCountForLeftWheel() {
  iL++;
  pulseCount++;
}

void pulseCountForRightWheel() {
  iR++;
  pulseCount++;
}

//change the button status to control the car with joystick
void buttonhPressedISR() {
  buttonPressed = !buttonPressed;
}