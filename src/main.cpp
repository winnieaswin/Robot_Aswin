#include <Arduino.h>
#include <ESP32Encoder.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define RXD2 16
#define TXD2 17
// put function declarations here:
String receivedMessage;
const int Ledboard = 2;
// setting pin out
const int mtLeftPwm = 27; 
const int mtLeftDir = 25; 
const int mtLeftCur_ADC = 34; //adc input
const int mtLeftEncoder_1 = 32;
const int mtLeftEncoder_2 = 33;
const int secLeftEncoder_A = 18;
const int secLeftEncoder_B = 19;

const int mtRightPwm = 23; 
const int mtRightDir = 13; 
const int mtRightCur_ADC = 35; //adc input
const int mtRightEncoder_1 = 26;
const int mtRightEncoder_2 = 14;
const int secRightEncoder_A = 4;
const int secRightEncoder_B = 5;


// Setting PWM properties
const int freq = 30000;
const int pwmLeftChannel = 0;
const int pwmRightChannel = 1;
const int resolution = 8;
int dutyCycle = 50;
static char txbuf[96];


static int32_t lastPosition = -1;
static int32_t encoderCntRightMain = 0;
static int32_t encoderCntRightSec = 0;
static int32_t encoderCntLeftMain = 0;
static int32_t encoderCntLeftSec = 0;

int angle1 = 10;
int angle2 = 20;

ESP32Encoder encoderLeftMain;
ESP32Encoder encoderLeftSec;
ESP32Encoder encoderRightMain;
ESP32Encoder encoderRightSec;


// Création de l'objet PCA9685 (adresse par défaut 0x40)
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Configuration pour des Servomoteurs standards
#define SERVO_FREQ 50 // 50Hz est le standard pour les servos analogiques

// Largeur d'impulsion (dépend de votre servo, à ajuster)
#define USMIN  600 // Longueur d'impulsion min (0 degré)
#define USMAX  2400 // Longueur d'impulsion max (180 degrés)

struct MotorMessage {
  uint8_t  pwmRightPct; // 0..100
  uint8_t  pwmLeftPct;  // 0..100
  uint8_t  dirRight;    // 0/1
  uint8_t  dirLeft;     // 0/1
  uint16_t dutyRight;   // converted duty (0..255)
  uint16_t dutyLeft;    // converted duty (0..255)
  bool     valid;
};

void setServoAngle(uint8_t n, double angle);
static uint8_t adcToPercent(uint16_t adc);
void infoRobot ();
bool readSerial2Line(String &outLine);
MotorMessage parseMotorMessagePct(const String &line);
static uint16_t pctToDuty8(uint8_t pct);
static void applyMotor(const MotorMessage &m);
static void stopMotors();
void setup() {
  // put your setup code here, to run once:
 
  Serial.begin(115200);
  Serial.println("Serial Uart available");
  Serial2.begin(115200,SERIAL_8N1,RXD2,TXD2);
  
  pinMode(Ledboard, OUTPUT);
  pinMode(mtLeftDir, OUTPUT);
  pinMode(mtRightDir,OUTPUT);
  // configure motor PWM
  ledcSetup(pwmLeftChannel,freq,resolution);
  ledcAttachPin(mtLeftPwm,pwmLeftChannel);   
  ledcSetup(pwmRightChannel,freq,resolution);
  ledcAttachPin(mtRightPwm,pwmRightChannel);

  // init encorder
  pinMode(secLeftEncoder_A, INPUT_PULLUP);
  pinMode(secLeftEncoder_B, INPUT_PULLUP);
  pinMode(secRightEncoder_A, INPUT_PULLUP);
  pinMode(secRightEncoder_B, INPUT_PULLUP);
  encoderRightMain.attachHalfQuad(mtRightEncoder_1, mtRightEncoder_2);
  encoderRightMain.setCount(0);   // Initialise le compteur à 0
  encoderRightSec.attachHalfQuad(secRightEncoder_A, secRightEncoder_B);
  encoderRightSec.setCount(0);
  encoderLeftMain .attachHalfQuad(mtLeftEncoder_1, mtLeftEncoder_2);
  encoderLeftMain.setCount(0);
  encoderLeftSec.attachHalfQuad(secLeftEncoder_A, secLeftEncoder_B);
  encoderLeftSec.setCount(0);
  stopMotors();

  // init PCA9685
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  
  Serial.println("ESP32 motor controller ready.");

}




void loop() {
  // put your main code here, to run repeatedly:


  digitalWrite (Ledboard, HIGH);
    // Exemple : changer la direction toutes les 2 secondes
  setServoAngle (0,angle1);
  setServoAngle (1,angle1);
  String line;
  if (readSerial2Line(line)) {
    MotorMessage msg = parseMotorMessagePct(line);

    if (msg.valid) {
      applyMotor(msg);
      Serial.print("OK pctR=");
      Serial.print(msg.pwmRightPct);
      Serial.print("% pctL=");
      Serial.print(msg.pwmLeftPct);
      Serial.print("% dirR=");
      Serial.print(msg.dirRight);
      Serial.print(" dirL=");
      Serial.print(msg.dirLeft);
      Serial.print(" dutyR=");
      Serial.print(msg.dutyRight);
      Serial.print(" dutyL=");
      Serial.println(msg.dutyLeft);
    }
    else {
        Serial.print("Bad message: ");
        Serial.println(line);
    }
    
  }

static uint32_t lastInfo = 0;
  if (millis() - lastInfo >= 100) {  // 10 Hz
    lastInfo = millis();
    infoRobot();
  }
}


void setServoAngle(uint8_t n, double angle) {
  double pulse = map(angle, 0, 180, USMIN, USMAX);
  double pulselength = 1000000; // 1,000,000 us par seconde
  pulselength /= SERVO_FREQ;   // Longueur d'une période en us
  // Le PCA9685 a une résolution de 12 bits (4096 pas)
  pulse = (pulse * 4096) / pulselength;
  pwm.setPWM(n, 0, pulse);
}

void infoRobot() {
  uint8_t leftPct  = adcToPercent(analogRead(mtLeftCur_ADC));
  uint8_t rightPct = adcToPercent(analogRead(mtRightCur_ADC));

  long encR = (long)encoderRightMain.getCount();
  long encL = (long)encoderLeftMain.getCount();
  long encSecR = (long)encoderRightSec.getCount();
  long encSecL = (long)encoderLeftSec.getCount();

  // CSV for Jetson: leftPct,rightPct,encR,encL
  snprintf(txbuf, sizeof(txbuf), "%u,%u,%ld,%ld,%ld,%ld", leftPct, rightPct, encR, encL, encSecR, encSecL);

  Serial2.println(txbuf);   // DATA ONLY to Jetson
  Serial.println(txbuf);
  // Serial.println(txbuf); // optional debug to USB
}


static uint8_t adcToPercent(uint16_t adc) {
  if (adc > 4095) adc = 4095;
  return (uint8_t)((adc * 100UL) / 4095UL);
}

// Reads one '\n' terminated line from Serial2 (non-blocking-ish)
bool readSerial2Line(String &outLine) {
  static String buf;

  while (Serial2.available() > 0) {
    char c = (char)Serial2.read();
    if (c == '\r') continue;       // ignore CR
    if (c == '\n') {
      outLine = buf;
      buf = "";
      outLine.trim();
      return outLine.length() > 0;
    }

    // basic protection against runaway lines
    if (buf.length() < 80) buf += c;
    else buf = ""; // reset if too long
  }
  return false;
}

// Parse "pctR,pctL,dirR,dirL"
MotorMessage parseMotorMessagePct(const String &line) {
  MotorMessage msg{};
  msg.valid = false;

  int p1 = line.indexOf(',');
  int p2 = (p1 >= 0) ? line.indexOf(',', p1 + 1) : -1;
  int p3 = (p2 >= 0) ? line.indexOf(',', p2 + 1) : -1;
  if (p1 < 0 || p2 < 0 || p3 < 0) return msg;

  long pctR = line.substring(0, p1).toInt();
  long pctL = line.substring(p1 + 1, p2).toInt();
  long dirR = line.substring(p2 + 1, p3).toInt();
  long dirL = line.substring(p3 + 1).toInt();

  if (pctR < 0 || pctR > 100) return msg;
  if (pctL < 0 || pctL > 100) return msg;
  if (!(dirR == 0 || dirR == 1)) return msg;
  if (!(dirL == 0 || dirL == 1)) return msg;

  msg.pwmRightPct = (uint8_t)pctR;
  msg.pwmLeftPct  = (uint8_t)pctL;
  msg.dirRight    = (uint8_t)dirR;
  msg.dirLeft     = (uint8_t)dirL;

  msg.dutyRight = pctToDuty8(msg.pwmRightPct);
  msg.dutyLeft  = pctToDuty8(msg.pwmLeftPct);

  msg.valid = true;
  return msg;
}

// Convert 0..100% to 0..255 (rounded)
static uint16_t pctToDuty8(uint8_t pct) {
  return (uint16_t)((pct * 255 + 50) / 100);
}

static void applyMotor(const MotorMessage &m) {
  // Direction pins
  digitalWrite(mtRightDir, m.dirRight ? HIGH : LOW);
  digitalWrite(mtLeftDir,  m.dirLeft ? LOW : HIGH);

  // Duty
  uint16_t dutyR = pctToDuty8(m.pwmRightPct);
  uint16_t dutyL = pctToDuty8(m.pwmLeftPct);

  ledcWrite(pwmRightChannel, dutyR);
  ledcWrite(pwmLeftChannel,  dutyL);
}

static void stopMotors() {
  ledcWrite(pwmRightChannel, 0);
  ledcWrite(pwmLeftChannel,  0);
}