#include <Arduino.h>
#include <ESP32Encoder.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define RXD2 16
#define TXD2 17
// put function declarations here:
int myFunction(int, int);
String receivedMessage;
const int Ledboard = 2;
// Motor A
const int  mtLeftPwm = 27; 
const int  mtLeftDir = 26; 
 
// Setting PWM properties
const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;
int dutyCycle = 50;
char buffer[64];
const int adcPin = 25;
static int32_t lastPosition = -1;
int angle1 = 0;
int angle2 = 0;

ESP32Encoder encoder1;
ESP32Encoder encoder2;

// Création de l'objet PCA9685 (adresse par défaut 0x40)
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Configuration pour des Servomoteurs standards
#define SERVO_FREQ 50 // 50Hz est le standard pour les servos analogiques

// Largeur d'impulsion (dépend de votre servo, à ajuster)
#define USMIN  600 // Longueur d'impulsion min (0 degré)
#define USMAX  2400 // Longueur d'impulsion max (180 degrés)

void setup() {
  // put your setup code here, to run once:
  int result = myFunction(2, 3);
  pinMode(Ledboard, OUTPUT);
  Serial.begin(115200);
  Serial.println("Serial Uart available");
  Serial2.begin(115200,SERIAL_8N1,RXD2,TXD2);
  Serial2.println("Serial 2 Uart from esp32");
  pinMode(mtLeftDir, OUTPUT);
   // configure LEDC PWM
  ledcSetup(pwmChannel,freq,resolution);
  ledcAttachPin(mtLeftPwm,pwmChannel);
  // Initialiser la direction et le PWM
  digitalWrite(mtLeftDir, LOW); // Direction initiale
  ledcWrite(pwmChannel, 30); // Rapport cyclique à 50% (vitesse moyenne)
  // Attache les pins (A sur 26, B sur 27)
  encoder1.attachHalfQuad(32, 33);
  // Initialise le compteur à 0
  encoder1.setCount(0);
  encoder2.attachHalfQuad(35, 34);
  encoder2.setCount(0);
  
  pwm.begin();
  // Fréquence PWM
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  

}

void setServoAngle(uint8_t n, double angle) {
  double pulse = map(angle, 0, 180, USMIN, USMAX);
  double pulselength = 1000000; // 1,000,000 us par seconde
  pulselength /= SERVO_FREQ;   // Longueur d'une période en us
  
  // Le PCA9685 a une résolution de 12 bits (4096 pas)
  pulse = (pulse * 4096) / pulselength;
  
  pwm.setPWM(n, 0, pulse);
}
void loop() {
  // put your main code here, to run repeatedly:
  int adcValue = analogRead(adcPin);
  float voltage = (adcValue / 4095.0) * 3.3;
  int32_t pos1 = encoder1.getCount();
  int32_t pos2 = encoder2.getCount();
  digitalWrite (Ledboard, HIGH);
    // Exemple : changer la direction toutes les 2 secondes
  digitalWrite(mtLeftDir, LOW);
  ledcWrite(pwmChannel, dutyCycle); // Vitesse élevée
  setServoAngle (0,angle1);
  setServoAngle (1,angle1);

  sprintf(buffer, "led high 2, mtLeftDir = Fwd, dutyCycle = %d, Cur = %f, pos1Cnt = %ld, pos2Cnt = %ld , servo1ang = %d"  , dutyCycle,voltage,pos1,pos2,angle1 );
  Serial2.println(buffer);

  sleep (3);
  if (dutyCycle >= 150) {
    dutyCycle = 30;
  }
  else {
    dutyCycle += 10;
  }
  if (angle1 >= 180){
    angle1 = 0;
  }
  else {
    angle1 +=20;
  }

}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}