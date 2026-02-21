#include <Arduino.h>

#define RXD2 16
#define TXD2 17
// put function declarations here:
int myFunction(int, int);
String receivedMessage;
const int Ledboard = 2;


void setup() {
  // put your setup code here, to run once:
  int result = myFunction(2, 3);
  pinMode(Ledboard, OUTPUT);
  Serial.begin(115200);
  Serial.println("Serial Uart available");
  Serial2.begin(115200,SERIAL_8N1,RXD2,TXD2);
  Serial2.println("Serial 2 Uart from esp32");

}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite (Ledboard, HIGH);
  Serial2.println("led high 2");
  sleep (3);
  digitalWrite (Ledboard, LOW);
  Serial2.println("led low 2");
  sleep (3);
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}