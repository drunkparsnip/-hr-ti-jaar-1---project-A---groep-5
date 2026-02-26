#include <SPI.h>
#include <MFRC522.h>

// RFID
#define SS_PIN 11
#define RST_PIN 12

// Motor A links
#define enA 9
#define in1 8
#define in2 7

// Motor B rechts
#define enB 3
#define in3 5
#define in4 4

// Richtingen
#define links 0
#define rechts 1
#define vooruit 0
#define achteruit 1

// IR sensoren
#define ir_rechts 6
#define ir_links 2
#define ir_achter 15

// Ultrasoon
#define trigPinL A0
#define echoPinL A1
#define trigPinR A2
#define echoPinR A3

#define ULTRA_DREMPEL 25

// Bediening
#define start_knop 13
#define nood_knop 14
#define micro_switch 10

// Status machine
#define wacht_op_start 0
#define start_vooruit 1
#define wacht_op_tag1 2
#define route_keuze 3
#define zoek_gang 4
#define draai_links 5
#define zoek_route_plus_6 6
#define stop_bij_route 7
#define vrij_rijden 8

MFRC522 mfrc522(SS_PIN, RST_PIN);
MFRC522::MIFARE_Key key;

byte huidigeStatus = wacht_op_start;

byte knopTeller = 0;
unsigned long knopStartTijd = 0;
const unsigned long knopTimeout = 5000;

byte gekozenGang = 0;
byte doelTag = 0;

unsigned long lastTriggerTime = 0;
const unsigned long debounceDelay = 100;

bool laatsteStartStatus = HIGH;

float distanceL = 0;
float distanceR = 0;

void setMotor(bool wiel, bool richting, uint8_t snelheid) {
  uint8_t inPin1, inPin2, enPin;

  if (wiel == links) {
    inPin1 = in1;
    inPin2 = in2;
    enPin = enA;
  } else {
    inPin1 = in3;
    inPin2 = in4;
    enPin = enB;
  }

  if (snelheid == 0) {
    digitalWrite(inPin1, LOW);
    digitalWrite(inPin2, LOW);
    analogWrite(enPin, 0);
    return;
  }

  if (richting == vooruit) {
    digitalWrite(inPin1, HIGH);
    digitalWrite(inPin2, LOW);
  } else {
    digitalWrite(inPin1, LOW);
    digitalWrite(inPin2, HIGH);
  }

  analogWrite(enPin, snelheid);
}

void forward(uint8_t s) {
  setMotor(links, vooruit, s);
  setMotor(rechts, vooruit, s);
}

void backwards(uint8_t s) {
  setMotor(links, achteruit, s);
  setMotor(rechts, achteruit, s);
}

void left(uint8_t s) {
  setMotor(links, achteruit, s);
  setMotor(rechts, vooruit, s);
}

void stopMotors() {
  setMotor(links, vooruit, 0);
  setMotor(rechts, vooruit, 0);
}

bool checkNoodKnop() {
  if (digitalRead(nood_knop) == LOW) {
    stopMotors();
    huidigeStatus = wacht_op_start;
    return true;
  }
  return false;
}

bool safetyDelay(unsigned long ms) {
  unsigned long startTijd = millis();

  while (millis() - startTijd < ms) {
    if (digitalRead(nood_knop) == LOW) return false;
    if (digitalRead(ir_achter) == HIGH) return false;
    delay(10);
  }
  return true;
}

float meetAfstand(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duur = pulseIn(echoPin, HIGH, 30000);
  if (duur == 0) return -1;

  return (duur * 0.0343) / 2.0;
}

void updateUltrasoon() {
  distanceL = meetAfstand(trigPinL, echoPinL);
  distanceR = meetAfstand(trigPinR, echoPinR);
}

bool ultrasoonObstakel() {
  updateUltrasoon();

  if (distanceL > 0 && distanceL < ULTRA_DREMPEL) return true;
  if (distanceR > 0 && distanceR < ULTRA_DREMPEL) return true;

  return false;
}

void obstakelVermijding() {

  if (ultrasoonObstakel()) {
    stopMotors();
    backwards(180);
    safetyDelay(300);
    left(180);
    safetyDelay(300);
    return;
  }

  byte rechtsSensor = digitalRead(ir_rechts);
  byte linksSensor = digitalRead(ir_links);
  byte achterSensor = digitalRead(ir_achter);

  unsigned long nu = millis();

  if (rechtsSensor && nu - lastTriggerTime > debounceDelay) {
    lastTriggerTime = nu;
    if (!achterSensor) backwards(200);
    safetyDelay(300);
    left(200);
    safetyDelay(300);
  }

  else if (linksSensor && nu - lastTriggerTime > debounceDelay) {
    lastTriggerTime = nu;
    if (!achterSensor) backwards(200);
    safetyDelay(300);
    setMotor(links, vooruit, 200);
    setMotor(rechts, achteruit, 200);
    safetyDelay(300);
  }

  else {
    forward(200);
  }
}

byte leesRFIDNummerOfSpeciaal() {
  if (!mfrc522.PICC_IsNewCardPresent()) return 0;
  if (!mfrc522.PICC_ReadCardSerial()) return 0;

  byte buffer[18];
  byte size = sizeof(buffer);

  if (mfrc522.MIFARE_Read(4, buffer, &size) != MFRC522::STATUS_OK) {
    mfrc522.PICC_HaltA();
    return 0;
  }

  mfrc522.PICC_HaltA();

  // normaal genummerde tags
  if (buffer[0] >= '0' && buffer[0] <= '9') {
    byte nummer = buffer[0] - '0';

    if (buffer[1] >= '0' && buffer[1] <= '9') {
      nummer = nummer * 10 + (buffer[1] - '0');
    }

    if (nummer >= 1 && nummer <= 14) {
      return nummer;
    }
  }

  // speciaal geval, zoals tag 9
  return 9;
}

void setup() {

  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  pinMode(ir_rechts, INPUT);
  pinMode(ir_links, INPUT);
  pinMode(ir_achter, INPUT);

  pinMode(start_knop, INPUT_PULLUP);
  pinMode(nood_knop, INPUT_PULLUP);
  pinMode(micro_switch, INPUT_PULLUP);

  pinMode(trigPinL, OUTPUT);
  pinMode(echoPinL, INPUT);
  pinMode(trigPinR, OUTPUT);
  pinMode(echoPinR, INPUT);

  SPI.begin();
  mfrc522.PCD_Init();

  for (byte i = 0; i < 6; i++) key.keyByte[i] = 0xFF;
}

void loop() {

  if (checkNoodKnop()) return;

  bool startNu = digitalRead(start_knop) == LOW;

  switch (huidigeStatus) {

    case wacht_op_start:
      stopMotors();
      if (startNu && laatsteStartStatus == HIGH) huidigeStatus = start_vooruit;
      break;

    case start_vooruit:
      forward(200);
      huidigeStatus = wacht_op_tag1;
      break;

    case wacht_op_tag1:
      forward(200);
      if (leesRFIDGetal() == 1) {
        stopMotors();
        knopTeller = 0;
        knopStartTijd = millis();
        huidigeStatus = route_keuze;
      }
      break;

    case route_keuze:
      stopMotors();
      if (startNu && laatsteStartStatus == HIGH) {
        knopTeller++;
        if (knopTeller > 6) knopTeller = 6;
      }
      if (millis() - knopStartTijd > knopTimeout) {
        gekozenGang = knopTeller + 1;
        doelTag = gekozenGang + 6;
        huidigeStatus = zoek_gang;
      }
      break;

    case zoek_gang:
      obstakelVermijding();
      if (leesRFIDGetal() == gekozenGang) {
        stopMotors();
        delay(500);
        huidigeStatus = draai_links;
      }
      break;

    case draai_links:
      left(200);
      if (safetyDelay(600)) {
        stopMotors();
        huidigeStatus = zoek_route_plus_6;
      }
      break;

    case zoek_route_plus_6:
      forward(200);
      if (leesRFIDGetal() == doelTag) {
        stopMotors();
        huidigeStatus = stop_bij_route;
      }
      break;

    case stop_bij_route:
      stopMotors();
      if (digitalRead(micro_switch) == LOW) huidigeStatus = vrij_rijden;
      break;

    case vrij_rijden:
      obstakelVermijding();
      break;
  }

  laatsteStartStatus = startNu;
  delay(10);
}