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

// RFID
MFRC522 mfrc522(SS_PIN, RST_PIN);
MFRC522::MIFARE_Key key;

// Huidige toestand van de robot
byte huidigeStatus = wacht_op_start;

// Routekeuze via knop
byte knopTeller = 0;                 // Aantal keer route knop ingedrukt
unsigned long knopStartTijd = 0;     // Starttijd routekeuze
const unsigned long knopTimeout = 5000;

// Gekozen route
byte gekozenGang = 0;                // Gang 2 tot 7
byte doelTag = 0;                    // Gang + 6

// Debounce obstakels
unsigned long lastTriggerTime = 0;
const unsigned long debounceDelay = 100;

// Vorige knopstatus voor edge detectie
bool laatsteStartStatus = HIGH;

void setMotor(bool wiel, bool richting, uint8_t snelheid) { 
  uint8_t inPin1, inPin2, enPin; 
  if (snelheid == 0) { 
    if (wiel == links) { 
      digitalWrite(in1, LOW); 
      digitalWrite(in2, LOW); 
      analogWrite(enA, 0); 
      } else if (wiel == rechts) { 
        digitalWrite(in3, LOW); 
        digitalWrite(in4, LOW); 
        analogWrite(enB, 0); 
      } 
    } else { 
      if (wiel == links) { 
        inPin1 = in1; 
        inPin2 = in2; 
        enPin = enA; 
      } else if (wiel == rechts) { 
        inPin1 = in3; 
        inPin2 = in4; 
        enPin = enB; 
      } 
      
      if (richting == vooruit) { 
        digitalWrite(inPin1, HIGH); 
        digitalWrite(inPin2, LOW); 
      } else if (richting == achteruit) { 
        digitalWrite(inPin1, LOW); 
        digitalWrite(inPin2, HIGH); 
      } 
      
      analogWrite(enPin, snelheid); } }

// Beide motoren vooruit
void forward(uint8_t s) {
  setMotor(links, vooruit, s);
  setMotor(rechts, vooruit, s);
}

// Beide motoren achteruit
void backwards(uint8_t s) {
  setMotor(links, achteruit, s);
  setMotor(rechts, achteruit, s);
}

// Links draaien op de plaats
void left(uint8_t s) {
  setMotor(links, achteruit, s);
  setMotor(rechts, vooruit, s);
}

// Alles stoppen
void stopMotors() {
  setMotor(links, vooruit, 0);
  setMotor(rechts, vooruit, 0);
}

// Controle noodknop
bool checkNoodKnop() {

  // Noodknop actief laag
  if (digitalRead(nood_knop) == LOW) {

    // Alles stoppen
    stopMotors();

    // Terug naar beginstatus
    huidigeStatus = wacht_op_start;

    return true;
  }
  return false;
}

/* 
Delay waarbij nog constant de ir sensoren worden gechekt 
zodat de APM niet achteruit ergens vanaf rijdt
*/
bool safetyDelay(unsigned long ms) {

  unsigned long startTijd = millis();

  while (millis() - startTijd < ms) {

    // Altijd noodknop controleren
    if (digitalRead(nood_knop) == LOW) return false;

    // Achter obstakel betekent stoppen
    if (digitalRead(ir_achter) == HIGH) return false;

    delay(10);
  }
  return true;
}

void obstakelVermijding() {

  // Sensoren uitlezen
  byte rechtsSensor = digitalRead(ir_rechts);
  byte linksSensor = digitalRead(ir_links);
  byte achterSensor = digitalRead(ir_achter);

  unsigned long nu = millis();

  // Obstakel rechts
  if (rechtsSensor && nu - lastTriggerTime > debounceDelay) {

    lastTriggerTime = nu;

    // Alleen achteruit als achter vrij is
    if (!achterSensor) backwards(200);

    safetyDelay(300);

    // Daarna links wegdraaien
    left(200);
    safetyDelay(300);
  }

  // Obstakel links
  else if (linksSensor && nu - lastTriggerTime > debounceDelay) {

    lastTriggerTime = nu;

    if (!achterSensor) backwards(200);

    safetyDelay(300);

    // Rechts draaien
    setMotor(links, vooruit, 200);
    setMotor(rechts, achteruit, 200);
    safetyDelay(300);
  }

  // Geen obstakel
  else {
    forward(200);
  }
}

byte leesRFIDGetal() {

  // Geen kaart aanwezig
  if (!mfrc522.PICC_IsNewCardPresent()) return 0;

  // Kaart niet leesbaar
  if (!mfrc522.PICC_ReadCardSerial()) return 0;

  byte blok = 4;
  byte buffer[18];
  byte size = sizeof(buffer);

  // Authenticatie
  if (mfrc522.PCD_Authenticate(
      MFRC522::PICC_CMD_MF_AUTH_KEY_A,
      blok,
      &key,
      &(mfrc522.uid)) != MFRC522::STATUS_OK) return 0;

  // Blok lezen
  if (mfrc522.MIFARE_Read(blok, buffer, &size) != MFRC522::STATUS_OK) return 0;

  // Kaart afsluiten
  mfrc522.PICC_HaltA();
  mfrc522.PCD_StopCrypto1();

  // Zoek geldig getal
  for (byte i = 0; i < 16; i++) {
    if (buffer[i] >= 1 && buffer[i] <= 14) {
      return buffer[i];
    }
  }

  return 0;
}

void setup() {

  // Motor pinnen
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Sensoren
  pinMode(ir_rechts, INPUT);
  pinMode(ir_links, INPUT);
  pinMode(ir_achter, INPUT);

  // Knoppen
  pinMode(start_knop, INPUT_PULLUP);
  pinMode(nood_knop, INPUT_PULLUP);
  pinMode(micro_switch, INPUT_PULLUP);

  // RFID starten
  SPI.begin();
  mfrc522.PCD_Init();

  // Standaard RFID sleutel
  for (byte i = 0; i < 6; i++) {
    key.keyByte[i] = 0xFF;
  }
}

void loop() {

  // Altijd eerst noodknop
  if (checkNoodKnop()) return;

  // Huidige startknop status
  bool startNu = digitalRead(start_knop) == LOW;

  switch (huidigeStatus) {

    case wacht_op_start:
      stopMotors();

      // Startknop net ingedrukt
      if (startNu && laatsteStartStatus == HIGH) {
        huidigeStatus = start_vooruit;
      }
      break;

    case start_vooruit:
      forward(200);

      // Direct door naar zoeken van tag 1
      huidigeStatus = wacht_op_tag1;
      break;

    case wacht_op_tag1:
      forward(200);

      // Wachten tot RFID 1 wordt gelezen
      if (leesRFIDGetal() == 1) {
        stopMotors();
        knopTeller = 0;
        knopStartTijd = millis();
        huidigeStatus = route_keuze;
      }
      break;

    case route_keuze:
      stopMotors();

      // Elke druk telt op voor route
      if (startNu && laatsteStartStatus == HIGH) {
        knopTeller++;
        if (knopTeller > 6) knopTeller = 6;
      }

      // Timeout bepaalt keuze
      if (millis() - knopStartTijd > knopTimeout) {
        gekozenGang = knopTeller + 1;
        doelTag = gekozenGang + 6;
        huidigeStatus = zoek_gang;
      }
      break;

    case zoek_gang:
      obstakelVermijding();

      // Gang gevonden
      if (leesRFIDGetal() == gekozenGang) {
        stopMotors();
        delay(500);
        huidigeStatus = draai_links;
      }
      break;

    case draai_links:
      left(200);

      // Vaste draaitijd
      if (safetyDelay(600)) {
        stopMotors();
        huidigeStatus = zoek_route_plus_6;
      }
      break;

    case zoek_route_plus_6:
      forward(200);

      // Tweede tag gevonden
      if (leesRFIDGetal() == doelTag) {
        stopMotors();
        huidigeStatus = stop_bij_route;
      }
      break;

    case stop_bij_route:
      stopMotors();

      // Microswitch activeert vrij rijden
      if (digitalRead(micro_switch) == LOW) {
        huidigeStatus = vrij_rijden;
      }
      break;

    case vrij_rijden:
      obstakelVermijding();
      break;
  }

  // Edge detectie opslaan
  laatsteStartStatus = startNu;

  delay(10);
}