#include <SPI.h>
#include <MFRC522.h>

// RFID reader connections
#define SS_PIN 11
#define RST_PIN 12

// Motor A connections
#define enA 9
#define in1 8
#define in2 7
// Motor B connections
#define enB 3
#define in3 5
#define in4 4
// movement defines
#define links 0
#define rechts 1
#define vooruit 0
#define achteruit 1

#define ir_rechts 6
#define ir_links 2
#define ir_achter 15

#define start_knop 13
#define nood_knop 14

//route definities
#define wacht_op_start 0
#define kies_route 1
#define zoek_checkpoint1 2
#define zoek_checkpoint2 3
#define route_voltooid 4
#define vrij_rijden 5

MFRC522 mfrc522(SS_PIN, RST_PIN);
MFRC522::MIFARE_Key key;

uint8_t sensorValueRechts;
uint8_t sensorValueLinks;
uint8_t sensorValueAchter;

unsigned long lastTriggerTime = 0;
const unsigned long debounceDelay = 100;

// Status variabelen voor knoppen
bool APMRijdt = false;
bool laatsteStartStatus = HIGH;
bool laatsteNoodStatus = HIGH;

// Route variabelen
bool routeActief = false;
bool routeKiezen = false;
byte gekozenRoute = 0;
byte doelGetal1 = 0;
byte doelGetal2 = 0;
bool checkpoint1Bereikt = false;
unsigned long routeKiesTijd = 0;
const unsigned long routeKiesTimer = 5000;

byte huidigeStatus = wacht_op_start;

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
    analogWrite(enPin, snelheid);
  }
}

void setup() {
  Serial.begin(9600);

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

  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

  SPI.begin();
  mfrc522.PCD_Init();

  for (byte i = 0; i < 6; i++) {
    key.keyByte[i] = 0xFF;
  }
}

void forward(uint8_t snelheid) {
  setMotor(rechts, vooruit, snelheid);
  setMotor(links, vooruit, snelheid);
}

void backwards(uint8_t snelheid) {
  setMotor(rechts, achteruit, snelheid);
  setMotor(links, achteruit, snelheid);
}

void left(uint8_t snelheid) {
  setMotor(rechts, vooruit, snelheid);
  setMotor(links, achteruit, snelheid);
}

void right(uint8_t snelheid) {
  setMotor(rechts, achteruit, snelheid);
  setMotor(links, vooruit, snelheid);
}

void stopMotors() {
  setMotor(rechts, vooruit, 0);
  setMotor(links, vooruit, 0);
}

bool checkNoodKnop() {
  if(digitalRead(nood_knop) == LOW) {
    APMRijdt = false;
    routeActief = false;
    stopMotors();
    huidigeStatus = wacht_op_start;
    return true;
  }
  return false;
}

bool safetyDelay(unsigned long ms) {
  unsigned long startTime = millis();
  while (millis() - startTime < ms) {
    // Check noodknop tijdens delay
    if (digitalRead(nood_knop) == LOW) {
      APMRijdt = false;
      stopMotors();
      return false;
    }
    
    sensorValueAchter = digitalRead(ir_achter);
    if (sensorValueAchter == 1) {
      stopMotors();
      return false;
    }
    delay(10);
  }
  return true;
}

void obstakelVermijding() {
  sensorValueRechts = digitalRead(ir_rechts);
  sensorValueLinks = digitalRead(ir_links);
  sensorValueAchter = digitalRead(ir_achter);

  unsigned long currentTime = millis();

  if (sensorValueRechts == 1 && (currentTime - lastTriggerTime >= debounceDelay)) {
    lastTriggerTime = currentTime;

    if (sensorValueAchter == 1) {
      left(255);
      if (!safetyDelay(300)) {
        return;
      }
    } else {
      backwards(255);

      if (!safetyDelay(300)) {
        return;
      }

      left(255);
      if (!safetyDelay(300)) {
        return;
      }
    }
  } else if (sensorValueLinks == 1 && (currentTime - lastTriggerTime >= debounceDelay)) {
    lastTriggerTime = currentTime;

    if (sensorValueAchter == 1) {
      right(255);
      if (!safetyDelay(300)) {
        return;
      }
    } else {
      backwards(255);

      if (!safetyDelay(300)) {
        return;
      }

      right(255);
      if (!safetyDelay(300)) {
        return;
      }
    }
  } else {
    forward(255);
  }
}

byte leesRFIDGetal() {
  if (!mfrc522.PICC_IsNewCardPresent()) {
    return 0;
  }
  
  if (!mfrc522.PICC_ReadCardSerial()) {
    return 0;
  }
  
  byte blok = 4;
  byte buffer[18];
  byte size = sizeof(buffer);
  
  MFRC522::StatusCode status = mfrc522.PCD_Authenticate(
    MFRC522::PICC_CMD_MF_AUTH_KEY_A, 
    blok, 
    &key, 
    &(mfrc522.uid)
  );
  
  if (status != MFRC522::STATUS_OK) {
    mfrc522.PICC_HaltA();
    mfrc522.PCD_StopCrypto1();
    return 0;
  }
  
  status = mfrc522.MIFARE_Read(blok, buffer, &size);
  
  byte gevondenGetal = 0;
  
  if (status == MFRC522::STATUS_OK) {
    for (byte i = 0; i < 16; i++) {
      if (buffer[i] >= 1 && buffer[i] <= 14) {
        gevondenGetal = buffer[i];
        Serial.print("RFID gelezen: ");
        Serial.println(gevondenGetal);
        break;
      }
    }
  }
  
  mfrc522.PICC_HaltA();
  mfrc522.PCD_StopCrypto1();
  
  return gevondenGetal;
}

void checkKnoppen() {
  bool startIngedrukt = digitalRead(start_knop) == LOW;
  bool noodIngedrukt = digitalRead(nood_knop) == LOW;
  
  if (startIngedrukt && laatsteStartStatus == HIGH) {
    if(huidigeStatus == wacht_op_start) {
      huidigeStatus = kies_route;
      routeKiesTijd = millis();
      routeKiezen = true;
    }
  }
  
  if (noodIngedrukt && laatsteNoodStatus == HIGH) {
    checkNoodKnop();
  }
  
  laatsteStartStatus = digitalRead(start_knop);
  laatsteNoodStatus = digitalRead(nood_knop);
}

void loop() {
  if (checkNoodKnop()) {
    return;
  }
  
  checkKnoppen();
  
  if (huidigeStatus == wacht_op_start) {
    stopMotors();
  }
  
  else if (huidigeStatus == kies_route) {
    if (millis() - routeKiesTijd > routeKiesTimer) {
      huidigeStatus = wacht_op_start;
      routeKiezen = false;
    } else {
      byte gelezenRoute = leesRFIDGetal();
      
      if (gelezenRoute >= 1 && gelezenRoute <= 7) {
        gekozenRoute = gelezenRoute;
        doelGetal1 = gekozenRoute;
        doelGetal2 = gekozenRoute + 7;
      
        routeKiezen = false;
        routeActief = true;
        APMRijdt = true;
        checkpoint1Bereikt = false;
        huidigeStatus = zoek_checkpoint1;
        delay(1000);
      }
    }
  }
  
  else if (huidigeStatus == zoek_checkpoint1) {
    // Rijd met obstakelvermijding
    obstakelVermijding();
    
    // Check RFID voor checkpoint
    byte getal = leesRFIDGetal();
    if (getal == doelGetal1) {
      stopMotors();
      checkpoint1Bereikt = true;
      huidigeStatus = zoek_checkpoint2;
      delay(1000);
    }
  }
  
  else if (huidigeStatus == zoek_checkpoint2) {
    obstakelVermijding();

    byte getal2 = leesRFIDGetal();
    if (getal2 == doelGetal2) {
      stopMotors();
      delay(2000);
      huidigeStatus = vrij_rijden;
    }
  }
  
  else if (huidigeStatus == vrij_rijden) {
    obstakelVermijding();
  }
  
  delay(10);
}
