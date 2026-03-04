#include <SPI.h>
#include <MFRC522.h>

#define SS_PIN 0
#define RST_PIN 0

#define RED_PIN 0
#define BLUE_PIN 0
#define GREEN_PIN 0

#define klaar_voor_start 0
#define depot_selectie 1
#define onderweg_naar_depot 2
#define wachten_op_pakket 3
#define wacht_met_wegrijden 4
#define onderweg_naar_eindpunt 5
#define op_eindpunt 6
#define vrij_rijden 7

enum Kleur {rood, blauw, groen, geel, cyaan, paars, wit, knipperen};

MFRC522 mfrc522(SS_PIN, RST_PIN);

byte leesRFIDGetal() {
  if (!mfrc522.PICC_IsNewCardPresent()) return 0;
  if (!mfrc522.PICC_ReadCardSerial()) return 0;

  byte pagina = 4;
  byte buffer[18];
  byte size = sizeof(buffer);

  if (mfrc522.MIFARE_Read(pagina, buffer, &size) != MFRC522::STATUS_OK) {
    mfrc522.PICC_HaltA();
    return 0;
  }

  mfrc522.PICC_HaltA();

  byte b0 = buffer[0];
  byte b1 = buffer[1];

  // Tweetallig: "10"-"14"
  if (b0 == 0x31 && b1 >= 0x30 && b1 <= 0x34) {
    return 10 + (b1 - 0x30);
  }

  // Eentallig: "1"-"9"
  if (b0 >= 0x31 && b0 <= 0x39 && b1 == 0x00) {
    return b0 - 0x30;
  }

  return 0;
}

void setColor(int redValue, int greenValue,  int blueValue) {
  analogWrite(RED_PIN, redValue);
  analogWrite(GREEN_PIN,  greenValue);
  analogWrite(BLUE_P, blueValue);
}

void rgbLED(Kleur kleur) {
  switch (kleurLED) {
    case rood:
      setColor(255, 0, 0);
      break;

    case green:
      setColor(0, 255, 0);
      break;

    case blauw:
      setColor(0, 0, 255);
      break;

    case geel:
      setColor(255, 255, 0);
      break;   

    case cyaan:
      setColor(0, 255, 255);
      break;

    case paars:
      setColor(255, 0, 255);
      break;

    case wit:
      setColor(255, 255, 255);
      break; 

    case knipperen: 
      setColor(255, 0, 0)
      delay(200)
      setColor(0, 0, 0);
      delay(200);
      setColor(255, 0, 0);
      delay(200);
      setColor(0, 0, 0);
      delay(200);
      setColor(255, 0, 0);
      delay(200);
      setColor(0, 0, 0);
      delay(200);
      setColor(255, 0, 0);
  }
}

void checkStatus(uint8_t status) {
  switch (status) {
    case depot_selectie: rgbLED(rood); 
    case klaar_voor_start: rgbLED(groen); 
    case onderweg_naar_depot: rgbLED(blauw); 
    case wachten_op_pakket: rgbLED(geel); 
    case onderweg_naar_eindpunt: rgbLED(cyaan); 
    case vrij_rijden: rgbLED(paars); 
    case op_eindpunt: rgbLED(wit); 
    case wacht_met_wegrijden: rgbLED(knipperen);
  }
}

void setup() {
  Serial2.begin(115200, SERIAL_8N1, 0, 0); // RX (main ESP), TX (ESP 2)
  SPI.begin();
  mfrc522.PCD_Init();

  pinMode(RED_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
}

void loop() {
  // RFID uitlezen
  byte tag = leesRFIDGetal();
  if (tag > 0) {
    Serial2.write(tag); // Stuur tagnummer als een byte
    delay(500);         // Voorkom dubbele uitlezing
  }

  // status main esp ontvangen
  if (Serail2.available() >= 2) {
    if (Serial.read() == 0xFF) {
      unint8_t status = Serial2.read();
      checkStatus(status);
    }
  }
}