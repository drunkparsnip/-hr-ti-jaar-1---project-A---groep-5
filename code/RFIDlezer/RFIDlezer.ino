#include <SPI.h>
#include <MFRC522.h>

#define SS_PIN 10
#define RST_PIN 9

MFRC522 mfrc522(SS_PIN, RST_PIN);
MFRC522::MIFARE_Key key;

void setup() {
  Serial.begin(9600);
  SPI.begin();
  mfrc522.PCD_Init();
  
  // Standaard key
  for (byte i = 0; i < 6; i++) {
    key.keyByte[i] = 0xFF;
  }
  
  Serial.println("Houd je RFID tag bij de lezer...");
}

void loop() {
  if (!mfrc522.PICC_IsNewCardPresent()) {
    return;
  }
  
  if (!mfrc522.PICC_ReadCardSerial()) {
    return;
  }
  
  byte blok = 4;
  byte buffer[18];
  byte size = sizeof(buffer);
  
  // Authenticatie
  MFRC522::StatusCode status = mfrc522.PCD_Authenticate(
    MFRC522::PICC_CMD_MF_AUTH_KEY_A, 
    blok, 
    &key, 
    &(mfrc522.uid)
  );
  
  if (status != MFRC522::STATUS_OK) {
    Serial.print("Authenticatie mislukt: ");
    Serial.println(mfrc522.GetStatusCodeName(status));
    return;
  }
  
  // Lees het blok
  status = mfrc522.MIFARE_Read(blok, buffer, &size);
  
  if (status != MFRC522::STATUS_OK) {
    Serial.print("Lezen mislukt: ");
    Serial.println(mfrc522.GetStatusCodeName(status));
  } else {
    // Zoek door het hele blok naar een getal tussen 1-14
    bool gevonden = false;
    byte getalInBlok = 0;
    
    for (byte i = 0; i < 16; i++) {
      if (buffer[i] >= 1 && buffer[i] <= 14) {
        getalInBlok = buffer[i];
        gevonden = true;
        Serial.print("Waarde gevonden op positie ");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(getalInBlok);
        break; // Stop na het eerste getal
      }
    }
    
    if (gevonden) {
      // Switch case om actie te ondernemen op basis van het getal
      switch(getalInBlok) {
        case 1:
          Serial.println("Getal 1");
          break;
          
        case 2:
          Serial.println("Getal 2");
          break;
          
        case 3:
          Serial.println("Getal 3");
          break;
          
        case 4:
          Serial.println("Getal 4");
          break;
          
        case 5:
          Serial.println("Getal 5");
          break;
          
        case 6:
          Serial.println("Getal 6");
          break;
          
        case 7:
          Serial.println("Getal 7");
          break;
          
        case 8:
          Serial.println("Getal 8");
          break;
          
        case 9:
          Serial.println("Getal 9");
          break;
          
        case 10:
          Serial.println("Getal 10");
          break;
          
        case 11:
          Serial.println("Getal 11");
          break;
          
        case 12:
          Serial.println("Getal 12");
          break;
          
        case 13:
          Serial.println("Getal 13");
          break;
          
        case 14:
          Serial.println("Getal 14");
          break;
          
        default:
          Serial.println("Onbekend getal");
          break;
      }
    } else {
      Serial.println("Geen getal tussen 1-14 gevonden in dit blok.");
    }
    
    // Toon de volledige inhoud
    Serial.print("Blok inhoud: ");
    for (byte i = 0; i < 16; i++) {
      Serial.print(buffer[i]);
      Serial.print(" ");
    }
    Serial.println();
  }
  
  mfrc522.PICC_HaltA();
  mfrc522.PCD_StopCrypto1();
  
  delay(2000);
}