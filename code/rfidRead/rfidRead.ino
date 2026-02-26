#include <SPI.h>
#include <MFRC522.h>

#define RST_PIN 9
#define SS_PIN 10

MFRC522 mfrc522(SS_PIN, RST_PIN);

void setup() {
    Serial.begin(9600);
    while (!Serial);
    SPI.begin();
    mfrc522.PCD_Init();
    Serial.println("Scan een RFID kaart");
}

void loop() {
    if (!mfrc522.PICC_IsNewCardPresent()) return;
    if (!mfrc522.PICC_ReadCardSerial()) return;

    Serial.print("UID:");
    for (byte i = 0; i < mfrc522.uid.size; i++) {
        Serial.print(" ");
        if (mfrc522.uid.uidByte[i] < 0x10) Serial.print("0");
        Serial.print(mfrc522.uid.uidByte[i], HEX);
    }
    Serial.println();

    MFRC522::PICC_Type piccType = mfrc522.PICC_GetType(mfrc522.uid.sak);
    Serial.print("PICC type: ");
    Serial.println(mfrc522.PICC_GetTypeName(piccType));

    byte buffer[18];
    byte size = sizeof(buffer);

    for (byte page = 0; page < 16; page++) {
        MFRC522::StatusCode status = mfrc522.MIFARE_Read(page, buffer, &size);
        if (status != MFRC522::STATUS_OK) {
            Serial.print("Leesfout bij pagina ");
            Serial.println(page);
            break;
        }

        Serial.print("Pagina ");
        Serial.print(page);
        Serial.print(":");

        for (byte i = 0; i < 4; i++) {
            Serial.print(" ");
            if (buffer[i] < 0x10) Serial.print("0");
            Serial.print(buffer[i], HEX);
        }
        Serial.println();
    }

    mfrc522.PICC_HaltA();
    mfrc522.PCD_StopCrypto1();
    Serial.println();
}