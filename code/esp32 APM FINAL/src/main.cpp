#include <Arduino.h>
#include <SPI.h>
#include <MFRC522.h>

// ─── RFID ─────────────────────────────────────────────────────────────────────
// ESP32 VSPI: SCK=18, MISO=19, MOSI=23 (handled by SPI.begin())
#define SS_PIN  5
#define RST_PIN 27

// ─── Motor A (links) ──────────────────────────────────────────────────────────
#define enA 25  // PWM-capable
#define in1 26
#define in2 14

// ─── Motor B (rechts) ─────────────────────────────────────────────────────────
#define enB 33  // PWM-capable
#define in3 32
#define in4 16

// ─── Richtingen ───────────────────────────────────────────────────────────────
#define links     0
#define rechts    1
#define vooruit   0
#define achteruit 1

// ─── IR Sensoren ──────────────────────────────────────────────────────────────
#define ir_rechts 34  // Input-only pin — OK voor sensor
#define ir_links  36  // Input-only pin — OK voor sensor
#define ir_achter 39  // Input-only pin — OK voor sensor

// ─── Bediening ────────────────────────────────────────────────────────────────
#define start_knop          13
#define nood_knop           12
#define selectie_knop       15
#define pakket_detectie     4

// ─── Ultrasoon sensoren ───────────────────────────────────────────────────────
// Pins gekozen zodat ze niet overlappen met bestaande pin-definities.
// Vrije output-capable GPIO's op ESP32: 2, 17, 21, 22
#define trigPinL 21
#define echoPinL 22
#define trigPinR 17
#define echoPinR 2

#define NUM_READINGS 8

// ─── LEDC PWM configuratie ────────────────────────────────────────────────────
#define PWM_FREQ       5000
#define PWM_RESOLUTION 8       // 0–255
#define LEDC_CH_A      0       // Kanaal motor A
#define LEDC_CH_B      1       // Kanaal motor B

// ─── main statusmachine states ────────────────────────────────────────────────
#define wacht_op_start    0
#define start_vooruit     1
#define wacht_op_tag1     2
#define route_keuze       3
#define zoek_gang         4
#define draai_links       5
#define zoek_route_plus_6 6
#define stop_bij_route    7
#define vrij_rijden       8

// ─── RFID ─────────────────────────────────────────────────────────────────────
MFRC522 mfrc522(SS_PIN, RST_PIN);
MFRC522::MIFARE_Key key;

// ─── Noodstop interrupt ───────────────────────────────────────────────────────
volatile bool noodStopActief      = false;
volatile unsigned long noodStopTijd = 0;
const unsigned long noodDebounce  = 200;  // ms

void IRAM_ATTR noodStopISR() {
  unsigned long nu_isr = millis();
  if (nu_isr - noodStopTijd > noodDebounce) {
    noodStopActief = true;
    noodStopTijd   = nu_isr;
  }
}

// ─── Globale variabelen ───────────────────────────────────────────────────────
byte huidigeStatus = wacht_op_start;

// Routekeuze
byte knopTeller         = 0;
unsigned long knopStartTijd = 0;
const unsigned long knopTimeout = 5000;

unsigned long nu;

// Gekozen depot
byte gekozenGang = 0;
byte doelTag     = 0;

// Obstakel debounce
unsigned long lastTriggerTime   = 0;
const unsigned long debounceDelay = 100;

// Edge detectie knoppen
bool laatsteStartStatus    = HIGH;
bool laatsteSelectieStatus = HIGH;

// Ultrasoon afstand resultaten (globaal, gevuld door ping())
float distanceL = -1;
float distanceR = -1;

// ─── PWM hulpfunctie ──────────────────────────────────────────────────────────
void motorPWM(bool wiel, uint8_t snelheid) {
  if (wiel == links) {
    ledcWrite(LEDC_CH_A, snelheid);
  } else {
    ledcWrite(LEDC_CH_B, snelheid);
  }
}

// ─── Motor aansturing ─────────────────────────────────────────────────────────
void setMotor(bool wiel, bool richting, uint8_t snelheid) {
  if (snelheid == 0) {
    if (wiel == links) {
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      motorPWM(links, 0);
    } else {
      digitalWrite(in3, LOW);
      digitalWrite(in4, LOW);
      motorPWM(rechts, 0);
    }
    return;
  }

  if (wiel == links) {
    digitalWrite(in1, richting == vooruit ? HIGH : LOW);
    digitalWrite(in2, richting == vooruit ? LOW  : HIGH);
    motorPWM(links, snelheid);
  } else {
    digitalWrite(in3, richting == vooruit ? HIGH : LOW);
    digitalWrite(in4, richting == vooruit ? LOW  : HIGH);
    motorPWM(rechts, snelheid);
  }
}

void forward(uint8_t s)   { setMotor(links, vooruit,    s); setMotor(rechts, vooruit,    s); }
void backwards(uint8_t s) { setMotor(links, achteruit,  s); setMotor(rechts, achteruit,  s); }
void left(uint8_t s)      { setMotor(links, achteruit,  s); setMotor(rechts, vooruit,    s); }
void stopMotors()         { setMotor(links, vooruit,    0); setMotor(rechts, vooruit,    0); }

// ─── Noodstop verwerking ──────────────────────────────────────────────────────
bool checkNoodKnop() {
  if (noodStopActief) {
    noodStopActief = false;

    digitalWrite(in1, LOW); digitalWrite(in2, LOW); ledcWrite(LEDC_CH_A, 0);
    digitalWrite(in3, LOW); digitalWrite(in4, LOW); ledcWrite(LEDC_CH_B, 0);

    huidigeStatus = wacht_op_start;
    return true;
  }
  return false;
}

// ─── Veilige delay met sensor- en noodstopcontrole ────────────────────────────
bool safetyDelay(unsigned long ms) {
  unsigned long startTijd = millis();
  while (millis() - startTijd < ms) {
    if (noodStopActief)                 return false;
    if (digitalRead(ir_achter) == HIGH) return false;
    delay(10);
  }
  return true;
}

// ─── Obstakel vermijding ──────────────────────────────────────────────────────
void obstakelVermijding() {
  byte rechtsSensor = digitalRead(ir_rechts);
  byte linksSensor  = digitalRead(ir_links);
  byte achterSensor = digitalRead(ir_achter);

  if (rechtsSensor && nu - lastTriggerTime > debounceDelay) {
    lastTriggerTime = nu;
    if (!achterSensor) backwards(200);
    safetyDelay(300);
    left(200);
    safetyDelay(300);
  } else if (linksSensor && nu - lastTriggerTime > debounceDelay) {
    lastTriggerTime = nu;
    if (!achterSensor) backwards(200);
    safetyDelay(300);
    setMotor(links,  vooruit,    200);
    setMotor(rechts, achteruit,  200);
    safetyDelay(300);
  } else {
    forward(200);
  }
}

// ─── RFID lezen ───────────────────────────────────────────────────────────────
byte leesRFIDGetal() {
  if (!mfrc522.PICC_IsNewCardPresent()) return 0;
  if (!mfrc522.PICC_ReadCardSerial())   return 0;

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

  // Tags 10–14: b0='1', b1='0'..'4'
  if (b0 == 0x31 && b1 >= 0x30 && b1 <= 0x39) {
    byte getal = 10 + (b1 - 0x30);
    if (getal >= 10 && getal <= 14) return getal;
  }

  // Tags 1–9: b0='1'..'9', b1=0x00
  if (b0 >= 0x31 && b0 <= 0x39 && b1 == 0x00) {
    return b0 - 0x30;
  }

  return 0;
}

// ─── MAD filter ───────────────────────────────────────────────────────────────
// Filtert uitschieters uit een reeks metingen via de Median Absolute Deviation.
// Geeft het gemiddelde terug van de metingen binnen 2.5 * MAD van de mediaan.
float filtered_average_mad(float *readings) {
  float sorted[NUM_READINGS];
  for (int i = 0; i < NUM_READINGS; i++) {
    sorted[i] = readings[i];
  }

  // Bubblesort
  for (int i = 0; i < NUM_READINGS - 1; i++) {
    for (int j = 0; j < NUM_READINGS - 1 - i; j++) {
      if (sorted[j] > sorted[j + 1]) {
        float temp  = sorted[j];
        sorted[j]   = sorted[j + 1];
        sorted[j + 1] = temp;
      }
    }
  }

  // Mediaan van 8 waarden (gemiddelde van de twee middelste)
  float median = (sorted[3] + sorted[4]) / 2.0f;

  // Bereken afwijkingen
  for (int i = 0; i < NUM_READINGS; i++) {
    sorted[i] = fabsf(readings[i] - median);
  }

  // Sorteer afwijkingen
  for (int i = 0; i < NUM_READINGS - 1; i++) {
    for (int j = 0; j < NUM_READINGS - 1 - i; j++) {
      if (sorted[j] > sorted[j + 1]) {
        float temp  = sorted[j];
        sorted[j]   = sorted[j + 1];
        sorted[j + 1] = temp;
      }
    }
  }

  float mad       = (sorted[3] + sorted[4]) / 2.0f;
  float threshold = 2.5f * mad;
  float sum       = 0;
  int   valid     = 0;

  for (int i = 0; i < NUM_READINGS; i++) {
    if (fabsf(readings[i] - median) <= threshold) {
      sum += readings[i];
      valid++;
    } else {
      Serial.print("Uitgesloten waarde: ");
      Serial.println(readings[i]);
    }
  }

  return (valid > 0) ? sum / valid : median;
}

// ─── Ultrasoon ping (beide sensoren, MAD gefilterd) ───────────────────────────
// Vult de globale variabelen distanceL en distanceR (in cm).
// distanceL / distanceR == -1 bij onvoldoende geldige metingen.
void ping() {
  float readingsL[NUM_READINGS];
  float readingsR[NUM_READINGS];
  int   validL = 0;
  int   validR = 0;

  for (int i = 0; i < NUM_READINGS; i++) {
    // ── Links ──
    digitalWrite(trigPinL, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPinL, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPinL, LOW);

    // pulseIn werkt op ESP32; timeout 30 ms voorkomt blokkering bij geen echo
    unsigned long durationL = pulseIn(echoPinL, HIGH, 30000UL);
    if (durationL > 0) {
      readingsL[validL++] = (durationL * 0.0343f) / 2.0f;
    }

    delay(60);

    // ── Rechts ──
    digitalWrite(trigPinR, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPinR, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPinR, LOW);

    unsigned long durationR = pulseIn(echoPinR, HIGH, 30000UL);
    if (durationR > 0) {
      readingsR[validR++] = (durationR * 0.0343f) / 2.0f;
    }

    delay(60);
  }

  // Minimaal 4 (= NUM_READINGS / 2) geldige metingen nodig voor betrouwbare MAD
  distanceL = (validL >= NUM_READINGS / 2) ? filtered_average_mad(readingsL) : -1.0f;
  distanceR = (validR >= NUM_READINGS / 2) ? filtered_average_mad(readingsR) : -1.0f;
}

// ─── Setup ────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  // Motor output pinnen
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // LEDC PWM kanalen koppelen aan enable-pinnen
  ledcSetup(LEDC_CH_A, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(enA, LEDC_CH_A);

  ledcSetup(LEDC_CH_B, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(enB, LEDC_CH_B);

  // IR sensoren
  pinMode(ir_rechts, INPUT);
  pinMode(ir_links,  INPUT);
  pinMode(ir_achter, INPUT);

  // Knoppen
  pinMode(start_knop,      INPUT_PULLUP);
  pinMode(selectie_knop,   INPUT_PULLUP);
  pinMode(pakket_detectie, INPUT_PULLUP);

  // Noodknop als hardware interrupt (FALLING = actief laag)
  pinMode(nood_knop, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(nood_knop), noodStopISR, FALLING);

  // Ultrasoon sensoren
  pinMode(trigPinL, OUTPUT);
  pinMode(echoPinL, INPUT);
  pinMode(trigPinR, OUTPUT);
  pinMode(echoPinR, INPUT);

  // RFID — expliciete VSPI pinnen
  SPI.begin(18, 19, 23, SS_PIN);  // SCK, MISO, MOSI, SS
  mfrc522.PCD_Init();

  for (byte i = 0; i < 6; i++) key.keyByte[i] = 0xFF;
}

// ─── Loop ─────────────────────────────────────────────────────────────────────
void loop() {
  // Noodstop altijd als eerste verwerken
  if (checkNoodKnop()) return;

  nu = millis();

  bool startNu    = digitalRead(start_knop)    == LOW;
  bool selectieNu = digitalRead(selectie_knop) == LOW;

  switch (huidigeStatus) {

    // ── Wacht tot startknop wordt ingedrukt ──────────────────────────────────
    case wacht_op_start:
      stopMotors();
      if (startNu && laatsteStartStatus == HIGH)
        huidigeStatus = start_vooruit;
      break;

    // ── Rij vooruit, ga direct naar tag 1 zoeken ─────────────────────────────
    case start_vooruit:
      forward(200);
      huidigeStatus = wacht_op_tag1;
      break;

    // ── Rij vooruit tot RFID tag 1 wordt gelezen ─────────────────────────────
    case wacht_op_tag1:
      forward(200);
      if (leesRFIDGetal() == 1) {
        stopMotors();
        knopTeller    = 0;
        knopStartTijd = millis();
        huidigeStatus = route_keuze;
      }
      break;

    // ── Selectieknop telt het gewenste depot (1–6), timeout bevestigt keuze ──
    case route_keuze:
      stopMotors();

      if (selectieNu && laatsteSelectieStatus == HIGH) {
        knopTeller++;
        if (knopTeller > 6) knopTeller = 1;
        knopStartTijd = millis();
      }

      if (knopTeller > 0 && millis() - knopStartTijd > knopTimeout) {
        gekozenGang   = knopTeller + 1;
        doelTag       = gekozenGang + 6;
        huidigeStatus = zoek_gang;
      }
      break;

    // ── Rij met obstakel vermijding tot de RFID tag van de gekozen gang ───────
    case zoek_gang:
      obstakelVermijding();
      if (leesRFIDGetal() == gekozenGang) {
        stopMotors();
        delay(500);
        huidigeStatus = draai_links;
      }
      break;

    // ── Draai links de gang in ────────────────────────────────────────────────
    case draai_links:
      left(200);
      if (safetyDelay(600)) {
        stopMotors();
        huidigeStatus = zoek_route_plus_6;
      }
      break;

    // ── Rij de gang in tot doel-tag (gang + 6) ────────────────────────────────
    case zoek_route_plus_6:
      forward(200);
      if (leesRFIDGetal() == doelTag) {
        stopMotors();
        huidigeStatus = stop_bij_route;
      }
      break;

    // ── Stop en wacht op pakketdetectie schakelaar ────────────────────────────
    case stop_bij_route:
      stopMotors();
      if (digitalRead(pakket_detectie) == LOW)
        huidigeStatus = vrij_rijden;
      break;

    // ── Vrij rijden met obstakel vermijding ───────────────────────────────────
    case vrij_rijden:
      obstakelVermijding();
      break;
  }

  // Edge detectie opslaan voor volgende iteratie
  laatsteStartStatus    = startNu;
  laatsteSelectieStatus = selectieNu;

  delay(10);
}