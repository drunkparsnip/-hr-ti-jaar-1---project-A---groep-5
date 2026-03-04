#include <Arduino.h>
#include <SPI.h>

// ─── UART met ESP 2 ───────────────────────────────────────────────────────────
#define RX_PIN 16
#define TX_PIN 17

// ─── Motor A (links) ──────────────────────────────────────────────────────────
#define enA 25  // PWM-geschikt
#define in1 26
#define in2 14

// ─── Motor B (rechts) ─────────────────────────────────────────────────────────
#define enB 33  // PWM-geschikt
#define in3 32
#define in4 16

// ─── Richtingen ───────────────────────────────────────────────────────────────
#define links     0
#define rechts    1
#define vooruit   0
#define achteruit 1

// ─── 74HC165 Shift Register ───────────────────────────────────────────────────
#define SR_LOAD 13   // SH/LD pin  — pulse LOW om inputs vast te leggen
#define SR_CLK   4   // klokpin
#define SR_DATA 36   // Q7 seriële uitgang — alleen invoer GPIO

// ─── Shift register bit posities ──────────────────────────────────────────────
// Sluit de componenten aan op de 74HC165 in deze volgorde (A=bit7 .. H=bit0)
#define BIT_IR_RECHTS       7   // A
#define BIT_IR_LINKS        6   // B
#define BIT_IR_ACHTER       5   // C
#define BIT_START_KNOP      4   // D
#define BIT_SELECTIE_KNOP   3   // E
#define BIT_PAKKET_DETECTIE 2   // F
// G en H ongebruikt, verbind met GND (lees als 0)

// ─── Noodknop ──────────────────────────────────
#define nood_knop 23

// ─── Ultrasoon sensoren ───────────────────────────────────────────────────────
// Pins gekozen zodat ze niet overlappen met bestaande pin-definities.
// Vrije output-capable GPIO's op ESP32: 2, 17, 21, 22
#define trigPinL 5
#define echoPinL 18
#define trigPinR 21
#define echoPinR 22

#define NUM_READINGS 8

// ─── LEDC PWM configuratie ────────────────────────────────────────────────────
#define PWM_FREQ       5000
#define PWM_RESOLUTION 8       // 0–255
#define LEDC_CH_A      0       // Kanaal motor A
#define LEDC_CH_B      1       // Kanaal motor B

// ─── main statusmachine states ────────────────────────────────────────────────
#define klaar_voor_start 0
#define depot_selectie 1
#define onderweg_naar_depot 2
#define wachten_op_pakket 3
#define wacht_met_wegrijden 4
#define onderweg_naar_eindpunt 5
#define op_eindpunt 6
#define vrij_rijden 7

// ─── driving states ───────────────────────────────────────────────────────────────

// ─── Non-blocking ping state ──────────────────────────────────────────────────
#define PING_IDLE   0
#define PING_TRIG_L 1
#define PING_WAIT_L 2
#define PING_TRIG_R 3
#define PING_WAIT_R 4

// ─── Ping Variabelen ───────────────────────────────────────────────────────────────
byte pingState             = PING_IDLE;
unsigned long pingTimer    = 0;
int           pingIndex    = 0;
float readingsL[NUM_READINGS];
float readingsR[NUM_READINGS];
int   validL = 0, validR  = 0;

// ─── Noodstop interrupt ───────────────────────────────────────────────────────
volatile bool noodStopActief      = false;
volatile unsigned long noodStopTijd = 0;
const unsigned long noodDebounce  = 200;  // ms

// ruwe byte gelezen van 74HC165, elke loop bijgewerkt
uint8_t srState = 0;

// individuele booleans afgeleid van srState voor eenvoudiger gebruik
bool sr_irRechts        = false;
bool sr_irLinks         = false;
bool sr_irAchter        = false;
bool sr_startKnop       = false;   // active-low op de shift inputs
bool sr_selectieKnop    = false;   // active-low
bool sr_pakketDetectie  = false;   // active-low

// ─── Globale variabelen ───────────────────────────────────────────────────────
byte huidigeStatus = klaar_voor_start;
bool statusVeranderd = false;

// Routekeuze
byte knopTeller = 0;
unsigned long knopStartTijd = 0;
const unsigned long knopTimeout = 5000;

unsigned long nu;

// Gekozen depot
byte gekozenGang = 0;

// Obstakel debounce
unsigned long lastTriggerTime   = 0;
const unsigned long debounceDelay = 100;

// knopstatussen
  bool startNu;
  bool selectieNu;
  bool pakketNu;

  bool startIngedrukt = false;
  bool selectieIngedrukt = false;
  bool pakketIngedrukt = false;

// Edge detectie knoppen
bool laatsteStartStatus    = HIGH;
bool laatsteSelectieStatus = HIGH;
bool laatstePakketStatus  = HIGH;

// paket timer variabelen
unsigned long pakketStartTijd = 0;
unsigned long pakketDelay = 10000; // 10 seconden

// Ultrasoon afstand resultaten (globaal, gevuld door ping())
float distanceL = -1;
float distanceR = -1;

// lees de parallelle inputs van de 74HC165 en werk de booleans bij
void readShiftInputs() {
  // leg de inputs vast in het shiftregister
  digitalWrite(SR_LOAD, LOW);
  delayMicroseconds(5);
  digitalWrite(SR_LOAD, HIGH);

  // klok acht bits uit, MSB eerst (A = bit7)
  srState = 0;
  for (int i = 0; i < 8; ++i) {
    srState <<= 1;
    if (digitalRead(SR_DATA)) srState |= 1;
    digitalWrite(SR_CLK, HIGH);
    delayMicroseconds(1);
    digitalWrite(SR_CLK, LOW);
    delayMicroseconds(1);
  }

  // werk booleans bij
  sr_irRechts       = (srState >> BIT_IR_RECHTS) & 1;
  sr_irLinks        = (srState >> BIT_IR_LINKS)  & 1;
  sr_irAchter       = (srState >> BIT_IR_ACHTER) & 1;
  // ingangen worden hoog gehouden; actieve toestand is LAAG
  sr_startKnop      = !((srState >> BIT_START_KNOP)     & 1);
  sr_selectieKnop   = !((srState >> BIT_SELECTIE_KNOP)  & 1);
  sr_pakketDetectie = !((srState >> BIT_PAKKET_DETECTIE)& 1);
}

void IRAM_ATTR noodStopISR() {
  unsigned long nu_isr = millis();
  if (nu_isr - noodStopTijd > noodDebounce) {
    noodStopActief = true;
    noodStopTijd   = nu_isr;
  }
}

// ─── PWM-hulpfunctie ──────────────────────────────────────────────────────────
void motorPWM(bool wiel, uint8_t snelheid) {
  if (wiel == links) {
    ledcWrite(LEDC_CH_A, snelheid);
  } else {
    ledcWrite(LEDC_CH_B, snelheid);
  }
}

// ─── Motorsturing ─────────────────────────────────────────────────────────────
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

// ─── Noodstopverwerking ──────────────────────────────────────────────────────
bool checkNoodKnop() {
  if (noodStopActief) {
    noodStopActief = false;

    digitalWrite(in1, LOW); digitalWrite(in2, LOW); ledcWrite(LEDC_CH_A, 0);
    digitalWrite(in3, LOW); digitalWrite(in4, LOW); ledcWrite(LEDC_CH_B, 0);

    huidigeStatus = klaar_voor_start;
    return true;
  }
  return false;
}

// ─── Veilige delay met sensor- en noodstopcontrole ────────────────────────────
bool safetyDelay(unsigned long ms) {
  unsigned long startTijd = millis();
  while (millis() - startTijd < ms) {
    if (noodStopActief) return false;
    readShiftInputs();
    if (sr_irAchter) return false; // gebruik boolean van shift register
    delay(10);
  }
  return true;
}

// ─── Obstakelvermijding ──────────────────────────────────────────────────────
void obstakelVermijding() {
  bool rechtsSensor = sr_irRechts;
  bool linksSensor  = sr_irLinks;
  bool achterSensor = sr_irAchter;

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

// ─── MAD-filter ───────────────────────────────────────────────────────────────
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

// ─── Ultrasoon ping (beide sensoren, MAD-gefilterd) ───────────────────────────
// Vult de globale variabelen distanceL en distanceR (in cm).
// distanceL / distanceR == -1 bij onvoldoende geldige metingen.
void ping() {
  unsigned long nu_ping = millis();

  if (pingState == PING_IDLE) {
    if (pingIndex >= NUM_READINGS) {
      distanceL = (validL >= NUM_READINGS / 2) ? filtered_average_mad(readingsL) : -1.0f;
      distanceR = (validR >= NUM_READINGS / 2) ? filtered_average_mad(readingsR) : -1.0f;
      pingIndex = 0;
      validL    = 0;
      validR    = 0;
    }
    digitalWrite(trigPinL, LOW);
    pingTimer = nu_ping;
    pingState = PING_TRIG_L;
  }

  else if (pingState == PING_TRIG_L) {
    if (nu_ping - pingTimer >= 1) {
      digitalWrite(trigPinL, HIGH);
      pingTimer = nu_ping;
      pingState = PING_WAIT_L;
    }
  }

  else if (pingState == PING_WAIT_L) {
    if (nu_ping - pingTimer >= 1) {
      digitalWrite(trigPinL, LOW);
      unsigned long dur = pulseIn(echoPinL, HIGH, 30000UL);
      if (dur > 0) readingsL[validL++] = (dur * 0.0343f) / 2.0f;
      pingTimer = nu_ping;
      pingState = PING_TRIG_R;
    }
  }

  else if (pingState == PING_TRIG_R) {
    if (nu_ping - pingTimer >= 60) {
      digitalWrite(trigPinR, LOW);
      pingTimer = nu_ping;
      pingState = PING_WAIT_R;
    }
  }

  else if (pingState == PING_WAIT_R) {
    if (nu_ping - pingTimer >= 1) {
      digitalWrite(trigPinR, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPinR, LOW);
      unsigned long dur = pulseIn(echoPinR, HIGH, 30000UL);
      if (dur > 0) readingsR[validR++] = (dur * 0.0343f) / 2.0f;
      pingIndex++;
      pingTimer = nu_ping;
      pingState = PING_IDLE;
    }
  }
}

// second mcu led control 
void serialLedControl(int8_t setColour) {
  // status sturen voor LED
  if (setColour == -1){
    Serial2.write(0xFF);
    Serial2.write(huidigeStatus);
  }
  else {
    Serial2.print("led_colour: ");
    Serial2.println(setColour);
  }
}
// ─── state machine ───────────────────────────────────────────────────────────────
void handleStates(){
    switch (huidigeStatus) {

      case klaar_voor_start:
        if (startIngedrukt){
          huidigeStatus = vrij_rijden;
          statusVeranderd = true;
        }
        else if (selectieIngedrukt){
          huidigeStatus = depot_selectie;
          statusVeranderd = true;
        }
      break;

      case depot_selectie:
        if (statusVeranderd){
          serialLedControl(gekozenGang);
        }

        else if (selectieIngedrukt){
          if (gekozenGang < 7){
            gekozenGang++;
          }
          else {
            gekozenGang = 2;
          }
        }
        else if (startIngedrukt){
          huidigeStatus = onderweg_naar_depot;
          statusVeranderd = true;
        }
      break;

      case onderweg_naar_depot:
      // code hier
      statusVeranderd = true;
      break;
      
      case wachten_op_pakket:
        if (pakketIngedrukt){
          huidigeStatus = wacht_met_wegrijden;
          statusVeranderd = true;
        }
      break;

      case wacht_met_wegrijden:
        if (pakketNu) {
          if (startIngedrukt){
            huidigeStatus = onderweg_naar_eindpunt;
            statusVeranderd = true;
          }
        }
        else {
          huidigeStatus = wachten_op_pakket;
          statusVeranderd = true;
        }
      break;

      case onderweg_naar_eindpunt:
      // code hier
      statusVeranderd = true;
      break;

      case op_eindpunt:
      // code hier
      statusVeranderd = true;
      break;

      case vrij_rijden:
      // code hier
      statusVeranderd = true;
      break;

  }
}

// ─── Setup ────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  // motoruitgangspinnen
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // LEDC-PWM-kanalen koppelen aan enable-pinnen
  ledcSetup(LEDC_CH_A, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(enA, LEDC_CH_A);

  ledcSetup(LEDC_CH_B, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(enB, LEDC_CH_B);

  // shift register control pins
  pinMode(SR_LOAD, OUTPUT);
  pinMode(SR_CLK,  OUTPUT);
  pinMode(SR_DATA, INPUT);

  // IR-sensoren (via 74HC165)
  // pinMode(BIT_IR_RECHTS, INPUT);
  // pinMode(BIT_IR_LINKS,  INPUT);
  // pinMode(BIT_IR_ACHTER, INPUT);

  // knoppen (via 74HC165)
  // pinMode(BIT_START_KNOP,      INPUT_PULLUP);
  // pinMode(BIT_SELECTIE_KNOP,   INPUT_PULLUP);
  // pinMode(BIT_PAKKET_DETECTIE, INPUT_PULLUP);

  // Noodknop als hardware interrupt (FALLING = actief laag)
  pinMode(nood_knop, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(nood_knop), noodStopISR, FALLING);

  // ultrasone sensoren
  pinMode(trigPinL, OUTPUT);
  pinMode(echoPinL, INPUT);
  pinMode(trigPinR, OUTPUT);
  pinMode(echoPinR, INPUT);
}

// ─── Loop ─────────────────────────────────────────────────────────────────────
void loop() {
  // Noodstop altijd als eerste verwerken
  if (checkNoodKnop()) return;

  nu = millis();

  // lees de parallelle inputs bij elke lus iteratie
  readShiftInputs();
  ping();

  startNu    = sr_startKnop;
  selectieNu = sr_selectieKnop;
  pakketNu    = sr_pakketDetectie;

  if (startNu && laatsteStartStatus == HIGH) {
    startIngedrukt = true;
  }
  else {
    startIngedrukt = false;
  }
  if (selectieNu && laatsteSelectieStatus == HIGH) {
    selectieIngedrukt = true;
  }
  else {
    selectieIngedrukt = false;
  }
  if (pakketNu && laatstePakketStatus == HIGH) {
    pakketIngedrukt = true;
  }
  else {
    pakketIngedrukt = false;
  }
  
  // update led controler
   if (statusVeranderd) {
    serialLedControl(huidigeStatus);
  }
  // edge-detectie opslaan voor volgende iteratie
  laatsteStartStatus    = startNu;
  laatsteSelectieStatus = selectieNu;
  laatstePakketStatus = pakketNu;

  // statusmachine verandering updaten
  statusVeranderd = false;

  delay(10);
}