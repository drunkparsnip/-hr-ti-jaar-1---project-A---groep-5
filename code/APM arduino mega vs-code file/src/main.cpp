#include <Arduino.h>
#include <SPI.h>
#include <MFRC522.h>

// Motor A (links) 
#define enA 11 // PWM-geschikt
#define in1 5
#define in2 6

// Motor B (rechts) 
#define enB 12 // PWM-geschikt
#define in3 4
#define in4 3

// Knoppen 
#define nood_knop 20
#define start_knop 32
#define route_selectie 33
#define pakket_detectie 34

// IR sensor
#define IR_links 22
#define IR_rechts 23
#define IR_achter 21

// Ultrasoon sensoren
#define trigPinL 9
#define echoPinL 10
#define trigPinR 7
#define echoPinR 8

// RFID
#define RST_PIN 49
#define SS_PIN 53

// LED
#define ROOD_PIN 25
#define GROEN_PIN 26 
#define BLAUW_PIN 27

// Richtingen 
#define links     0
#define rechts    1
#define vooruit   0
#define achteruit 1

#define NUM_READINGS 8

// main statusmachine states 
#define klaar_voor_start 0
#define depot_selectie 1
#define onderweg_naar_depot 2
#define wachten_op_pakket 3
#define wacht_met_wegrijden 4
#define onderweg_naar_eindpunt 5
#define op_eindpunt 6
#define vrij_rijden 7

// driving states 
#define ForewardRijsnelheid 255
#define ReverseRijsnelheid 100
#define clear 0
#define rightDodge 1
#define leftDodge 2
#define backDodge 3

// Non-blocking ping state 
// #define PING_IDLE   0
// #define PING_TRIG_L 1
// #define PING_WAIT_L 2
// #define PING_TRIG_R 3
// #define PING_WAIT_R 4

MFRC522 mfrc522(SS_PIN, RST_PIN);

// Ping Variabelen 
byte pingState             = PING_IDLE;
unsigned long pingTimer    = 0;
int           pingIndex    = 0;
float readingsL[NUM_READINGS];
float readingsR[NUM_READINGS];
int   validL = 0, validR  = 0;


// Noodstop interrupt 
volatile bool noodStopActief      = false;
volatile unsigned long noodStopTijd = 0;
const unsigned long noodDebounce  = 200;  // ms


// Globale variabelen 
byte huidigeStatus = klaar_voor_start;
bool statusVeranderd = false;

// Routekeuze
byte knopTeller = 0;
unsigned long knopStartTijd = 0;
const unsigned long knopTimeout = 5000;

unsigned long nu;

// ping regel variables
unsigned long wachtPingR = 0;
unsigned long wachtPingL = 0;
const uint8_t pingDelay = 60;
bool PingAanDeBeurt = links;


// Gekozen depot
byte gekozenGang = 0;

// Obstakel debounce
unsigned long laatsteHitTijd   = 0;
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

// kleur enum
enum Kleur {rood, blauw, groen, geel, cyaan, paars, wit, knipperen};


void setMotor(bool wiel, bool richting, uint8_t snelheid) {
  uint8_t inPin1, inPin2, enPin;
  
  if (wiel == links) {
    inPin1 = in1;
    inPin2 = in2;
    enPin = enA;
  } else { // rechts
    inPin1 = in3;
    inPin2 = in4;
    enPin = enB;
  }
 
  if (snelheid == 0) {
    digitalWrite(inPin1, LOW);
    digitalWrite(inPin2, LOW);
    analogWrite(enPin, 0);
  } else {
    if (richting == vooruit) {
      digitalWrite(inPin1, HIGH);
      digitalWrite(inPin2, LOW);
    } else {
      digitalWrite(inPin1, LOW);
      digitalWrite(inPin2, HIGH);
    }
    analogWrite(enPin, snelheid);
  }
}

void forward(uint8_t s)   { setMotor(links, vooruit,    s); setMotor(rechts, vooruit,    s); }
void backwards(uint8_t s) { setMotor(links, achteruit,  s); setMotor(rechts, achteruit,  s); }
void left(uint8_t s)      { setMotor(links, achteruit,  s); setMotor(rechts, vooruit,    s); }
void right(uint8_t s)     { setMotor(links, vooruit,    s); setMotor(rechts, achteruit,  s); }
void stopMotors()         { setMotor(links, vooruit,    0); setMotor(rechts, vooruit,    0); }

void setColor(bool redValue, bool greenValue,  bool blueValue) {
  digitalWrite(ROOD_PIN, redValue);
  digitalWrite(GROEN_PIN, greenValue);
  digitalWrite(BLAUW_PIN, blueValue);
}

void rgbLED(Kleur kleur) {
  switch (kleur) {
    case rood:
      setColor(0, 1, 1);
      break;

    case groen:
      setColor(1, 0, 1);
      break;

    case blauw:
      setColor(1, 1, 0);
      break;

    case geel:
      setColor(0, 0, 1);
      break;   

    case cyaan:
      setColor(1, 0, 0);
      break;

    case paars:
      setColor(0, 1, 0);
      break;

    case wit:
      setColor(0, 0, 0);
      break; 

    case knipperen: 
      setColor(0, 0, 0);
      delay(200);
      setColor(1, 1, 1);
      delay(200);
      setColor(0, 0, 0);
      delay(200);
      setColor(1, 1, 1);
      delay(200);
      setColor(0, 0, 0);
      delay(200);
      setColor(1, 1, 1);
      delay(200);
      setColor(0, 0, 0);
  }
}

void checkStatus(uint8_t status) {
  switch (status) {
    case depot_selectie: rgbLED(rood); break;
    case klaar_voor_start: rgbLED(groen); break;
    case onderweg_naar_depot: rgbLED(blauw); break;
    case wachten_op_pakket: rgbLED(geel); break;
    case onderweg_naar_eindpunt: rgbLED(cyaan); break;
    case vrij_rijden: rgbLED(paars); break;
    case op_eindpunt: rgbLED(wit); break;
    case wacht_met_wegrijden: rgbLED(knipperen); break;
  }
}

void RGBRouteSelecter(byte gekozenGang) {

}

bool checkNoodKnop() {
  if (noodStopActief) {
    noodStopActief = false;

    digitalWrite(in1, LOW); digitalWrite(in2, LOW);
    digitalWrite(in3, LOW); digitalWrite(in4, LOW);

    huidigeStatus = klaar_voor_start;
    return true;
  }
  return false;
}

bool safetyDelay(unsigned long ms) {
  unsigned long startTijd = millis();
  while (millis() - startTijd < ms) {
    if (noodStopActief) return false;
    if (digitalRead(IR_achter)) return false;
    delay(10);
  }
  return true;
}

void achteruitInterupt() {
  forward(ForewardRijsnelheid);
}

void stopButtonInterupt() {
  stopMotors();
  // uitendeijlk de stopped state hier
}

// MAD filter function
float filtered_average_mad(float *readings) {
    // Sort a copy to find median
    float sorted[NUM_READINGS];
    for(int i = 0; i < NUM_READINGS; i++) {
        sorted[i] = readings[i];
    }
    
    // Bubble sort
    for(int i = 0; i < NUM_READINGS - 1; i++) {
        for(int j = 0; j < NUM_READINGS - 1 - i; j++) {
            if(sorted[j] > sorted[j + 1]) {
                float temp = sorted[j];
                sorted[j] = sorted[j + 1];
                sorted[j + 1] = temp;
            }
        }
    }
    
    // Median of 8 values (average of middle two)
    float median = (sorted[3] + sorted[4]) / 2.0;
    
    // Calculate deviations and reuse sorted array
    for(int i = 0; i < NUM_READINGS; i++) {
        sorted[i] = abs(readings[i] - median);  
    }
    
    // Sort deviations
    for(int i = 0; i < NUM_READINGS - 1; i++) {
        for(int j = 0; j < NUM_READINGS - 1 - i; j++) {
            if(sorted[j] > sorted[j + 1]) {
                float temp = sorted[j];
                sorted[j] = sorted[j + 1];
                sorted[j + 1] = temp;
            }
        }
    }
    
    // MAD (median of 8 values)
    float mad = (sorted[3] + sorted[4]) / 2.0;
    
    // Filter outliers and calculate average
    float threshold = 2.5 * mad;
    float sum = 0;
    int valid_count = 0;
    
    for (int i = 0; i < NUM_READINGS; i++) {
        if(abs(readings[i] - median) <= threshold) {
            sum += readings[i];
            valid_count++;
        } else {
            Serial.print("Excluded value: ");
            Serial.println(readings[i]);
        }
    }
    
    return (valid_count > 0) ? sum / valid_count : median;
}

// Updated ping function that updates both sensors
void ping(){
  if (nu > wachtPingL && PingAanDeBeurt == links) {
    float readingsL[NUM_READINGS];
    int validReadingsL = 0;

    for (int i = 0; i < NUM_READINGS; i++) {
      digitalWrite(trigPinL, LOW);
      delayMicroseconds(2);
      digitalWrite(trigPinL, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPinL, LOW);

      float durationL = pulseIn(echoPinL, HIGH, 30000);
      if (durationL > 0) {
        readingsL[validReadingsL++] = (durationL * 0.0343) / 2;
      }
    }

    distanceL = (validReadingsL >= 4) ? filtered_average_mad(readingsL) : -1;

    wachtPingR = nu + pingDelay;
    PingAanDeBeurt = rechts;
  }
  else if (nu > wachtPingR && PingAanDeBeurt == rechts) {
    float readingsR[NUM_READINGS];
    int validReadingsR = 0;

    for (int i = 0; i < NUM_READINGS; i++) {  // ← for-loop toegevoegd
      digitalWrite(trigPinR, LOW);
      delayMicroseconds(2);
      digitalWrite(trigPinR, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPinR, LOW);

      float durationR = pulseIn(echoPinR, HIGH, 30000);
      if (durationR > 0) {
        readingsR[validReadingsR++] = (durationR * 0.0343) / 2;
      }
    }

    distanceR = (validReadingsR >= 4) ? filtered_average_mad(readingsR) : -1;

    wachtPingL = nu + pingDelay;
    PingAanDeBeurt = links;
  }
  Serial.print("links: ");
  Serial.print(distanceL);
  Serial.print(" rechts: ");
  Serial.println(distanceR);
}

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

byte selectRoute() {
  bool selectieNu = digitalRead(route_selectie);

  if (selectieNu == LOW && laatsteSelectieStatus == HIGH) {
    if (knopTeller == 0) knopStartTijd = nu;
    knopTeller++;
  }
  laatsteSelectieStatus = selectieNu;

  if (knopTeller > 0 && nu - knopStartTijd > knopTimeout) {
    byte gang = knopTeller + 1; // 1x = gang 2, 2x = gang 3 enz.

    if (gang < 2 || gang > 7) {
      knopTeller = 0;
      return 0;
    }

    knopTeller = 0;
    return gang;
  }

  return 0;
}

void rfidCheck() {
  byte rfidGang = leesRFIDGetal();
  if (rfidGang == 0) return;

  stopMotors();

  if (rfidGang == gekozenGang) {
    stopMotors();

  }
}

void handleStates(){
    switch (huidigeStatus) {

      case klaar_voor_start:
        if ((digitalRead(start_knop))
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
       if (depotBereikt){
      huidigeStatus = wachten_op_pakket;
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
      if (bestemminBereikt){
        huidigeStatus = vrij_rijden 
      statusVeranderd = true;
      break;

      case vrij_rijden:
      // code hier
      statusVeranderd = true;
      break;

  }
}

void lijnDetectie() {
  ping();

  bool rechtsSensorIR = digitalRead(IR_rechts);
  bool linksSensorIR = digitalRead(IR_links);
  bool achterSensorIR = digitalRead(IR_achter);

  if (rechtsSensorIR && nu - laatsteHitTijd > debounceDelay) {
    laatsteHitTijd = nu;

    if (!achterSensorIR) backwards(ReverseRijsnelheid);

    safetyDelay(500);
    left(ForewardRijsnelheid);
    safetyDelay(200);

  } else if (linksSensorIR && nu - laatsteHitTijd > debounceDelay) {
    laatsteHitTijd = nu;

    if (!achterSensorIR) backwards(ReverseRijsnelheid);

    safetyDelay(500);
    right(ForewardRijsnelheid);
    safetyDelay(200);
    
  } else if (distanceL < 20 && distanceL > 0){ 
    right(ForewardRijsnelheid);

  } else if (distanceR < 20 && distanceR > 0){
    left(ForewardRijsnelheid);

  } else {
      forward(ForewardRijsnelheid);
  }
}

void setup() {
  Serial.begin(9600);

  // RFID
  SPI.begin();
  mfrc522.PCD_Init();
  
  // Motoren
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // LEDs
  pinMode(ROOD_PIN, OUTPUT);
  pinMode(GROEN_PIN, OUTPUT);
  pinMode(BLAUW_PIN, OUTPUT);

  // Knoppen
  pinMode(nood_knop, INPUT_PULLUP);
  pinMode(start_knop, INPUT_PULLUP);
  pinMode(route_selectie, INPUT_PULLUP);
  pinMode(pakket_detectie, INPUT_PULLUP);

  // IR sensor
  pinMode(IR_achter, INPUT);
  pinMode(IR_rechts, INPUT);
  pinMode(IR_links, INPUT);

  // ultrasone sensor
  pinMode(trigPinL, OUTPUT);
  pinMode(echoPinL, INPUT);
  pinMode(trigPinR, OUTPUT);
  pinMode(echoPinR, INPUT);

  // interupts
  attachInterrupt(digitalPinToInterrupt(21), achteruitInterupt, FALLING);
}

void loop() {
  nu = millis();
  lijnDetectie();
  ping();
}