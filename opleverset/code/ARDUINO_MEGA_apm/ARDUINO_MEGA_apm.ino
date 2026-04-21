#include <Arduino.h>
#include <SPI.h>
#include <MFRC522.h>


// ----------------------- PIN NUMBER DEFINES -------------------------------------
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

// ----------------------------------- READABILITY DEFINES -------------------------------------
// Richtingen 
#define links     0
#define rechts    1
#define vooruit   0
#define achteruit 1

#define NUM_READINGS 4

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
#define ForwardRijsnelheid 255
#define ReverseRijsnelheid 150
#define clear 0
#define rightDodge 1
#define leftDodge 2
#define backDodge 3

// NOODSTOP_PAUZE
#define NOODSTOP_PAUZE 8

MFRC522 mfrc522(SS_PIN, RST_PIN);

// -------------------------------- VARIABELEN --------------------------------------


// Noodstop interrupt 
volatile bool noodStopActief      = false;
volatile bool forwardsOveride = false; 
volatile unsigned long noodStopTijd = 0;
const unsigned long noodDebounce  = 200;  // ms


// Globale variabelen 
byte huidigeStatus = klaar_voor_start;
bool statusVeranderd = false;
bool pakketGeplaatst = false;
bool gangGevonden = false;
bool depotGevonden = false;

// Routekeuze
byte knopTeller = 0;
unsigned long knopStartTijd = 0;
const unsigned long knopTimeout = 5000;
unsigned long laatsteKnopTijd = 0;
const unsigned long knopDebounce = 50; // ms

unsigned long nu;

// ping regel variables
unsigned long wachtPingR = 0;
unsigned long wachtPingL = 0;
const uint8_t pingDelay = 60;
bool PingAanDeBeurt = links;


// Gekozen depot
byte gekozenGang = 2;

// Obstakel debounce
unsigned long laatsteHitTijd   = 0;
const unsigned long obstakelDebounceDelay = 100;

// knopstatus
bool selectieNu;  
bool selectieIngedrukt = false;

// Edge detectie knop
bool laatsteSelectieStatus = HIGH;
const uint8_t selectieDebounceDelay = 100;
unsigned long selectieDebounceTime = 0;
// Ultrasoon afstand resultaten (globaal, gevuld door ping())
float distanceL = -1;
float distanceR = -1;

// kleur enum
enum Kleur {rood, blauw, groen, geel, cyaan, paars, wit};
bool 
  ledR = 0, 
  ledG = 0, 
  ledB= 0;
bool kleurupdate = false;
uint8_t ledKnipper = 0;

unsigned long ledKnipperTijd = 0;
const uint16_t slowLedKnipperDelay = 500;
const uint16_t fastLedKnipperDelay = 200;
bool ledKnipperToggle = 0;

// ----------------------- MOTOR FUNCTIES -----------------------------------------------------
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

// functies van de motortoestanden 
void forward(uint8_t s)   { setMotor(links, vooruit,    s); setMotor(rechts, vooruit,    s); }
void backwards(uint8_t s) { setMotor(links, achteruit,  s); setMotor(rechts, achteruit,  s); }
void left(uint8_t s)      { setMotor(links, achteruit,  s); setMotor(rechts, vooruit,    s); }
void right(uint8_t s)     { setMotor(links, vooruit,    s); setMotor(rechts, achteruit,  s); }
void stopMotors()         { setMotor(links, vooruit,    0); setMotor(rechts, vooruit,    0); }

// --------------------- MULTICOLOR LED FUNCTIONS --------------------------------------------
void setColor(bool redValue, bool greenValue,  bool blueValue) {
  digitalWrite(ROOD_PIN, redValue);
  digitalWrite(GROEN_PIN, greenValue);
  digitalWrite(BLAUW_PIN, blueValue);
}

// functie om de led te laten knipperen 
void updateColor() {
  if (ledKnipper != 0) {
    uint16_t knipperDelay = ledKnipper == 1 ? fastLedKnipperDelay : slowLedKnipperDelay;
    if (ledKnipperTijd < nu && ledKnipperToggle == 0) {
      setColor(ledR, ledG, ledB);
      ledKnipperToggle = 1;
      ledKnipperTijd = nu + knipperDelay;
    }

    else if (ledKnipperTijd < nu && ledKnipperToggle == 1) {
      setColor(1, 1, 1);
      ledKnipperToggle = 0;
      ledKnipperTijd = nu + knipperDelay;
    }
  }

  else if (kleurupdate) {
    setColor(ledR, ledG, ledB);
    kleurupdate = false;
  }
}

// stuurt de LED aan met kleur en wel of niet knipperen
void rgbLED(Kleur kleur, uint8_t knipper) {
  switch (kleur) {
    case rood:
      ledR = 0;
      ledG = 1;
      ledB = 1;
      //setColor(0, 1, 1);
      break;

    case groen:
      ledR = 1;
      ledG = 0;
      ledB = 1;
      //setColor(1, 0, 1);
      break;

    case blauw:
      ledR = 1;
      ledG = 1;
      ledB = 0;
      //setColor(1, 1, 0);
      break;

    case geel:
      ledR = 0;
      ledG = 0;
      ledB = 1;
      //setColor(0, 0, 1);
      break;   

    case cyaan:
      ledR = 1;
      ledG = 0;
      ledB = 0;
      //setColor(1, 0, 0);
      break;

    case paars:
      ledR = 0;
      ledG = 1;
      ledB = 0;
      //setColor(0, 1, 0);
      break;

    case wit:
      ledR = 0;
      ledG = 0;
      ledB = 0;
      //setColor(0, 0, 0);
      break; 
  }
  ledKnipper = knipper;
  kleurupdate = true;
}

void displayStatus(uint8_t status) {
  switch (status) {
    // case depot_selectie: rgbLED(rood, 0); break;
    case klaar_voor_start: rgbLED(groen, 2); break;
    case onderweg_naar_depot: rgbLED(blauw, 0); break;
    //case wachten_op_pakket: rgbLED(geel, 0); break;
    case onderweg_naar_eindpunt: rgbLED(cyaan, 0); break;
    case vrij_rijden: rgbLED(paars, 0); break;
    case op_eindpunt: rgbLED(wit, 0); break;
    case wacht_met_wegrijden: rgbLED(geel, 2); break;
  }
}

//------------------------- SAFETY FUNCTIONS ----------------------------------------------------
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
  forward(ForwardRijsnelheid);
}

void stopButtonInterupt() {
  stopMotors();
  huidigeStatus = klaar_voor_start;
  // uitendeijlk de stopped state hier
}

//----------------------- PING RELATED FUNCTIONS -------------------------------------------------
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
            // Serial.print("Excluded value: ");
            // Serial.println(readings[i]);
        }
    }
    
    return (valid_count > 0) ? sum / valid_count : median;
}

// Updated ping function that updates both ultrasonic sensors
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

    distanceL = (validReadingsL >= 2) ? filtered_average_mad(readingsL) : -1;

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

    distanceR = (validReadingsR >= 2) ? filtered_average_mad(readingsR) : -1;

    wachtPingL = nu + pingDelay;
    PingAanDeBeurt = links;
  }
  // Serial.print("links: ");
  // Serial.print(distanceL);
  // Serial.print(" rechts: ");
  // Serial.println(distanceR);
}

// --------------------------- APM ROUTE RELATED FUNCTIONS ---------------------------------------
// functie om RFID tag te lezen
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

  // Tweetallig: "10"-"15"
  if (b0 == 0x31 && b1 >= 0x30 && b1 <= 0x35) {
    return 10 + (b1 - 0x30);
  }

  // Eentallig: "1"-"9"
  if (b0 >= 0x31 && b0 <= 0x39 && b1 == 0x00) {
    return b0 - 0x30;
  }

  return 0;
}

// LED kleur afhankelijk van de gekozen gang in selectRoute
void RGBRouteSelecter(byte gekozenGang) {
  switch (gekozenGang) {
    case 2:
      rgbLED(rood, 1);
    break;

    case 3:
      rgbLED(groen, 1);
    break;

    case 4:
      rgbLED(blauw, 1);
    break;   

    case 5:
      rgbLED(geel, 1);
    break;

    case 6:
      rgbLED(cyaan, 1);
    break;
      
    case 7:
      rgbLED(paars, 1);
    break;      
  }   
}

// leest de RFID tag en voert iets uit adhv het nummer
void rfidCheck() {
  byte rfidGang = leesRFIDGetal();

  Serial.println(rfidGang);
  
  // geen gang/geen tag
  if (rfidGang == 0)
    return;

  stopMotors(); // stopt om de tag te lezen
  
  //eerste tag, na het vrij rijden stukje
  if (rfidGang == 1) {
    huidigeStatus = depot_selectie;
    return;
  }
  // eind punt
  else if (rfidGang == 15) {
    stopMotors();
    huidigeStatus = op_eindpunt;
    return;
  }
  //vergelijkt gelezen gang van de tag met de gekozenroute
  else if (rfidGang == gekozenGang) {
    stopMotors();
    gangGevonden = true;
    left(ForwardRijsnelheid);
    delay(770); //delay van 770 zorgt voor ongeveer een draai van 90 graden
    return;
  }
  else {
    return;
  }
}

// ---------------------------- MAIN PROGRAM FUNCTIONS ---------------------------------------

void handleStates(){
  do { // do while dus loopt ten minste een keer en is loop zodat de veranderde status gelijk gerund wordt
    statusVeranderd = false; // zet op false om de repeating loop niet oneindig door te laten gaan
    
    switch (huidigeStatus) { // de main switch case
      //------------------ klaar voor start state -------------------------
      case klaar_voor_start:
        if (digitalRead(start_knop) == LOW) { // check de startknop
          huidigeStatus = vrij_rijden;
          statusVeranderd = true;
        }
        else if (ReadSelectButton()) { // check de selectie knop
          huidigeStatus = depot_selectie;
          RGBRouteSelecter(gekozenGang);
          statusVeranderd = true;
        }
        break;

      case vrij_rijden:
        gekozenGang = 1;
        gangGevonden = false;
        lijnDetectie();
        rfidCheck();

        if(gangGevonden) {
          stopMotors();
          huidigeStatus = depot_selectie;
          statusVeranderd = true;
        }
        break;

      // ------------------ depot slectie state -------------------------
      case depot_selectie:
        if (ReadSelectButton()) { // schakel door de te kiezen depot tags (2-7)
          if (gekozenGang < 7){
            gekozenGang++;
          }
          else {
            gekozenGang = 2;
          }
          RGBRouteSelecter(gekozenGang);
        }
        
        else if (digitalRead(start_knop) == LOW){ // start reis naar depot
          huidigeStatus = onderweg_naar_depot;
          statusVeranderd = true;
        }
        break;

      // ----------------- onderweg naar depot state ------------------
      case onderweg_naar_depot:
        lijnDetectie();
        rfidCheck();
        if (gangGevonden) {
          // als tag 2-7 is gevonden, opzoek naar corresponderende tag 8-13
          if (!depotGevonden) {
            gangGevonden = false;
            gekozenGang += 6;
            depotGevonden = true;
          } 
          // tag 8-13 gevonden
          else { 
            gangGevonden = false;
            depotGevonden = false;
            huidigeStatus = wachten_op_pakket;
            stopMotors();
            statusVeranderd = true;
          }
        }
        
        break;
      // ----------------- WACHTEN OP PAT STATE ---------------------------
      case wachten_op_pakket:
        if (digitalRead(pakket_detectie) == LOW) {
          rgbLED(geel, 2);
        } else {
          rgbLED(geel, 0);
        }

        if (digitalRead(start_knop) == LOW) {
          huidigeStatus = onderweg_naar_eindpunt;
          statusVeranderd = true;
        }
        break;
        
      // ----------------- RIJDEN NAAR EINDPUNT STATE ----------------------
      case onderweg_naar_eindpunt:
        gekozenGang = 15;
        lijnDetectie();
        rfidCheck();

        if (gangGevonden) {
          stopMotors();
          huidigeStatus = op_eindpunt;
          statusVeranderd = true;
        }
        break;

      // ------------------ OP EINDPUNT ----------------------------------
      case op_eindpunt:
        stopMotors();
        break;
    }

    if (statusVeranderd) {statusVerandertUpdate();} 
  } while (statusVeranderd);
}

void statusVerandertUpdate(){
  displayStatus(huidigeStatus);
}

// zwarte lijndetectie door ir_sensors
void lijnDetectie() {
  ping();

  bool rechtsSensorIR = digitalRead(IR_rechts);
  bool linksSensorIR = digitalRead(IR_links);
  bool achterSensorIR = digitalRead(IR_achter);

  if (rechtsSensorIR && nu - laatsteHitTijd > obstakelDebounceDelay) {
    laatsteHitTijd = nu;

    backwards(ReverseRijsnelheid);

    safetyDelay(600);
    left(ForwardRijsnelheid);
    safetyDelay(300);

  } else if (linksSensorIR && nu - laatsteHitTijd > obstakelDebounceDelay) {
    laatsteHitTijd = nu;

    backwards(ReverseRijsnelheid);

    safetyDelay(600);
    right(ForwardRijsnelheid);
    safetyDelay(300);
    
  } else if (distanceL < 20 && distanceL > 0){ 
    right(ForwardRijsnelheid);

  } else if (distanceR < 20 && distanceR > 0){
    left(ForwardRijsnelheid);

  } else {
      forward(ForwardRijsnelheid);
  }
}

// ISR functie voor noodstop knop, deze code draait als de knop wordt ingedrukt
void knopISR(){
  unsigned long nu = millis();

  if(nu - noodStopTijd > noodDebounce){
    noodStopActief = true;
    noodStopTijd = nu;
    
    //stopMotors();
    
    // om het terug te zetten op de state klaar_voor_start, moet je dat in loop zetten m
    huidigeStatus = NOODSTOP_PAUZE;
  }
}
// ISR funcite tegen het achterover kuukelen van de APM van randjes af
void sensorISR(){
  unsigned long nu = millis();

  if(nu - noodStopTijd > noodDebounce){
    //noodStopActief = true;
    noodStopTijd = nu;
    
    //forward(ForwardRijsnelheid);   
  }
}

// geeft aan de status van de knop 
bool ReadSelectButton() {
  if (selectieDebounceTime < nu){
    selectieIngedrukt = !digitalRead(route_selectie);
    if (selectieIngedrukt && laatsteSelectieStatus){
      laatsteSelectieStatus = LOW;
      selectieDebounceTime = nu + selectieDebounceDelay;
      return true;
    }
    else if (!selectieIngedrukt && !laatsteSelectieStatus){
      laatsteSelectieStatus = HIGH;
      selectieDebounceTime = nu + selectieDebounceDelay;
    }
  }
  return false;
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
  attachInterrupt(digitalPinToInterrupt(21), sensorISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(20), knopISR, FALLING);

  displayStatus(huidigeStatus);
  // Serial.println("APM gestart");
  // Serial.print("huidige status: ");
  // Serial.println(huidigeStatus);
}

void loop() {
  nu = millis();
  updateColor();
  
  if (noodStopActief){
    stopMotors();
    if (digitalRead(start_knop) == LOW) {
      noodStopActief = false;
      huidigeStatus = klaar_voor_start;
      displayStatus(klaar_voor_start);
    }
  }
  else if (forwardsOveride){
    forward(ForwardRijsnelheid);
    delay(200);
  }
  else {
    handleStates();
  }
}