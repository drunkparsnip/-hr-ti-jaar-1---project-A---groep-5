#include <Arduino.h>
#include <SPI.h>

// ─── Motor A (links) ──────────────────────────────────────────────────────────
#define enA 11 // PWM-geschikt
#define in1 5
#define in2 6

// ─── Motor B (rechts) ─────────────────────────────────────────────────────────
#define enB 12 // PWM-geschikt
#define in3 4
#define in4 3

// ─── knoppen ──────────────────────────────────
#define nood_knop 31
#define start_knop 32
#define route_selectie 33
#define pakket_detectie 34

// IR sensor
#define IR_links 22
#define IR_rechts 23
#define IR_achter 24

// ─── Ultrasoon sensoren ───────────────────────────────────────────────────────
#define trigPinL 9
#define echoPinL 10
#define trigPinR 7
#define echoPinR 8

// RFID
#define RST_PIN 49
#define SS_PIN 53

// LED
#define RED_PIN 25
#define GREEN_PIN 26 
#define BLUE_PIN 27

// ─── Richtingen ───────────────────────────────────────────────────────────────
#define links     0
#define rechts    1
#define vooruit   0
#define achteruit 1

#define klaar_voor_start 0
#define depot_selectie 1
#define onderweg_naar_depot 2
#define wachten_op_pakket 3
#define wacht_met_wegrijden 4
#define onderweg_naar_eindpunt 5
#define op_eindpunt 6
#define vrij_rijden 7

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

void setColor(int redValue, int greenValue,  int blueValue) {
  analogWrite(RED_PIN, redValue);
  analogWrite(GREEN_PIN, greenValue);
  analogWrite(BLUE_PIN, blueValue);
}

void rgbLED(Kleur kleur) {
  switch (kleurLED) {
    case rood:
      setColor(0, 255, 255);
      break;

    case green:
      setColor(255, 0, 255);
      break;

    case blauw:
      setColor(255, 255, 0);
      break;

    case geel:
      setColor(0, 0, 255);
      break;   

    case cyaan:
      setColor(255, 0, 0);
      break;

    case paars:
      setColor(0, 255, 0);
      break;

    case wit:
      setColor(0, 0, 0);
      break; 

    case knipperen: 
      setColor(0, 0, 0);
      delay(200)
      setColor(255, 255, 255);
      delay(200);
      setColor(0, 0, 0);
      delay(200);
      setColor(255, 255, 255);
      delay(200);
      setColor(0, 0, 0);
      delay(200);
      setColor(255, 255, 255);
      delay(200);
      setColor(0, 0, 0);
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

void RGBRouteSelecter(gekozenGang) {
  
}


void setup() {
  Serial.begin(115200); // ESP32 standaard is 115200
  
  // Motoren
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // LEDs
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);

  // Knoppen
  pinMode(nood_knop, INPUT);
  pinMode(start_knop, INPUT);
  pinMode(route_selectie, INPUT);
  pinMode(pakket_detectie, INPUT);

  // IR sensor
  pinMode(IR_achter, INPUT);
  pinMode(IR_rechts, INPUT);
  pinMode(IR_links, INPPUT);
  
  // Test run
  Serial.println("Motoren starten...");
}

void loop() {
}

