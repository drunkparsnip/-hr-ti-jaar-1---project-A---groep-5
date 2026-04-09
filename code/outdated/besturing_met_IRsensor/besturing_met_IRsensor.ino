// Motor A connections
#define enA 9
#define in1 8
#define in2 7
// Motor B connections
#define enB 3
#define in3 5
#define in4 4
// movement defines
#define links 0      // AANGEPAST: was 1
#define rechts 1     // AANGEPAST: was 0
#define vooruit 0
#define achteruit 1

#define ir_rechts 6
#define ir_links 2

int sensorValueRechts;
int sensorValueLinks;

unsigned long lastTriggerTime = 0;
const unsigned long debounceDelay = 100;

 
void setMotor(bool wiel, bool richting, uint8_t snelheid){
  uint8_t inPin1, inPin2, enPin;
  
  if (snelheid == 0){
    if (wiel == links){
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      analogWrite(enA, 0);
    }
    else if(wiel == rechts){
      digitalWrite(in3, LOW);
      digitalWrite(in4, LOW);
      analogWrite(enB, 0);
    }
  }
  else {
    if (wiel == links){
      inPin1 = in1;
      inPin2 = in2;
      enPin = enA;
    }
    else if (wiel == rechts){
      inPin1 = in3;
      inPin2 = in4;
      enPin = enB;
    }
    
    if (richting == vooruit){
      digitalWrite(inPin1, HIGH);
      digitalWrite(inPin2, LOW);
    }
    else if (richting == achteruit){
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
 
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
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

void loop() {
  sensorValueRechts = digitalRead(ir_rechts);
  sensorValueLinks = digitalRead(ir_links);

  unsigned long currentTime = millis();
  
  forward(255);

  if (sensorValueLinks == 1 && sensorValueRechts == 1 && (currentTime - lastTriggerTime >= debounceDelay)) {
    lastTriggerTime = currentTime;

    unsigned long startTime = millis();
    while (millis() - startTime < 300) {
      backwards(255);
      delay(300);
    }
  }
  // Als RECHTS iets ziet -> draai naar LINKS
  else if (sensorValueRechts == 1 && (currentTime - lastTriggerTime >= debounceDelay)) {
    Serial.println("rechts detecteert -> draai links");  // AANGEPAST
    lastTriggerTime = currentTime;

    unsigned long startTime = millis();
    while (millis() - startTime < 300) { 
      backwards(255);
      delay(300);
      right(255);
      delay(300);
    }
  }
  // Als LINKS iets ziet -> draai naar RECHTS
  else if (sensorValueLinks == 1 && (currentTime - lastTriggerTime >= debounceDelay)) {
    Serial.println("links detecteert -> draai rechts");  // AANGEPAST
    lastTriggerTime = currentTime;

    unsigned long startTime = millis();
    while (millis() - startTime < 300) { 
      backwards(255);
      delay(300);
 
      left(255);
      delay(300);
      
    }
  }
}