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
// ir connections
#define ir_rechts 6
#define ir_links 2

// IR detection states
#define IR_NONE 0
#define IR_LEFT 1
#define IR_RIGHT 2
#define IR_BOTH_LEFT_FIRST 3
#define IR_BOTH_RIGHT_FIRST 4

// Navigation states
#define NAV_FORWARD 0
#define NAV_BACKWARD 1
#define NAV_ROTATE_LEFT 2
#define NAV_ROTATE_RIGHT 3
#define NAV_ROTATE_LEFT_EXTRA 4
#define NAV_ROTATE_RIGHT_EXTRA 5

unsigned long IRdebounceL = 0, IRdebounceR = 0;
uint8_t debounceDelay = 50;
uint8_t coincidenceWindow = 100; // Time window to detect "both" scenarios
unsigned long currentTime;

// Navigation state tracking
uint8_t navState = NAV_FORWARD;
unsigned long navStateStartTime = 0;
uint16_t navDuration = 0;

// <---------- MOTOR CONTROL FUNCTIONS ---------->
void stopMotor(bool wiel){
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

void setMotor(bool wiel, bool richting, uint8_t snelheid){
  uint8_t inPin1, inPin2, enPin;
  
  if (snelheid == 0){
    stopMotor(wiel);
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

// <---------- SENSOR CONTROL ---------->
uint8_t readIR() {
  static bool leftTriggered = false;
  static bool rightTriggered = false;
  static unsigned long firstTriggerTime = 0;
  static uint8_t firstSensor = IR_NONE;
  
  bool sensorValueRechts = digitalRead(ir_rechts);
  bool sensorValueLinks = digitalRead(ir_links);
  
  // Debouncing for left sensor
  if (sensorValueLinks == HIGH) {
    if (currentTime - IRdebounceL >= debounceDelay) {
      IRdebounceL = currentTime;
      
      // Check if this is the first trigger
      if (!leftTriggered && !rightTriggered) {
        leftTriggered = true;
        firstTriggerTime = currentTime;
        firstSensor = IR_LEFT;
      } else if (rightTriggered && (currentTime - firstTriggerTime <= coincidenceWindow)) {
        // Right was first, now left detected within window
        leftTriggered = false;
        rightTriggered = false;
        firstSensor = IR_NONE;
        return IR_BOTH_RIGHT_FIRST;
      }
    }
  }
  
  // Debouncing for right sensor
  if (sensorValueRechts == HIGH) {
    if (currentTime - IRdebounceR >= debounceDelay) {
      IRdebounceR = currentTime;
      
      // Check if this is the first trigger
      if (!rightTriggered && !leftTriggered) {
        rightTriggered = true;
        firstTriggerTime = currentTime;
        firstSensor = IR_RIGHT;
      } else if (leftTriggered && (currentTime - firstTriggerTime <= coincidenceWindow)) {
        // Left was first, now right detected within window
        leftTriggered = false;
        rightTriggered = false;
        firstSensor = IR_NONE;
        return IR_BOTH_LEFT_FIRST;
      }
    }
  }
  
  // Check if coincidence window has expired
  if ((leftTriggered || rightTriggered) && (currentTime - firstTriggerTime > coincidenceWindow)) {
    uint8_t result = firstSensor;
    leftTriggered = false;
    rightTriggered = false;
    firstSensor = IR_NONE;
    return result;
  }
  
  return IR_NONE;
}

// <---------- NAVIGATION ---------->
void navigate() {
  uint8_t irState = readIR();
  
  // Check if current navigation action is complete
  if (navState != NAV_FORWARD && (currentTime - navStateStartTime >= navDuration)) {
    navState = NAV_FORWARD;
  }
  
  // Execute current navigation state
  switch (navState) {
    case NAV_FORWARD:
      forward(255);
      
      // Check for new IR detections and change state accordingly
      if (irState == IR_LEFT) {
        Serial.println("links detecteert -> draai rechts");
        navState = NAV_BACKWARD;
        navStateStartTime = currentTime;
        navDuration = 300;
      }
      else if (irState == IR_RIGHT) {
        Serial.println("rechts detecteert -> draai links");
        navState = NAV_BACKWARD;
        navStateStartTime = currentTime;
        navDuration = 300;
      }
      else if (irState == IR_BOTH_LEFT_FIRST) {
        Serial.println("beide (links eerst) -> extra draai rechts");
        navState = NAV_BACKWARD;
        navStateStartTime = currentTime;
        navDuration = 300;
      }
      else if (irState == IR_BOTH_RIGHT_FIRST) {
        Serial.println("beide (rechts eerst) -> extra draai links");
        navState = NAV_BACKWARD;
        navStateStartTime = currentTime;
        navDuration = 300;
      }
      break;
      
    case NAV_BACKWARD:
      backwards(255);
      
      // Transition to rotation after backward movement
      if (currentTime - navStateStartTime >= navDuration) {
        // Determine which rotation based on last IR detection
        if (irState == IR_LEFT || irState == IR_BOTH_LEFT_FIRST) {
          navState = (irState == IR_BOTH_LEFT_FIRST) ? NAV_ROTATE_RIGHT_EXTRA : NAV_ROTATE_RIGHT;
          navDuration = (irState == IR_BOTH_LEFT_FIRST) ? 500 : 300; // Extra rotation for corner
        } else {
          navState = (irState == IR_BOTH_RIGHT_FIRST) ? NAV_ROTATE_LEFT_EXTRA : NAV_ROTATE_LEFT;
          navDuration = (irState == IR_BOTH_RIGHT_FIRST) ? 500 : 300; // Extra rotation for corner
        }
        navStateStartTime = currentTime;
      }
      break;
      
    case NAV_ROTATE_LEFT:
    case NAV_ROTATE_LEFT_EXTRA:
      left(255);
      break;
      
    case NAV_ROTATE_RIGHT:
    case NAV_ROTATE_RIGHT_EXTRA:
      right(255);
      break;
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

void loop() {
  currentTime = millis();
  navigate();
}