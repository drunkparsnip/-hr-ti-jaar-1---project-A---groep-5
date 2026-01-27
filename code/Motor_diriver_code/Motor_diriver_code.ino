// Motor A connections
#define enA 9
#define in1 8
#define in2 7
// Motor B connections
#define enB 3
#define in3 5
#define in4 4
// movement defines
#define links 0
#define rechts 1
#define vooruit 0
#define achteruit 1

 
void setMotor(bool wiel, bool richting, uint8_t snelheid){
  uint8_t inPin1, inPin2, enPin;
  
  if (snelheid == 0){
    if (wiel == links){
      digitalWrite(in1, LOW);  // Fixed: was in3
      digitalWrite(in2, LOW);  // Fixed: was in4
      analogWrite(enA, 0);     // Fixed: was enB
    }
    else if(wiel == links){
      digitalWrite(in3, LOW);  // Fixed: was in1
      digitalWrite(in4, LOW);  // Fixed: was in2
      analogWrite(enB, 0);     // Fixed: was enA
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
      digitalWrite(inPin1, HIGH);  // Fixed: case sensitivity
      digitalWrite(inPin2, LOW);   // Fixed: case sensitivity and function name
    }
    else if (richting == achteruit){
      digitalWrite(inPin1, LOW);   // Fixed: case sensitivity
      digitalWrite(inPin2, HIGH);  // Fixed: case sensitivity and function name
    }
    analogWrite(enPin, snelheid);
  }
}
 

 
void setup() {
  Serial.begin(9600);
  Serial.println("=== ULTRASONIC SENSOR DEBUG ===");
  
  // Set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
 
  // Turn off motors - Initial state
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}
 
void loop() {
  
}