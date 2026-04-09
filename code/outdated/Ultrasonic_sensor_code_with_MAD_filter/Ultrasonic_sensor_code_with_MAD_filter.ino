#define trigPinL 9
#define echoPinL 10
#define trigPinR 6
#define echoPinR 5

#define NUM_READINGS 8

float distanceL, distanceR;

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
    
    for(int i = 0; i < NUM_READINGS; i++) {
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
  float readingsL[NUM_READINGS];
  float readingsR[NUM_READINGS];
  int validReadingsL = 0;
  int validReadingsR = 0;
  
  // Collect readings from both sensors
  for (int i = 0; i < NUM_READINGS; i++){
    // Read LEFT sensor
    digitalWrite(trigPinL, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPinL, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPinL, LOW);
    
    float durationL = pulseIn(echoPinL, HIGH, 30000);
    
    if (durationL > 0) {
      readingsL[validReadingsL] = (durationL * 0.0343) / 2;
      validReadingsL++;
    }
    
    delay(60); // Wait between readings
    
    // Read RIGHT sensor
    digitalWrite(trigPinR, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPinR, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPinR, LOW);
    
    float durationR = pulseIn(echoPinR, HIGH, 30000);
    
    if (durationR > 0) {
      readingsR[validReadingsR] = (durationR * 0.0343) / 2;
      validReadingsR++;
    }
    
    delay(60); // Wait between readings
  }
  
  // Update global distance variables with MAD filtered averages
  if (validReadingsL >= 4) { // Need at least half the readings for reliable MAD
    distanceL = filtered_average_mad(readingsL);
  } else {
    distanceL = -1; // Error
  }
  
  if (validReadingsR >= 4) {
    distanceR = filtered_average_mad(readingsR);
  } else {
    distanceR = -1; // Error
  }
}

void setup() {
  pinMode(trigPinL, OUTPUT);
  pinMode(echoPinL, INPUT);
  pinMode(trigPinR, OUTPUT);
  pinMode(echoPinR, INPUT);
  Serial.begin(9600);
}

void loop() {
  // Single call updates both distanceL and distanceR
  ping();
  
  // Display results
  Serial.print("Left: ");
  if (distanceL >= 0) {
    Serial.print(distanceL);
    Serial.print(" cm");
  } else {
    Serial.print("Error");
  }
  
  Serial.print(" | Right: ");
  if (distanceR >= 0) {
    Serial.print(distanceR);
    Serial.println(" cm");
  } else {
    Serial.println("Error");
  }
  
  delay(100);
}