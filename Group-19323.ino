#include <DHT.h>

// Constants for PIR sensor
int pirPin = 2;               // Pin connected to PIR sensor
int ledPin = 12;              // Pin connected to LED (optional)
int peopleCount = 0;          // Variable to store the number of people
int motionState = LOW;        // Current PIR state
int previousMotionState = LOW;  // Previous PIR state

// Constants for DHT11 sensor
#define DHTPIN 4              // Pin connected to DHT11 data pin (updated)
#define DHTTYPE DHT11         // Define DHT11 sensor type
DHT dht(DHTPIN, DHTTYPE);     // Initialize DHT11 sensor

// Constants for MQ-135 sensor
int mq135Pin = A0;            // Pin connected to MQ-135 analog out
int buzzerPin = 11;           // Pin connected to buzzer
int co2Threshold = 500;       // CO2 threshold in ppm

unsigned long previousTempTime = 0;  // Timer for temperature readings
const long tempInterval = 5000;      // Temperature reading interval (5 seconds)

void setup() {
  // Initialize pins for PIR, LED, and buzzer
  pinMode(pirPin, INPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  
  // Begin serial communication
  Serial.begin(9600);
  dht.begin();  // Initialize DHT11 sensor
}

void loop() {
  // Get the current time
  unsigned long currentMillis = millis();

  // Read temperature every 5 seconds
  if (currentMillis - previousTempTime >= tempInterval) {
    // Update last temperature reading time
    previousTempTime = currentMillis;
    
    // Read temperature from DHT11
    float temperature = dht.readTemperature();
    
    // Check if temperature reading is valid
    if (isnan(temperature)) {
      Serial.println("Failed to read from DHT sensor!");
    } else {
      //Serial.print("Temperature: ");
      Serial.print(temperature);
      //Serial.println(" °C");
      Serial.println(",");
    }
  }

  // Read motion data from PIR sensor
  motionState = digitalRead(pirPin);

  // If motion is detected
  if (motionState == HIGH && previousMotionState == LOW) {
    // Increment people count
    peopleCount++;
    //Serial.print("Person entered. Count: ");
    Serial.print(peopleCount);

    // Turn on LED when motion is detected
    digitalWrite(ledPin, HIGH);

    // Delay to avoid multiple counts for the same person
    delay(500);
    
    previousMotionState = HIGH;  // Update previous motion state
  } else if (motionState == LOW && previousMotionState == HIGH) {
    // No motion detected, turn off LED
    digitalWrite(ledPin, LOW);
    previousMotionState = LOW;
  }

  // Read CO2 level from MQ-135 sensor
  int co2Level = analogRead(mq135Pin);
  
  // Map analog value to ppm (example range for demonstration purposes)
  int co2ppm = map(co2Level, 0, 1023, 0, 1000); 

  // Print CO2 level
  //Serial.print("CO2 Level: ");
  Serial.print(co2ppm);
  //Serial.println(" ppm");

  // Check if CO2 level exceeds threshold
  if (co2ppm > co2Threshold) {
    //Serial.println("Warning: CO2 level too high!");
    digitalWrite(buzzerPin, HIGH);  // Activate buzzer
  } else {
    digitalWrite(buzzerPin, LOW);   // Deactivate buzzer
  }

  delay(1000);  // Short delay for stability
}
