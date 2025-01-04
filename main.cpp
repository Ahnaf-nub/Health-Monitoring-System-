#include <Wire.h>
#include <MAX30105.h>
#include <TFT_eSPI.h>          

MAX30105 particleSensor;
TFT_eSPI tft = TFT_eSPI(); 

const int numReadings = 10;
const long fingerThreshold = 50000; 

float bpmReadings[numReadings];
float spo2Readings[numReadings];
int readingIndex = 0;

// Kalman Filter Variables
float bpmEstimate = 0.0, bpmError = 1.0, bpmQ = 0.005, bpmR = 0.1;
float spo2Estimate = 0.0, spo2Error = 1.0, spo2Q = 0.05, spo2R = 0.25;

// Median Filter Function
float calculateMedian(float readings[], int size) {
  float sortedReadings[size];
  memcpy(sortedReadings, readings, size * sizeof(float));  // Copy readings to avoid modifying the original array
  for (int i = 0; i < size - 1; i++) {
    for (int j = i + 1; j < size; j++) {
      if (sortedReadings[i] > sortedReadings[j]) {
        float temp = sortedReadings[i];
        sortedReadings[i] = sortedReadings[j];
        sortedReadings[j] = temp;
      }
    }
  }
  return sortedReadings[size / 2];  // Return the middle value
}

// Kalman Filter Implementation
float kalmanFilter(float measurement, float &estimate, float &error, float Q, float R) {
  error += Q;
  float kalmanGain = error / (error + R);
  estimate = estimate + kalmanGain * (measurement - estimate);
  error = (1 - kalmanGain) * error;
  return estimate;
}

float calculateHeartRate(long red, long ir) {
  static unsigned long lastBeatTime = 0;
  static int beatCount = 0;
  static float beatsPerMinute = 0.0;

  if (ir > fingerThreshold && (millis() - lastBeatTime > 300)) {
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - lastBeatTime) / 1000.0;
    if (deltaTime > 0.3 && deltaTime < 2.0) { // Ensures BPM is within a valid range
      beatsPerMinute = 60.0 / deltaTime;
    }
    lastBeatTime = currentTime;
    beatCount++;
  }

  return beatsPerMinute;
}

float calculateSpO2(long red, long ir) {
  if (ir == 0) return 0;

  float ratio = (float)red / (float)ir;
  float spo2 = 110.0 - 25.0 * ratio;

  if (spo2 > 100.0) spo2 = 100.0;
  if (spo2 < 70.0) spo2 = 70.0;

  return spo2;
}

void setup() {
  Serial.begin(9600);
  Wire.begin(21, 22);
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);

  if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
    Serial.println("MAX30102 was not found. Please check wiring/power.");
    while (1);
  }

  particleSensor.setup();

  // Initialize the readings arrays to zero
  for (int i = 0; i < numReadings; i++) {
    bpmReadings[i] = 0.0;
    spo2Readings[i] = 0.0;
  }
}

void loop() {
  long redValue = particleSensor.getRed();
  long irValue = particleSensor.getIR();
  float temperature = particleSensor.readTemperature();

  if (irValue > fingerThreshold) {
    // Calculate heart rate and SpO2
    float bpm = calculateHeartRate(redValue, irValue);
    float spo2 = calculateSpO2(redValue, irValue);

    // Apply Kalman filter for smoothing
    float filteredBPM = kalmanFilter(bpm, bpmEstimate, bpmError, bpmQ, bpmR);
    float filteredSpO2 = kalmanFilter(spo2, spo2Estimate, spo2Error, spo2Q, spo2R);

    // Store the filtered values in the arrays
    bpmReadings[readingIndex] = filteredBPM;
    spo2Readings[readingIndex] = filteredSpO2;

    // Update the reading index (circular buffer)
    readingIndex = (readingIndex + 1) % numReadings;

    // Get the median of the readings
    float medianBPM = calculateMedian(bpmReadings, numReadings);
    float medianSpO2 = calculateMedian(spo2Readings, numReadings);

    // Display results on the TFT screen
    tft.fillScreen(TFT_BLACK);
    
    tft.setCursor(10, 20);
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.printf("BPM: %.2f", medianBPM);

    tft.setCursor(10, 60);
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.printf("SpO2: %.2f%%", medianSpO2);

    tft.setCursor(10, 100);
    tft.setTextColor(TFT_BLUE, TFT_BLACK);
    tft.printf("Temp: %.1f *C", temperature);

    tft.setTextColor(TFT_WHITE, TFT_BLACK); // Reset to default color

    Serial.printf("BPM: %.2f, SpO2: %.2f%%, Temp: %.2f C\n", medianBPM, medianSpO2, temperature);
  } else {
    tft.fillScreen(TFT_BLACK);
    tft.setCursor(10, 20);
    tft.printf("No finger detected");
  }

  delay(500); // Reduced delay for smoother updates
}