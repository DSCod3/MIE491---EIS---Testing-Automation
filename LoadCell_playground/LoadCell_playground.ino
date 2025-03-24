/*
  LoadCell_playground.cpp

  Basic PID control loop using two load cells
  on an Industrial Shields ESP32 PLC. Each load cell outputs a voltage 
  between 0.5V and 4.5V (but may have a higher no-load baseline due to 
  inherent offsets in wiring/signal conditioning). The ADC has a 0–10V range (0–2047 counts).

  The weight for each load cell is calculated as:
      weight (lb) = (correctedVoltage - 0.5) * (conversion factor)
  
  For load cell 1 (on I0_7): conversion factor = 1242.9920 lb/V
  For load cell 2 (on I0_8): conversion factor = 1242.0049 lb/V
  
  Dynamic calibration is performed at startup. After calibration, an offset is computed so that
  the effective (corrected) voltage will be 0.5V at no load.
  
  The process variable for the PID loop is the average of both load cell weights.
  Debug information is printed via Serial.
*/

#include <PID_v1.h>

// --- Conversion Factors ---
#define LB_PER_VOLT_1 1242.9920
#define LB_PER_VOLT_2 1242.0049

// --- Hardware Definitions ---
#define PIN_LOADCELL1 I0_7    // Load cell 1 input on I0_7 (0–10V)
#define PIN_LOADCELL2 I0_8    // Load cell 2 input on I0_8 (0–10V)
#define PIN_PWM_OUTPUT A0_5   // Analog output (PWM) on A0_5

// --- PID Control Variables ---
double setpoint = 30.0;  // Desired weight in lb
double input = 0;        // Process variable: average measured weight in lb
double output = 0;       // PID computed output (0-255 for PWM)

// PID tuning parameters (adjust as needed)
double Kp = 2.0, Ki = 0.5, Kd = 1.0;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// --- Timing Variables ---
unsigned long startTime = 0;               // To mark when the test started
const unsigned long TEST_DURATION = 120000; // Test duration: 2 minutes in ms

// --- Calibration Variables ---
double baseline1 = 0;  // Calibrated baseline for load cell 1 (in Volts)
double baseline2 = 0;  // Calibrated baseline for load cell 2 (in Volts)
double offset1 = 0;    // offset1 = baseline1 - 0.5 (to force no-load effective voltage = 0.5V)
double offset2 = 0;    // offset2 = baseline2 - 0.5
const int calibrationSamples = 10;  // Number of samples for calibration

// --- Helper Function ---
// Convert raw ADC reading to voltage (0–10V)
double convertRawToVoltage(int rawValue) {
  return (rawValue / 2047.0) * 10.0;
}

// --- Setup ---
void setup() {
  Serial.begin(115200);
  while (!Serial) {}  // Wait for Serial to initialize

  // Perform dynamic calibration for both load cells.
  double sum1 = 0, sum2 = 0;
  Serial.println("Calibrating load cells (no load)...");
  for (int i = 0; i < calibrationSamples; i++) {
    int raw1 = analogRead(PIN_LOADCELL1);
    int raw2 = analogRead(PIN_LOADCELL2);
    sum1 += convertRawToVoltage(raw1);
    sum2 += convertRawToVoltage(raw2);
    delay(100);
  }
  baseline1 = sum1 / calibrationSamples;
  baseline2 = sum2 / calibrationSamples;
  // Compute offsets such that no-load corrected voltage becomes 0.5V.
  offset1 = baseline1 - 0.5;
  offset2 = baseline2 - 0.5;
  Serial.print("Load Cell 1 Baseline: "); Serial.print(baseline1, 3); Serial.print(" V, Offset: "); Serial.print(offset1,3); Serial.println(" V");
  Serial.print("Load Cell 2 Baseline: "); Serial.print(baseline2, 3); Serial.print(" V, Offset: "); Serial.print(offset2,3); Serial.println(" V");

  // Mark the start time for the test.
  startTime = millis();
  
  // Initialize the PID controller in AUTOMATIC mode and set output limits.
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(0, 255);
  
  Serial.println("LoadCell_playground PID test starting...");
}

// --- Loop ---
void loop() {
  // End test after 2 minutes.
  if (millis() - startTime >= TEST_DURATION) {
    Serial.println("Test duration complete. Halting execution.");
    analogWrite(PIN_PWM_OUTPUT, 0);  // Optionally, set output to 0
    while (true) {
      delay(1000);
    }
  }
  
  // --- Load Cell 1 ---
  int rawValue1 = analogRead(PIN_LOADCELL1);
  double voltage1 = convertRawToVoltage(rawValue1);
  // Correct the voltage by subtracting the computed offset.
  double correctedVoltage1 = voltage1 - offset1;
  // Ensure that corrected voltage is at least 0.5V.
  if (correctedVoltage1 < 0.5) correctedVoltage1 = 0.5;
  double weight1 = (correctedVoltage1 - 0.5) * LB_PER_VOLT_1;
  
  // --- Load Cell 2 ---
  int rawValue2 = analogRead(PIN_LOADCELL2);
  double voltage2 = convertRawToVoltage(rawValue2);
  double correctedVoltage2 = voltage2 - offset2;
  if (correctedVoltage2 < 0.5) correctedVoltage2 = 0.5;
  double weight2 = (correctedVoltage2 - 0.5) * LB_PER_VOLT_2;
  
  // --- Compute Average Weight ---
  double avgWeight = (weight1 + weight2) / 2.0;
  
  // Use the average weight as the process variable for the PID controller.
  input = avgWeight;
  
  // Compute the PID output.
  pid.Compute();
  
  // Print diagnostic information.
  Serial.print("Setpoint: ");
  Serial.print(setpoint, 2);
  Serial.print(" lb, Weight1: ");
  Serial.print(weight1, 2);
  Serial.print(" lb, Weight2: ");
  Serial.print(weight2, 2);
  Serial.print(" lb, Avg Weight: ");
  Serial.print(avgWeight, 2);
  Serial.print(" lb, CorrVoltage1: ");
  Serial.print(correctedVoltage1, 3);
  Serial.print(" V, CorrVoltage2: ");
  Serial.print(correctedVoltage2, 3);
  Serial.print(" V, PID Output: ");
  Serial.println(output, 2);
  
  // Output the PID result to the analog output A0_5 using PWM.
  analogWrite(PIN_PWM_OUTPUT, (int)output);
  
  // Delay before next iteration.
  delay(10);
}
