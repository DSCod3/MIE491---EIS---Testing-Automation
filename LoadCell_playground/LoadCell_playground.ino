/*
  LoadCell_playground.cpp

  This sketch demonstrates a basic PID control loop using a load cell
  on an Industrial Shields ESP32 PLC. The load cell outputs a voltage 
  between 0.5V and 4.5V. According to the load cell manufacturer, the 
  conversion factor is:
  
      1242.9920 lb/V
  
  This means:
      weight (lb) = voltage * 1242.9920
  
  The load cell is connected to analog input I0_7 and the PID controller's 
  output is sent to the analog output A0_5. The PID loop uses the measured 
  weight (in lb) as the process variable, and the setpoint is defined in lb.
  
  Debug information is printed via Serial.
*/

#include <PID_v1.h>

// --- Conversion Factor ---
#define LB_PER_VOLT 1242.9920

// --- Hardware Definitions ---
// Use I0_7 for the load cell analog input (0.5V to 4.5V range).
#define PIN_LOADCELL I0_7

// Use A0_5 for the analog output (PWM) to drive an actuator.
#define PIN_PWM_OUTPUT A0_5

// --- PID Control Variables ---
// Define the setpoint in pounds (for example, 3000 lb).
double setpoint = 3000.0;  
double input = 0;       // Process variable: measured weight in lb
double output = 0;      // PID computed output (0-255 for PWM)

// PID tuning parameters (adjust as needed).
double Kp = 2.0, Ki = 0.5, Kd = 1.0;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// --- Setup ---
void setup() {
  Serial.begin(115200);
  while (!Serial) {}  // Wait for Serial to initialize
  
  // Initialize the PID controller in AUTOMATIC mode.
  pid.SetMode(AUTOMATIC);
  // Set the PID output limits (0-255 for PWM).
  pid.SetOutputLimits(0, 255);
  
  Serial.println("LoadCell_playground PID test starting...");
}

// --- Loop ---
void loop() {
  // Read raw ADC value from the load cell on I0_7.
  int rawValue = analogRead(PIN_LOADCELL);
  
  // Convert raw ADC reading to voltage.
  // For an 11-bit ADC (0–2047) with a load cell output range of 0.5-4.5V:
  // voltage = (rawValue / 2047.0) * (4.5 - 0.5) + 0.5 = (rawValue/2047.0)*4.0 + 0.5
  double voltage = (rawValue / 2047.0) * 4.0 + 0.5;
  
  // Convert voltage to weight (lb) using the manufacturer's conversion factor.
  double weight = voltage * LB_PER_VOLT;
  
  // Use the measured weight as the process variable for the PID controller.
  input = weight;
  
  // Compute the PID output.
  pid.Compute();
  
  // Print diagnostic information.
  Serial.print("Setpoint: ");
  Serial.print(setpoint, 2);
  Serial.print(" lb, Measured: ");
  Serial.print(weight, 2);
  Serial.print(" lb, Voltage: ");
  Serial.print(voltage, 2);
  Serial.print(" V, PID Output: ");
  Serial.println(output, 2);
  
  // Output the PID result to the analog output A0_5 using PWM.
  analogWrite(PIN_PWM_OUTPUT, (int)output);
  
  // Delay before next iteration (adjust as needed).
  delay(1000);
}
