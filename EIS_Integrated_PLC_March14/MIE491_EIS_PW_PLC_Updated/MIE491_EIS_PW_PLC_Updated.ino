/*
  EIS PLC Code

  Description:
    This sketch runs on an Industrial Shields ESP32 PLC 38ARIO+ and integrates:
      - Sensor processing (string potentiometers, two load cells, pressure sensors)
      - Dynamic calibration of load cells so that their no-load reading is forced to 0.5 V
      - Conversion of load cell readings into force (lb) using manufacturer conversion factors
      - Conversion of force into pressure (psi) using a dummy effective area (10.0 in²)
      - Two independent PID controllers that drive two electronic pressure relief valves (EPRVs)
        via analog outputs (PIN_VALVE1 and PIN_VALVE2) so that the measured pressure converges
        to the desired setpoint.
      - Serial communication with a laptop GUI supporting multiple operating modes:
            SELECT_MODE, MANUAL_MODE, OPERATION_MODE, LOAD_CURVE_MODE, PAUSE_MODE, ERROR_MODE.
      - In MANUAL_MODE, commands like "SET_L:xxx" and "SET_R:xxx" (where xxx is in psi) are processed.
      - In OPERATION_MODE, the PLC sends datapoints in the format "time,forceLeft,forceRight" and
        verifies the echo from the computer.
      - In LOAD_CURVE_MODE, the PLC follows a predefined load curve (same for both valves).
      - Additional system variables and functions for reporting intervals, connection checking, etc.
      - The test automatically stops after 5 minutes (300,000 ms).
      - During shutdown (either on test completion or error), the PLC sets the EPRV setpoints to
        15 psi, waits until the measured pressures converge, switches the solenoid valves to position B,
        and then turns off the main air switch valve.
      
  Units:
    - Maximum pressure: 145 psi
    - Sample period: 100 ms
    - Effective maximum ramp rate: 1.0 psi/second
    - Test duration: 5 minutes (300,000 ms)
    - Effective area per load cell: 0.00155 in²  (Pressure = Force / effectiveArea)
    - Retraction pressure: 15 psi (minimum pressure needed to actuate 5×2 solenoids to position B)

  Note:
    The conversion from load cell voltage to weight is:
       weight (lb) = (corrected voltage - 0.5) * conversion factor
    where corrected voltage is the measured voltage minus an offset computed during calibration.
  
  Operating Modes (via Serial commands):
    Global mode strings and commands (from friend’s code) are defined below.
*/

#include <PID_v1.h>
#include <vector>

//-----------------------------------
// Global Mode Variables & Mode Strings
//-----------------------------------
String message;
enum OPERATING_MODE {
  SELECT_MODE,      
  MANUAL_MODE,      
  OPERATION_MODE,         
  ERROR_MODE
};
OPERATING_MODE MODE = SELECT_MODE;

#pragma region Mode Strings
String PROGRAM_EXIT = "PROGRAM_EXIT";
String MANUAL_MODE_ENTER = "M_ENTER";
String OPERATION_MODE_ENTER = "O_ENTER";
String MODE_EXIT = "MODE_EXIT";
String ERROR_MODE_ENTER = "E_ENTER";
String ERROR_MODE_EXIT = "ERR_CLEAR";
String CONNECTION_CHECK_REQUEST = "Connection check.";
String CONNECTION_CHECK_RESPONSE = "Connection OK.";
String EXCHANGE_DATAPOINTS_REQUEST = "EXCH_DP_REQ";
String EXCHANGE_DATAPOINTS_TERMINATION = "EXCH_DP_TERM";
String REPORTING_INTERVAL_EDIT = "R_INT_EDIT";
#pragma endregion

#pragma region Manual Mode Commands and Variables
String SET_LEFT = "SET_L";  // Set left piston pressure (psi)
String SET_RIGHT = "SET_R"; // Set right piston pressure (psi)
String RETRACT = "RETRACT"; // Set both solenoid valves to position B
String RETRACT_L = "RETRACT_L"; // Set Solenoid Left to position B
String RETRACT_R = "RETRACT_R"; // Set Solenoid Right to position B
String EXTEND = "EXTEND"; // Set both solenoid valves to position A
String EXTEND_L = "EXTEND_L"; // Set Solenoid Left to position A
String EXTEND_R = "EXTEND_R"; // Set Solenoid Right to position A
int separatorIndex;
String messageHeader;
float pressureValue;
#pragma endregion

#pragma region Operation Mode Commands and Variables
String OPERATION_START = "O_START";
String OPERATION_PAUSE = "O_PAUSE";
String OPERATION_RESUME = "O_RESUME";
String OPERATION_ESTOP = "O_ESTOP";
bool forceDatapointsExchanged = false;
#pragma endregion

#pragma region System Variables
unsigned int dataReportingInterval = 250; // in milliseconds
unsigned long testStartTime;
unsigned long modeStartTimestamp = 0;
unsigned long lastDataReportTimestamp = 0;
double manualValveCommand = 0; 
float pressureLeft;
float pressureRight;
float forceLeft;
float forceRight;
float distanceUp;
float distanceDown;
bool runTest = false;
bool testPaused = false;
#pragma endregion

#pragma region I/O
// (I/O definitions are in the hardware pin definitions below.)
#pragma endregion


#pragma region Supporting Functions
bool debugMode = true;

void dPrint(String msg){
  if(debugMode){
    Serial.print(msg);
  }
}

void dPrintline(String msg){
  if(debugMode){
    Serial.println(msg);
  }
}

void ClearIncomingBytes(){
  delay(1500);
  while(Serial.available()){
    Serial.read();
  }
}
#pragma endregion

//----------------------------------------------------------------
// End of Global Mode Initialization and Support Functions
//----------------------------------------------------------------


//----------------------------------------------------------------
// PLC Code: Sensor Processing, PID, Valve Control, etc.
//----------------------------------------------------------------

#pragma region Constants and Settings:
const double MAX_PRESSURE = 145.0;             // Maximum pressure in psi
const double MIN_PRESSURE = 0.0;             // Maximum pressure in psi
const unsigned long SAMPLE_PERIOD_MS = 100;      // Sample period in ms
const double EFFECTIVE_MAX_RAMP_RATE = 1.0;      // Effective max ramp rate in psi/s
const double effectiveArea = 0.00155;               // Dummy effective area in square inches
const double RETRACTION_PRESSURE = 12.0;         // Pressure reuuired for solenoid valve retraction

// Conversion Factors for Load Cells:
#define LB_PER_VOLT_1 1242.9920   // For load cell 1 (lb per V)
#define LB_PER_VOLT_2 1242.0049   // For load cell 2 (lb per V)

// Hardware Pin Definitions:
#define PIN_POT1       I0_7   // Potentiometer (display)
#define PIN_POT2       I0_8   // Potentiometer (display)
#define PIN_LOAD1      I0_9   // Load cell 1 input (0-10V)
#define PIN_LOAD2      I0_10  // Load cell 2 input (0-10V)
#define PIN_PRESSURE1  I1_4   // Pressure sensor for piston 1 (EPRV Signal)
#define PIN_PRESSURE2  I1_5  // Pressure sensor for piston 2 (EPRV Signal)
#define PIN_VALVE1     A0_5   // EPRV1 control (analog output)
#define PIN_VALVE2     A0_6   // EPRV2 control (analog output)
#define SOLENOID_VALVE1    R1_1 // Solenoid Valve 1 relay (ON = Position A, OFF = Position B)
#define SOLENOID_VALVE2    R1_2 // Solenoid Valve 2 relay (ON = Position A, OFF = Position B)
#define PIN_SWITCH_VALVE R1_3 // Switch Valve controlling main air flow

// PID Controller Setup for EPRVs:
// For EPRV1:
double eprv1_setpoint = 2;  // Default pressure setpoint in psi for EPRV1
double eprv1_input = 0;         // Measured pressure (psi) from load cell 1
double eprv1_output = 0;        // PID output (0-4095 scaled) for EPRV1
PID eprv1PID(&eprv1_input, &eprv1_output, &eprv1_setpoint, 2.0, 0.5, 1.0, DIRECT);

// For EPRV2:
double eprv2_setpoint = 2;  // Default pressure setpoint in psi for EPRV2
double eprv2_input = 0;         // Measured pressure (psi) from load cell 2
double eprv2_output = 0;        // PID output (0-4095 scaled) for EPRV2
PID eprv2PID(&eprv2_input, &eprv2_output, &eprv2_setpoint, 2.0, 0.5, 1.0, DIRECT);

// Global Sensor Variables for Load Cells:
int pot1, pot2;                // corrected poteniometer readings in mm
int rawLoad1, rawLoad2;        // Raw ADC values from load cells
double voltageLoad1, voltageLoad2;  // Measured voltages (0-10V)
double baseline1 = 0, baseline2 = 0, potBaseline1 = 0, potBaseline2 = 0;  // Offsets computed during calibration (in V)
double correctedVoltage1, correctedVoltage2; // Corrected voltages (ensuring effective no-load = 0.5V)
double weight1, weight2;       // Calculated force (lb) from each load cell
double avgWeight;              // Average force (lb)

// Global Variables for Load Curve:
double curvePoints[20];
std::vector<float> curvePointsLeft; // Target pressures (psi) for left piston
std::vector<float> curvePointsRight; // Target pressures (psi) for right piston
std::vector<float> curveTimes;  // Ramp times (ms) for each segment
int datapointIndex = 0;
int numCurvePoints = 0;
bool curveLoaded = false;
enum LoadCurveState { LC_RAMP, LC_DONE };
LoadCurveState lcState = LC_RAMP;
int currentCurveSegment = 0;
double previousSegmentTarget = 0;
unsigned long segmentStartTime = 0;
double loadCurveSetpoint = 0;  // Global load curve setpoint

// Forward Declarations:
double convertRawToVoltage(int rawValue);
void calibrateLoadCells();
void calibratePotentiometers();
void updateSensors();
void setValve1(double command);
void setValve2(double command);
void controlValves();
void sendForceDatapoints();
double computeMinimumTotalTime();
void updateLoadCurve();
void processSerialCommands();
void switchSolenoidsToB();
void switchSolenoidsToA();
void retractAndShutdown();
void resetSystem();
#pragma endregion

#pragma region Data/IO Functions
void updateData(){

//  For simulation purposes, use random values.
//  pressureLeft = (float)random(0, 150);
//  pressureRight = (float)random(0, 150);
//  forceLeft = (float)random(0, 5000);
//  forceRight = (float)random(0, 5000);
//  distanceUp = (float)random(0,500);
//  distanceDown = (float)random(0,500);

  // Uncomment the following lines when real sensors are connected:
  pressureLeft = convertPressure(analogRead(PIN_PRESSURE1));  // Use Conversion to map analog values to 0 - 145 psi range
  pressureRight = convertPressure(analogRead(PIN_PRESSURE2));
  forceLeft = convertRawToVoltage(analogRead(PIN_LOAD1)); // Use conversion function.
  forceRight = convertRawToVoltage(analogRead(PIN_LOAD2));
  pot1 = convertDistance(analogRead(PIN_POT1)); // User conversion to map analog values to 0 - 635 mm range
  pot2 = convertDistance(analogRead(PIN_POT2));
  // digitalWrite(SOLENOID_VALVE1, desiredState);
  // digitalWrite(SOLENOID_VALVE2, desiredState);

}

void sendData(){
  Serial.println("$");
  Serial.print(millis()-modeStartTimestamp);
  Serial.print(",");
  Serial.print(pressureLeft);
  Serial.print(",");
  Serial.print(pressureRight);
  Serial.print(",");
  Serial.print(forceLeft);
  Serial.print(",");
  Serial.print(forceRight);
  Serial.print(",");
  Serial.print(distanceUp);
  Serial.print(",");
  Serial.println(distanceDown);
}

void updateAndSendData(){
  updateData();
  sendData();
}
#pragma endregion


//-----------------------------------
// Helper Function: Convert raw ADC reading (0-2047) to voltage (0-10V)
//-----------------------------------
double convertRawToVoltage(int rawValue) {
  return ((rawValue+1) / 2047.0) * 10.0;
}

double convertPressure(int rawValue) {
  return ((rawValue+1) * 145.0) / 2048.0; 
}

double convertDistance(double rawValue) {
  return ((rawValue+1) * 635.0) / 2048.0;
}

//-----------------------------------
// Calibrate load cells by averaging no-load samples and compute offsets so that
// the effective no-load voltage becomes 0.5V.
//-----------------------------------
void calibrateLoadCells() {
  double sum1 = 0, sum2 = 0, calibrationSamples = 20;
  Serial.println("Calibrating load cells (no load)...");
  for (int i = 0; i < calibrationSamples; i++) {
    sum1 += convertRawToVoltage(analogRead(PIN_LOAD1));
    sum2 += convertRawToVoltage(analogRead(PIN_LOAD2));
    delay(50);
  }
  double measuredBaseline1 = sum1 / calibrationSamples;
  double measuredBaseline2 = sum2 / calibrationSamples;
  // Compute offsets so that corrected voltage = measuredVoltage - offset equals 0.5V at no load.
  baseline1 = measuredBaseline1 - 0.5;
  baseline2 = measuredBaseline2 - 0.5;
  Serial.print("Load Cell 1 measured baseline: "); Serial.print(measuredBaseline1, 3);
  Serial.print(" V, offset = "); Serial.print(baseline1, 3); Serial.println(" V");
  Serial.print("Load Cell 2 measured baseline: "); Serial.print(measuredBaseline2, 3);
  Serial.print(" V, offset = "); Serial.print(baseline2, 3); Serial.println(" V");
}

//-----------------------------------
// Calibrate string potentiometers by averaging unextended and computes offset needed
//-----------------------------------
void calibratePotentiometers() {
  int samples = 10;
  double sum1 = 0, sum2 = 0;
  Serial.println("Calibrating string potentiometers (no load)...");
  for (int i = 0; i < samples; i++) {
    sum1 += convertDistance(analogRead(PIN_POT1));
    sum2 += convertDistance(analogRead(PIN_POT2));
    delay(50);
  }
  potBaseline1 = sum1 / samples;
  potBaseline2 = sum2 / samples;
  Serial.print("Potentiometer 1 baseline: "); Serial.print(potBaseline1, 3); Serial.println(" mm");
  Serial.print("Potentiometer 2 baseline: "); Serial.print(potBaseline2, 3); Serial.println(" mm");
}

//-----------------------------------
// Update sensor readings: 
//  - Read raw ADC values from load cells and from the EPRV analog pressure inputs.
//  - Convert load cell readings to voltages and then to forces (lb).
//  - Convert the raw EPRV readings to pressure values (psi) using convertPressure().
//  - Assign the converted pressures to the global variables (pressureLeft and pressureRight)
//    so that these are passed on to the PC.
//-----------------------------------
void updateSensors() {
  // Read raw values for the load cells
  int rawVal1 = analogRead(PIN_LOAD1);
  int rawVal2 = analogRead(PIN_LOAD2);
  
  // Read raw values for the "pressure sensors" (actually the EPRV analog outputs)
  int rawPressureInput1 = analogRead(PIN_PRESSURE1);
  int rawPressureInput2 = analogRead(PIN_PRESSURE2);
  
  // Convert raw pressure readings (0–4097) to pressure in psi (0–145 psi)
  double convPressure1 = convertPressure(rawPressureInput1);
  double convPressure2 = convertPressure(rawPressureInput2);
  
  // Now update the global pressure variables that are sent to the PC.
  pressureLeft = convPressure1;
  pressureRight = convPressure2;

  if (convPressure1 > 145 || convPressure2 > 145) {
    dPrintline("Error: Pressure limit exceeded.");
    MODE = ERROR_MODE;
  }

  // Convert and calibrate string potentiometer values.
  double rawPot1 = analogRead(PIN_POT1);
  double rawPot2 = analogRead(PIN_POT2);
  double measuredDistance1 = convertDistance(rawPot1);
  double measuredDistance2 = convertDistance(rawPot2);
  distanceUp = measuredDistance1 - potBaseline1;
  distanceDown = measuredDistance2 - potBaseline2;
  if (distanceUp < 0) distanceUp = 0;
  if (distanceDown < 0) distanceDown = 0;
  
  // Convert load cell raw readings to voltage (0–10 V)
  voltageLoad1 = convertRawToVoltage(rawVal1);
  voltageLoad2 = convertRawToVoltage(rawVal2);
  
  // Compute corrected voltages (using calibration offsets)
  correctedVoltage1 = voltageLoad1 - baseline1;
  correctedVoltage2 = voltageLoad2 - baseline2;
  if (correctedVoltage1 < 0.5) correctedVoltage1 = 0.5;
  if (correctedVoltage2 < 0.5) correctedVoltage2 = 0.5;
  
  // Calculate force (lb) from each load cell:
  // weight (lb) = (correctedVoltage - 0.5) * conversion factor.
  weight1 = (correctedVoltage1 - 0.5) * LB_PER_VOLT_1;
  weight2 = (correctedVoltage2 - 0.5) * LB_PER_VOLT_2;
  avgWeight = (weight1 + weight2) / 2.0;
}

//-----------------------------------
// Set EPRV1 output.
void setValve1(double command) {
  int pwmValue = map(command, 0, (int)MAX_PRESSURE, 0, 4095);
  analogWrite(PIN_VALVE1, pwmValue);
}

//-----------------------------------
// Set EPRV2 output.
void setValve2(double command) {
  int pwmValue = map(command, 0, (int)MAX_PRESSURE, 0, 4095);
  analogWrite(PIN_VALVE2, pwmValue);
}

//-----------------------------------
// Switch the solenoid valves to position B (retraction position).
// With a single relay per valve, setting the relay output HIGH puts the valve into position B.
void switchSolenoidsToB() {
  digitalWrite(SOLENOID_VALVE1, HIGH);
  digitalWrite(SOLENOID_VALVE2, HIGH);
  dPrintline("Solenoid valves switched to position B.");
}

//-----------------------------------
// Switch the solenoid valves to position A (extension position).
// With a single relay per valve, setting the relay output LOW puts the valve into position A.
void switchSolenoidsToA() {
  digitalWrite(SOLENOID_VALVE1, LOW);
  digitalWrite(SOLENOID_VALVE2, LOW);
  dPrintline("Solenoid valves switched to position A.");
}

//-----------------------------------
// PID Control: Convert measured force to pressure (psi) and update valves.
// Pressure (psi) = Force (lb) / effectiveArea.
//-----------------------------------
void controlValves() {
  eprv1_input = weight1 / effectiveArea;
  eprv2_input = weight2 / effectiveArea;
  eprv1PID.Compute();
  eprv2PID.Compute();
  setValve1(eprv1_output);
  setValve2(eprv2_output);
}

//-----------------------------------
// Send force datapoints in the format "time,forceLeft,forceRight" and verify echo.
//-----------------------------------
void sendForceDatapoints() {
  unsigned long elapsedTime = millis() - modeStartTimestamp;
  String data = String(elapsedTime) + "," + String(weight1, 2) + "," + String(weight2, 2);
  Serial.println(data);
  unsigned long echoStart = millis();
  while (Serial.available() == 0 && (millis() - echoStart) < 2000) {
    delay(10);
  }
  if (Serial.available() > 0) {
    String echoMsg = Serial.readStringUntil('\n');
    if (echoMsg != data) {
      Serial.print("Error: Echo mismatch. Sent: ");
      Serial.print(data);
      Serial.print(" Received: ");
      Serial.println(echoMsg);
      
      // On echo mismatch, clear datapoint lists and enter error mode.
      curveTimes.clear();
      curvePointsLeft.clear();
      curvePointsRight.clear();
      MODE = ERROR_MODE;
    } else {
      Serial.print("Verified datapoint: ");
      Serial.println(data);
    }
  } else {
    Serial.println("No echo received for datapoint.");
  }
}

//-----------------------------------
// Compute minimum total time for a load curve based on effective ramp rate.
//-----------------------------------
double computeMinimumTotalTime() {
  double total = 0;
  double prev = 0;
  for (int i = 0; i < numCurvePoints; i++) {
    double delta = fabs(curvePoints[i] - prev);
    double segTime = (delta / EFFECTIVE_MAX_RAMP_RATE) * 1000;
    total += segTime;
    prev = curvePoints[i];
  }
  total += (fabs(prev) / EFFECTIVE_MAX_RAMP_RATE) * 1000;
  return total;
}

//-----------------------------------
// Update the load curve during LOAD_CURVE_MODE.
// Interpolate between curve segments then ramp down.
//-----------------------------------
void updateLoadCurve() {
  if (!curveLoaded) {
    Serial.println("LOAD_CURVE_MODE -> No load curve loaded.");
    return;
  }
  
  unsigned long now = millis();
  if (currentCurveSegment < numCurvePoints) {
    unsigned long segTime = curveTimes[currentCurveSegment];
    unsigned long elapsed = now - segmentStartTime;
    double target = curvePoints[currentCurveSegment];
    if (elapsed < segTime) {
      loadCurveSetpoint = previousSegmentTarget + (target - previousSegmentTarget) * (elapsed / (double)segTime);
    } else {
      loadCurveSetpoint = target;
      previousSegmentTarget = target;
      currentCurveSegment++;
      segmentStartTime = now;
    }
  } else {
    double delta = previousSegmentTarget;
    unsigned long rampDownTime = (unsigned long)((delta / EFFECTIVE_MAX_RAMP_RATE) * 1000);
    unsigned long elapsed = now - segmentStartTime;
    if (elapsed < rampDownTime) {
      loadCurveSetpoint = previousSegmentTarget * (1.0 - (elapsed / (double)rampDownTime));
    } else {
      loadCurveSetpoint = 0;
      Serial.println("Load curve execution complete.");
      curveLoaded = false;
    }
  }
  
  // Run the PID controllers using current sensor readings.
  controlValves();
  
  Serial.print("LOAD_CURVE_MODE -> Load Curve Setpoint: ");
  Serial.print(loadCurveSetpoint);
  Serial.print(" | EPRV1 Pressure: ");
  Serial.print(eprv1_input);
  Serial.print(" psi, PID Output1: ");
  Serial.print(eprv1_output);
  Serial.print(" | EPRV2 Pressure: ");
  Serial.print(eprv2_input);
  Serial.print(" psi, PID Output2: ");
  Serial.println(eprv2_output);
}

//-----------------------------------
// Receive incoming force datapoints from PC
//-----------------------------------
void receiveForceDatapoints(){
  // Clear the previous curve data if any
  curveTimes.clear();
  curvePointsLeft.clear();
  curvePointsRight.clear();

  String message;
  float receivedTime;
  float receivedForceLeft;
  float receivedForceRight;
  int commaIndex;
  int start;
  while(true){
    if (Serial.available() > 0) {
      message = Serial.readStringUntil('\n');
      message.trim();

      if(message == EXCHANGE_DATAPOINTS_TERMINATION){ break; }

      start = 0;
      commaIndex = message.indexOf(",");
      receivedTime = message.substring(start, commaIndex).toFloat();
      curveTimes.push_back(receivedTime);
      start = commaIndex + 1;
      Serial.print(String(receivedTime,2));
      Serial.print(",");

      commaIndex = message.indexOf(",", start);
      receivedForceLeft = message.substring(start, commaIndex).toFloat();
      curvePointsLeft.push_back(receivedForceLeft);
      start = commaIndex + 1;
      Serial.print(String(receivedForceLeft,2));
      Serial.print(",");

      commaIndex = message.indexOf(",", start);
      receivedForceRight = message.substring(start, commaIndex).toFloat();
      curvePointsRight.push_back(receivedForceRight);
      Serial.println(String(receivedForceRight,2));
    }
  }

  curveLoaded = true;
  printForceDatapoints();
  
}

void printForceDatapoints(){
  Serial.println("Printing out force datapoints...");
  Serial.println("Time (ms), Left Force (lb), Right Force (lb)");
  for(int i = 0; i < curveTimes.size(); i++){
    Serial.print(curveTimes[i]);
    Serial.print(",");
    Serial.print(curvePointsLeft[i]);
    Serial.print(",");
    Serial.println(curvePointsRight[i]);
  }

  Serial.println("...end of force datapoints printout.");
}

//-----------------------------------
// Reset System Function to be called on MODE_EXIT
//-----------------------------------
void resetSystem() {
  dPrintline("Resetting system to startup state...");
  // Retract pistons:
  switchSolenoidsToB();
  // Set EPRV setpoints to 0 psi.
  eprv1_setpoint = MIN_PRESSURE;
  eprv2_setpoint = MIN_PRESSURE;
  // Wait for measured pressures to converge near 0 psi.
  while (fabs(eprv1_input) > 0.5 || fabs(eprv2_input) > 0.5) {
    controlValves();
    delay(100);
  }
  // Shut off main air supply.
  digitalWrite(PIN_SWITCH_VALVE, LOW);
  dPrintline("System reset complete.");
}

//-----------------------------------
// Retract and Shutdown Sequence:
// 1. Set EPRV setpoints to 15 psi (RETRACTION_PRESSURE).
// 2. Wait until measured pressures converge within ±1 psi.
// 3. Switch solenoid valves to position B (by turning their relay outputs OFF).
// 4. Turn off the main air supply by turning off the switch valve.
//-----------------------------------
void retractAndShutdown() {
  // Set both EPRV setpoints to the retraction pressure (15 psi)
  eprv1_setpoint = RETRACTION_PRESSURE;
  eprv2_setpoint = RETRACTION_PRESSURE;
  
  // Wait until measured pressures converge near 15 psi (within ±1 psi)
  while ((fabs(eprv1_input - RETRACTION_PRESSURE) > 1.0) || (fabs(eprv2_input - RETRACTION_PRESSURE) > 1.0)) {
    controlValves();
    delay(100);
  }
  
  // Once converged, switch the solenoids to position B
  switchSolenoidsToB();
  
  // Finally, shut off the compressor by turning off the switch valve.
  digitalWrite(PIN_SWITCH_VALVE, LOW);
  dPrintline("System shut down (switch valve OFF).");
}

//-----------------------------------
// Main Setup
//-----------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  // Keep the system switch valve OFF at startup.
  pinMode(PIN_SWITCH_VALVE, OUTPUT);
  digitalWrite(PIN_SWITCH_VALVE, LOW);

  switchSolenoidsToA();
  
  Serial.println("PLC System Initializing...");
  
  // Configure sensor pins.
  pinMode(PIN_POT1, INPUT);
  pinMode(PIN_POT2, INPUT);
  pinMode(PIN_LOAD1, INPUT);
  pinMode(PIN_LOAD2, INPUT);
  pinMode(PIN_PRESSURE1, INPUT);
  pinMode(PIN_PRESSURE2, INPUT);
  
  // Configure outputs.
  pinMode(PIN_VALVE1, OUTPUT);
  pinMode(PIN_VALVE2, OUTPUT);
  pinMode(SOLENOID_VALVE1, OUTPUT);
  pinMode(SOLENOID_VALVE2, OUTPUT);

  
  calibrateLoadCells();
  calibratePotentiometers();
  
  // Initialize PID controllers.
  eprv1PID.SetMode(AUTOMATIC);
  eprv1PID.SetOutputLimits(0, 255);
  eprv2PID.SetMode(AUTOMATIC);
  eprv2PID.SetOutputLimits(0, 255);
  
  MODE = SELECT_MODE;
  testStartTime = millis();
  modeStartTimestamp = millis();
  lastDataReportTimestamp = millis();
  ClearIncomingBytes();
  Serial.println("PLC System Initialized. In SELECT mode.");
}

//-----------------------------------
// Main Loop
//-----------------------------------
void loop(){
    switch(MODE){
  
    #pragma region SELECT
    case SELECT_MODE:

      dPrint("Entered selection mode.");
      // When returning to SELECT_MODE, reset the system.
      
      if(Serial.available()){
        message = Serial.readStringUntil('\n');
        
        if(message == MODE_EXIT){
          resetSystem();
        }
        else if(message == CONNECTION_CHECK_REQUEST){
          Serial.println(CONNECTION_CHECK_RESPONSE);
        }

        else if(message == MANUAL_MODE_ENTER){
          MODE = MANUAL_MODE;
        }

        else if(message == OPERATION_MODE_ENTER){
          MODE = OPERATION_MODE;
        }

        else if(message == ERROR_MODE_ENTER){
          MODE = ERROR_MODE;
        }

        else if(message == REPORTING_INTERVAL_EDIT){
          while(!Serial.available());
          dataReportingInterval = Serial.readStringUntil('\n').toInt();
          dPrint("Reporting interval set to ");
          dPrint(String(dataReportingInterval));
          dPrintline(" ms.");
        }

        else{
          dPrintline(message);
        }
      }
      else{ delay(100); }
      
      break;
    #pragma endregion
    
    #pragma region MANUAL
    case MANUAL_MODE:
    
      dPrintline("Entered manual mode.");
      modeStartTimestamp = millis();
      digitalWrite(PIN_SWITCH_VALVE, HIGH);
      
      while(true){
        // Read message from the computer if there are bytes in the incoming buffer  
        if(Serial.available()){
          message = Serial.readStringUntil('\n');
          if(message == MODE_EXIT){
            MODE = SELECT_MODE;
            resetSystem();
            break;
          }

          else if(message == RETRACT){
            switchSolenoidsToB();
            dPrint("Solenoid valves switched to retraction of pistons");
          }

          else if(message == EXTEND){
            switchSolenoidsToA();
            dPrint("Solenoid valves switched to extension of pistons");
          }

          else if (message == RETRACT_L){
            digitalWrite(SOLENOID_VALVE1, HIGH);
          }

          else if (message == RETRACT_R){
            digitalWrite(SOLENOID_VALVE2, HIGH);
          }

          else if (message == EXTEND_L){
            digitalWrite(SOLENOID_VALVE1, LOW);
          }
          else if (message == EXTEND_R){
            digitalWrite(SOLENOID_VALVE2, LOW);
          }
          
          // Split the string into header and value
          separatorIndex = message.indexOf(':');
          if(separatorIndex != -1){
            messageHeader = message.substring(0, separatorIndex);
            pressureValue = message.substring(separatorIndex+1).toFloat();
          }
          else{
            dPrintline(message);
            continue;
          }

          if(messageHeader == SET_LEFT){
            setValve1(pressureValue);
            dPrint("Left piston pressure set to ");
            dPrint(String(pressureValue));
            dPrintline(" psi.");
          }

          else if(messageHeader == SET_RIGHT){
            setValve2(pressureValue);
            dPrint("Right piston pressure set to ");
            dPrint(String(pressureValue));
            dPrintline(" psi.");
          }

          else{
            dPrintline(message);
            continue;
          }
        }

        // Report Data
        if((millis() - lastDataReportTimestamp) > dataReportingInterval){
          updateAndSendData();
          lastDataReportTimestamp = millis();
        }
        
      }
      break;
    #pragma endregion

    #pragma region OPERATION
    case OPERATION_MODE:
      modeStartTimestamp = millis(); 
      dPrintline("Entered operation mode."); 
      digitalWrite(PIN_SWITCH_VALVE, HIGH); 
      
      // While loop will wait for exchange for force datapoints from PC and the start commands
      while(true){
        // Read message from the computer if there are bytes in the incoming buffer  
        if(Serial.available()){
          message = Serial.readStringUntil('\n');
          message.trim();
          if(message == MODE_EXIT){
            resetSystem();
            MODE = SELECT_MODE;
            break;
          }

          else if (message == EXCHANGE_DATAPOINTS_REQUEST){
            receiveForceDatapoints();
            // Reset datapoint index after receiving new datapoints
            datapointIndex = 0;
          }

          else if(message == OPERATION_START){
            runTest = true;
            dPrintline("Test starting...");
            break;
          }
        }
        delay(10);
      }
        // Run the test loop
      while(runTest){

        // Handling communications
        if(Serial.available()){
          message = Serial.readStringUntil('\n');
          message.trim();
          if(message == MODE_EXIT && debugMode){
            MODE = SELECT_MODE;
            break;
          }
          else if(message == OPERATION_ESTOP){
            // Go immediately to error mode
            runTest = false;
            MODE = ERROR_MODE;
            break;
          }

          else if(message == OPERATION_PAUSE){
            testPaused = true;
            // Code to pause operation
          }

          else if(message == OPERATION_RESUME){
            testPaused = false;
            // Code to resume operation
          }
        }

        // Code for constant data reporting
        if((millis() - lastDataReportTimestamp) > dataReportingInterval && !testPaused){
          updateAndSendData();
          lastDataReportTimestamp = millis();
        }

        if(!testPaused){
          // Process datapoints if available.
          if(curveTimes.size() > 0 && datapointIndex < curveTimes.size()){
            if(millis() - modeStartTimestamp >= curveTimes[datapointIndex]){
              // Convert received force (lb) to pressure (psi) via effective area.
              // (Assuming the force datapoints are provided in lb.)
              eprv1_setpoint = curvePointsLeft[datapointIndex] / effectiveArea;
              eprv2_setpoint = curvePointsRight[datapointIndex] / effectiveArea;
              datapointIndex++;
            }
          }

          // Run the control loop (PID controllers update the valves ased on measured values)
          controlValves();

          //Send current datapoints to the computer for echo verification
          sendForceDatapoints();
        }

        // Data reporting delay
        delay(dataReportingInterval);
      }

      // After test, if datapoints are all processed, send termination keyword.
      if(curveTimes.size() > 0 && datapointIndex >= curveTimes.size()){
        Serial.println("TEST_COMPLETE");
      }
      
      // Now perform the retraction sequence:
      // 1. Set EPRV setpoints to 15 psi.
      eprv1_setpoint = RETRACTION_PRESSURE;
      eprv2_setpoint = RETRACTION_PRESSURE;
      // 2. Wait for convergence (within ±1 psi)
      while ((fabs(eprv1_input - RETRACTION_PRESSURE) > 1.0) ||
             (fabs(eprv2_input - RETRACTION_PRESSURE) > 1.0)) {
        controlValves();
        delay(100);
      }
      // 3. Switch the 5x2 solenoid valves to position B.
      switchSolenoidsToB();
      // 4. Turn off the switch valve (cut off compressor air).
      digitalWrite(PIN_SWITCH_VALVE, LOW);
      
      runTest = false;
      break;
    #pragma endregion
    
    #pragma region ERROR
    case ERROR_MODE:
      dPrintline("Entered error mode.");

      // In error mode, perform a safe shutdown sequence.
      // Set EPRV setpoints to 15 psi to retract the pistons.
      eprv1_setpoint = RETRACTION_PRESSURE;
      eprv2_setpoint = RETRACTION_PRESSURE;
      while ((fabs(eprv1_input - RETRACTION_PRESSURE) > 1.0) ||
             (fabs(eprv2_input - RETRACTION_PRESSURE) > 1.0)) {
        controlValves();
        delay(100);
      }
      // Switch solenoids to position B.
      switchSolenoidsToB();

      // Set both EPRVs to 0 psi to release all pressure in system
      setValve1(MIN_PRESSURE);
      setValve2(MIN_PRESSURE);
      
      // Turn off the switch valve.
      digitalWrite(PIN_SWITCH_VALVE, LOW);
      // Wait until error cleared.
      
      while(true){
        // Read message from the computer if there are bytes in the incoming buffer  
        if(Serial.available()){
          message = Serial.readStringUntil('\n');
          if(message == ERROR_MODE_EXIT){
            resetSystem();
            MODE = SELECT_MODE;
            break;
          }
        }

        else{
            continue;
        }
      }
      break;
    #pragma endregion

    default:
      dPrintline("NO CASE");
      break;

  }
}
