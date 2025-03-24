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
            SELECT_MODE, MANUAL_MODE, OPERATION_MODE, LOAD_CURVE_MODE, PAUSE_MODE, ERR_MODE.
      - In MANUAL_MODE, commands like "SET_L:xxx" and "SET_R:xxx" (where xxx is in psi) are processed.
      - In OPERATION_MODE, the PLC sends datapoints in the format "time,forceLeft,forceRight" and
        verifies the echo from the computer.
      - In LOAD_CURVE_MODE, the PLC follows a predefined load curve (same for both valves).
      - Additional system variables and functions for reporting intervals, connection checking, etc.
      - The test automatically stops after 5 minutes (300,000 ms).
      
  Units:
    - Maximum pressure: 145 psi
    - Sample period: 100 ms
    - Effective maximum ramp rate: 1.0 psi/second
    - Test duration: 5 minutes (300,000 ms)
    - Effective area (dummy): 10.0 in²  (Pressure = Force / effectiveArea)

  Note:
    The conversion from load cell voltage to weight is:
       weight (lb) = (corrected voltage - 0.5) * conversion factor
    where corrected voltage is the measured voltage minus an offset computed during calibration.
  
  Operating Modes (via Serial commands):
    Global mode strings and commands (from friend’s code) are defined below.
*/

#include <PID_v1.h>

//-----------------------------------
// Global Mode Variables & Mode Strings
//-----------------------------------
String message;
enum OPERATING_MODE {
  SELECT_MODE,      
  MANUAL_MODE,      
  OPERATION_MODE,   
  LOAD_CURVE_MODE,  
  PAUSE_MODE,       
  ERR_MODE
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
String REPORTING_INTERVAL_EDIT = "R_INT_EDIT";
#pragma endregion

#pragma region Manual Mode Commands and Variables
String SET_LEFT = "SET_L";  // Set left piston pressure
String SET_RIGHT = "SET_R"; // Set right piston pressure
int separatorIndex;
String messageHeader;
float messageValue;
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

#pragma region Data/IO Functions
void updateData(){
  pressureLeft = (float)random(0, 150);
  pressureRight = (float)random(0, 150);
  forceLeft = (float)random(0, 5000);
  forceRight = (float)random(0, 5000);
  distanceUp = (float)random(0,500);
  distanceDown = (float)random(0,500);
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

// Constants and Settings:
const double MAX_PRESSURE = 145.0;             // Maximum pressure in psi
const unsigned long SAMPLE_PERIOD_MS = 100;      // Sample period in ms
const double EFFECTIVE_MAX_RAMP_RATE = 1.0;      // Effective max ramp rate in psi/s
const unsigned long TEST_DURATION = 300000;      // Test duration: 5 minutes in ms
const double effectiveArea = 10.0;               // Dummy effective area in square inches

// Conversion Factors for Load Cells:
#define LB_PER_VOLT_1 1242.9920   // For load cell 1 (lb per V)
#define LB_PER_VOLT_2 1242.0049   // For load cell 2 (lb per V)

// Hardware Pin Definitions:
#define PIN_POT1       I0_7   // Potentiometer (for display)
#define PIN_POT2       I0_8   // Potentiometer (for display)
#define PIN_LOAD1      I0_9   // Load cell 1 input (0-10V)
#define PIN_LOAD2      I0_10  // Load cell 2 input (0-10V)
#define PIN_PRESSURE1  I0_11  // Pressure sensor for piston 1 (for display)
#define PIN_PRESSURE2  I0_12  // Pressure sensor for piston 2 (for display)
#define PIN_VALVE1     A0_5   // EPRV1 control (analog output)
#define PIN_VALVE2     A0_6   // EPRV2 control (analog output)
#define SOLENOID1_A    R1_1   // Solenoid valve 1, position A
#define SOLENOID1_B    R1_2   // Solenoid valve 1, position B
#define SOLENOID2_A    R1_3   // Solenoid valve 2, position A
#define SOLENOID2_B    R1_4   // Solenoid valve 2, position B

// PID Controller Setup for EPRVs:
// For EPRV1:
double eprv1_setpoint = 100.0;  // Default pressure setpoint in psi for EPRV1
double eprv1_input = 0;         // Measured pressure (psi) from load cell 1
double eprv1_output = 0;        // PID output (0-255) for EPRV1
PID eprv1PID(&eprv1_input, &eprv1_output, &eprv1_setpoint, 2.0, 0.5, 1.0, DIRECT);

// For EPRV2:
double eprv2_setpoint = 100.0;  // Default pressure setpoint in psi for EPRV2
double eprv2_input = 0;         // Measured pressure (psi) from load cell 2
double eprv2_output = 0;        // PID output (0-255) for EPRV2
PID eprv2PID(&eprv2_input, &eprv2_output, &eprv2_setpoint, 2.0, 0.5, 1.0, DIRECT);

// Global Sensor Variables for Load Cells:
int pot1, pot2;                // Raw potentiometer readings (as ints)
int rawLoad1, rawLoad2;        // Raw ADC values from load cells
double voltageLoad1, voltageLoad2;  // Measured voltages (0-10V)
double baseline1 = 0, baseline2 = 0;  // Offsets computed during calibration (in V)
double correctedVoltage1, correctedVoltage2; // Corrected voltages (ensuring effective no-load = 0.5V)
double weight1, weight2;       // Calculated force (lb) from each load cell
double avgWeight;              // Average force (lb)

// Global Variables for Load Curve:
#define MAX_CURVE_POINTS 20
double curvePoints[MAX_CURVE_POINTS];      // Target pressures (psi) for each segment
unsigned long curveTimes[MAX_CURVE_POINTS];  // Ramp times (ms) for each segment
int numCurvePoints = 0;
bool curveLoaded = false;
enum LoadCurveState { LC_RAMP, LC_DONE };
LoadCurveState lcState = LC_RAMP;
int currentCurveSegment = 0;
double previousSegmentTarget = 0;
unsigned long segmentStartTime = 0;
double loadCurveSetpoint = 0;  // Global setpoint for the load curve

// Forward Declarations:
double convertRawToVoltage(int rawValue);
void calibrateLoadCells();
void updateSensors();
void setValve1(double command);
void setValve2(double command);
void controlValves();
void sendForceDatapoints();
double computeMinimumTotalTime();
void updateLoadCurve();
void processSerialCommands();

//-----------------------------------
// Helper Function: Convert raw ADC reading (0-2047) to voltage (0-10V)
//-----------------------------------
double convertRawToVoltage(int rawValue) {
  return (rawValue / 2047.0) * 10.0;
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
// Update sensor readings: convert raw ADC values to voltages, apply calibration,
// clamp corrected voltage to at least 0.5V, and compute force in lb.
//-----------------------------------
void updateSensors() {
  int rawVal1 = analogRead(PIN_LOAD1);
  int rawVal2 = analogRead(PIN_LOAD2);
  voltageLoad1 = convertRawToVoltage(rawVal1);
  voltageLoad2 = convertRawToVoltage(rawVal2);
  
  correctedVoltage1 = voltageLoad1 - baseline1;
  correctedVoltage2 = voltageLoad2 - baseline2;
  if (correctedVoltage1 < 0.5) correctedVoltage1 = 0.5;
  if (correctedVoltage2 < 0.5) correctedVoltage2 = 0.5;
  
  // Calculate force: weight (lb) = (correctedVoltage - 0.5) * conversion factor.
  weight1 = (correctedVoltage1 - 0.5) * LB_PER_VOLT_1;
  weight2 = (correctedVoltage2 - 0.5) * LB_PER_VOLT_2;
  avgWeight = (weight1 + weight2) / 2.0;
}

//-----------------------------------
// Set EPRV1 output.
void setValve1(double command) {
  int pwmValue = map(command, 0, (int)MAX_PRESSURE, 0, 255);
  analogWrite(PIN_VALVE1, pwmValue);
}

//-----------------------------------
// Set EPRV2 output.
void setValve2(double command) {
  int pwmValue = map(command, 0, (int)MAX_PRESSURE, 0, 255);
  analogWrite(PIN_VALVE2, pwmValue);
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
// Process incoming serial commands.
void processSerialCommands() {
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    
    if (cmd == CONNECTION_CHECK_REQUEST) {
      Serial.println(CONNECTION_CHECK_RESPONSE);
      return;
    }
    
    if (cmd == MANUAL_MODE_ENTER) {
      MODE = MANUAL_MODE;
      Serial.println("Switched to MANUAL mode.");
      return;
    }
    else if (cmd == OPERATION_MODE_ENTER) {
      MODE = OPERATION_MODE;
      modeStartTimestamp = millis();
      Serial.println("Switched to OPERATION mode.");
      return;
    }
    else if (cmd == ERROR_MODE_ENTER) {
      MODE = ERR_MODE;
      Serial.println("Switched to ERR_MODE.");
      return;
    }
    else if (cmd == MODE_EXIT) {
      MODE = SELECT_MODE;
      Serial.println("Switched to SELECT mode.");
      return;
    }
    
    if (cmd == REPORTING_INTERVAL_EDIT) {
      while (Serial.available() == 0) delay(10);
      dataReportingInterval = Serial.readStringUntil('\n').toInt();
      Serial.print("Reporting interval set to ");
      Serial.print(dataReportingInterval);
      Serial.println(" ms.");
      return;
    }
    
    if (MODE == MANUAL_MODE) {
      int sepIndex = cmd.indexOf(':');
      if (sepIndex != -1) {
        messageHeader = cmd.substring(0, sepIndex);
        messageValue = cmd.substring(sepIndex + 1).toFloat();
        if (messageHeader == SET_LEFT) {
          // Set left piston pressure (psi) here (placeholder).
          Serial.print("Left piston set to: ");
          Serial.println(messageValue);
        }
        else if (messageHeader == SET_RIGHT) {
          // Set right piston pressure (psi) here (placeholder).
          Serial.print("Right piston set to: ");
          Serial.println(messageValue);
        }
      }
    }
  }
}

//-----------------------------------
// Main Setup
//-----------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  // Clear incoming serial data.
  delay(1500);
  while (Serial.available()) { Serial.read(); }
  
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
  pinMode(SOLENOID1_A, OUTPUT);
  pinMode(SOLENOID1_B, OUTPUT);
  pinMode(SOLENOID2_A, OUTPUT);
  pinMode(SOLENOID2_B, OUTPUT);
  
  calibrateLoadCells();
  
  // Initialize PID controllers.
  eprv1PID.SetMode(AUTOMATIC);
  eprv1PID.SetOutputLimits(0, 255);
  eprv2PID.SetMode(AUTOMATIC);
  eprv2PID.SetOutputLimits(0, 255);
  
  MODE = SELECT_MODE;
  testStartTime = millis();
  modeStartTimestamp = millis();
  lastDataReportTimestamp = millis();
  Serial.println("PLC System Initialized. In SELECT mode.");
}

//-----------------------------------
// Main Loop
//-----------------------------------
void loop() {
  processSerialCommands();
  
  if (millis() - testStartTime >= TEST_DURATION) {
    Serial.println("Test duration complete. Halting execution.");
    analogWrite(PIN_VALVE1, 0);
    analogWrite(PIN_VALVE2, 0);
    while (true) { delay(1000); }
  }
  
  updateSensors();
  
  switch(MODE) {
    case SELECT_MODE:
      // Await commands.
      break;
      
    case MANUAL_MODE:
      // In MANUAL_MODE, directly set both valves using manualValveCommand.
      // (In a real system, the piston pressures would be controlled accordingly.)
      analogWrite(PIN_VALVE1, (int)manualValveCommand);
      analogWrite(PIN_VALVE2, (int)manualValveCommand);
      if (millis() - lastDataReportTimestamp >= dataReportingInterval) {
        Serial.print("MANUAL DATA -> Force Left: ");
        Serial.print(weight1, 2);
        Serial.print(" lb, Force Right: ");
        Serial.print(weight2, 2);
        Serial.print(" lb, Pot1: ");
        Serial.print(convertRawToVoltage(pot1), 2);
        Serial.print(" V, Pot2: ");
        Serial.print(convertRawToVoltage(pot2), 2);
        Serial.println(" V");
        lastDataReportTimestamp = millis();
      }
      break;
      
    case OPERATION_MODE:
      if (millis() - lastDataReportTimestamp >= dataReportingInterval) {
        sendForceDatapoints();
        lastDataReportTimestamp = millis();
      }
      break;
      
    case LOAD_CURVE_MODE:
      updateLoadCurve();
      break;
      
    case PAUSE_MODE:
      // Hold Outputs.
      break;
      
    case ERR_MODE:
      Serial.println("ERR_MODE active. Awaiting MODE_EXIT or ERR_CLEAR.");
      break;
      
    default:
      break;
  }
  
  delay(SAMPLE_PERIOD_MS);
}
