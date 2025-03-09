#include <PID_v1.h>

// -------------------------------
// Configuration Parameters
// -------------------------------
const double MAX_PRESSURE = 100.0;           // Maximum safe pressure (units as needed)
const double RAMP_RATE    = 1.0;             // Maximum allowed increase/decrease per sample
const float WARMUP_TOLERANCE_PERCENT = 2.0;  // 2% tolerance for warmup pressure error

// -------------------------------
// Define Sensor and Actuator Pins (Change pins once we receive the PLC)
// -------------------------------

// NUMBERING OF THE PLC PINS IS LIKE A21, B21, ETC AND FOLLOW THE SAME NUMBERING SCHEME AS THE PLC. SPECIFIC LIBRARY, HAVE LINK IN USER MANUAL MANUAL AND NEED
// TO INCLUDE IN THE IDE TO USE THE CORRECT PIN NUMBERS.
#define PIN_POT1       34    // String potentiometer 1
#define PIN_POT2       35    // String potentiometer 2
#define PIN_LOAD1      32    // Load cell 1
#define PIN_LOAD2      33    // Load cell 2
#define PIN_PRESSURE1  25    // Pressure sensor for EPRV 1
#define PIN_PRESSURE2  26    // Pressure sensor for EPRV 2
#define PIN_VALVE      27    // Output pin to control the relief valve

// ADD SOLENOID VALVE PINS AS WELL.
// -------------------------------
// Global Variables for Sensor Data
// -------------------------------
float pot1, pot2;           // String potentiometer readings
float load1, load2;         // Load cell readings
float pressure1, pressure2; // Pressure sensor (EPRV) readings

// -------------------------------
// PID Control Setup (Auto Mode)
// -------------------------------
double setpoint, input, output; // PID variables
double Kp = 2.0, Ki = 0.5, Kd = 1.0; 
PID pressurePID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// -------------------------------
// Control Mode Definitions
// -------------------------------
enum ControlMode {
  AUTO_MODE,
  MANUAL_MODE,
  LOAD_CURVE_MODE
};
ControlMode currentMode = AUTO_MODE;

// -------------------------------
// Manual Mode Variables
// -------------------------------
double manualValveCommand = 0;  // Set by the GUI for manual control

// -------------------------------
// Load Curve Mode Variables
// -------------------------------
#define MAX_CURVE_POINTS 20
float loadCurve[MAX_CURVE_POINTS]; // Array to hold target pressures
int numCurvePoints = 0;              // Number of points loaded
bool curveLoaded = false;

enum LoadCurveState { LC_IDLE, LC_RAMP_UP, LC_HOLD, LC_RAMP_DOWN, LC_DONE };
LoadCurveState lcState = LC_IDLE;
int currentCurvePoint = 0;
double loadCurveValveCommand = 0;  // Current valve command during load curve execution
unsigned long lcStepStartTime = 0;
const unsigned long HOLD_DURATION = 5000; // Hold target for 3 seconds

// -------------------------------
// Function Prototypes
// -------------------------------
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);
void readSensors();
void setValve(double command);
void warmupRoutine();
void processSerialCommands();
void updateLoadCurve();


// setup(): Initialize Serial, sensors, PID, and run warmup.
void setup() {
  Serial.begin(115200);
  while (!Serial) {} // Wait for Serial to be ready
  Serial.println("System initializing...");

  // Initialize the PID controller.
  pressurePID.SetMode(AUTOMATIC);
  pressurePID.SetOutputLimits(0, MAX_PRESSURE);

  // Run warmup routine.
  warmupRoutine();
}

// mapFloat(): Map a float value from one range to another.
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// readSensors(): Read sensor data from hardware. Can add filtering here also.
// ADD CONVERSTION FUNCTION FOR THE ANALOG VALUES TO MAP THESE TO THE ACTUAL SENSOR VALUES
void readSensors() {
  pot1 = analogRead(PIN_POT1);
  pot2 = analogRead(PIN_POT2);
  load1 = analogRead(PIN_LOAD1);
  load2 = analogRead(PIN_LOAD2);
  pressure1 = analogRead(PIN_PRESSURE1);
  pressure2 = analogRead(PIN_PRESSURE2);

  // solenoid valves need to be added here as a sensor reading as well
}

// setValve(): Output the command to the relief valve - map the pressure (0–MAX_PRESSURE) to a PWM value (0–255).
void setValve(double command) {
  int valveValue = map(command, 0, MAX_PRESSURE, 0, 255);
  analogWrite(PIN_VALVE, valveValue);

  // Serial.print("Valve set to: ");
  // Serial.println(command);
}

// warmupRoutine(): Slowly ramp the valve command from 0 to MAX_PRESSURE over ~1 minute,
// then compare expected pressure from load cells with actual sensor readings.
// MODIFY THIS TO ENSURE THAT THE WARMUP ROUTINE TAKES SERIAL COMMANDS FROM THE GUI OF THE LAPTOP TO INPUT INSTEAD OF JUST MODIFYING THIS STUFF IN THE CODE
void warmupRoutine() {
  Serial.println("Starting warmup routine...");
  const int warmupDuration = 60000;  // 1 minute in ms
  const int warmupSteps = 60;        // 60 steps (1 step per second)
  double valveCommand = 0;
  double commandIncrement = MAX_PRESSURE / warmupSteps;
  
  for (int i = 0; i < warmupSteps; i++) {
    valveCommand += commandIncrement;
    setValve(valveCommand);
    delay(warmupDuration / warmupSteps);

    // Read sensors after ramp (NEED TO UPDATE EXPECTED AND ACTUAL PRESSURE TO BE INDIVIDUAL TO PISTONS)
    readSensors();
    float avgLoad = (load1 + load2) / 2.0;
    double expectedPressure = mapFloat(avgLoad, 0, 500, 0, MAX_PRESSURE);
    double actualPressure = (pressure1 + pressure2) / 2.0;
    double errorPercent = abs(expectedPressure - actualPressure) / expectedPressure * 100.0;
    
    Serial.print("Warmup -> Expected Pressure: ");
    Serial.print(expectedPressure);
    Serial.print(" | Actual Pressure: ");
    Serial.print(actualPressure);
    Serial.print(" | Error (%): ");
    Serial.println(errorPercent);

    if (errorPercent < WARMUP_TOLERANCE_PERCENT) {
      Serial.println("Warmup successful, no lockups in Air pistons found, please proceed with test.");
    } else {
      Serial.println("Warmup error: Pressure mismatch detected. Check system.");
    }
  }
}

// processSerialCommands(): Process incoming Serial commands from the GUI.
// ROUND THE NUMBERS OF THE SERIAL INPUT. ENSURE IT DOESN"T EXCEED THE BIT RATE AND MONITOR THE RATE AT WHICH THE DATA IS BEING SENT.
// CALCULATE HOW MANY CHARACTERS ARE BEING SENT AND MAKE SURE IT DOESN"T EXCEED THE BIT RATE OF THE PLC (it is 112500 bits/second). MIGHT NEED TO COMPRESS THE NUMBERS INTO ACTUAL VALUES
// AND NOT AS STRINGS. CHECK WHETHER THE VALUES RECEIVED ARE LSB FIRST OR MSB FIRST AS WELL.
void processSerialCommands() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    // Change operating mode
    if (command.startsWith("MODE:")) {
      String modeStr = command.substring(5);
      modeStr.trim();
      modeStr.toUpperCase();
      if (modeStr == "AUTO") {
        currentMode = AUTO_MODE;
        Serial.println("Switched to AUTO MODE");
      } else if (modeStr == "MANUAL") {
        currentMode = MANUAL_MODE;
        Serial.println("Switched to MANUAL MODE");
      } else if (modeStr == "LOAD_CURVE") {
        currentMode = LOAD_CURVE_MODE;
        // Reset load curve state variables
        lcState = LC_RAMP_UP;
        currentCurvePoint = 0;
        loadCurveValveCommand = 0;
        Serial.println("Switched to LOAD CURVE MODE");
      }
    }
    // In manual mode, update the valve command.
    else if (command.startsWith("VALVE:")) {
      String valStr = command.substring(6);
      valStr.trim();
      manualValveCommand = valStr.toFloat();
      Serial.print("Manual valve command updated: ");
      Serial.println(manualValveCommand);
    }
    // Load a curve: a comma-separated list of target pressures.
    else if (command.startsWith("CURVE:")) {
      String curveStr = command.substring(6);
      curveStr.trim();
      numCurvePoints = 0;
      int startIndex = 0;
      while (true) {
        int commaIndex = curveStr.indexOf(',', startIndex);
        String token;
        if (commaIndex == -1) {
          token = curveStr.substring(startIndex);
          token.trim();
          if (token.length() > 0 && numCurvePoints < MAX_CURVE_POINTS) {
            loadCurve[numCurvePoints++] = token.toFloat();
          }
          break;
        } else {
          token = curveStr.substring(startIndex, commaIndex);
          token.trim();
          if (token.length() > 0 && numCurvePoints < MAX_CURVE_POINTS) {
            loadCurve[numCurvePoints++] = token.toFloat();
          }
          startIndex = commaIndex + 1;
        }
      }
      if (numCurvePoints > 0) {
        curveLoaded = true;
        Serial.print("Load curve loaded with ");
        Serial.print(numCurvePoints);
        Serial.println(" points.");
      }
    }
  }
}

// updateLoadCurve(): Execute the load curve routine. For each curve point,
// ramp from 0 up to the target, hold, then ramp back to 0 before moving on.
// EACH CURVE POINT MIGHT HAVE VARIABLE RAMP RATE SO INTERPOLATE BETWEEN THEM AND USE THE MAX BETWEEN THE MAXIMUM RAMP RATE AND THE INTERPOLATED ONE.
void updateLoadCurve() {
  if (lcState == LC_RAMP_UP) {
    double target = loadCurve[currentCurvePoint];
    if (loadCurveValveCommand < target) {
      loadCurveValveCommand += RAMP_RATE;

      if (loadCurveValveCommand > target)
        loadCurveValveCommand = target;

      setValve(loadCurveValveCommand);
      Serial.print("Load Curve (Ramping Up): ");
      Serial.println(loadCurveValveCommand);

    } else {
      lcState = LC_HOLD;
      lcStepStartTime = millis();
      Serial.println("Load Curve: Holding at target");
    }

  }
  else if (lcState == LC_HOLD) {
    if (millis() - lcStepStartTime >= HOLD_DURATION) {
      lcState = LC_RAMP_DOWN;
      Serial.println("Load Curve: Ramping Down");
    }
  }

  else if (lcState == LC_RAMP_DOWN) {
    if (loadCurveValveCommand > 0) {
      loadCurveValveCommand -= RAMP_RATE;
      if (loadCurveValveCommand < 0)
        loadCurveValveCommand = 0;
      setValve(loadCurveValveCommand);
      Serial.print("Load Curve (Ramping Down): ");
      Serial.println(loadCurveValveCommand);

    } else {
      currentCurvePoint++;
      if (currentCurvePoint >= numCurvePoints) {
        lcState = LC_DONE;
        Serial.println("Load curve execution complete.");

      } else {
        lcState = LC_RAMP_UP;
        Serial.print("Moving to next curve point (");
        Serial.print(currentCurvePoint);
        Serial.println(")");
      }
    }
  }

  else if (lcState == LC_DONE) {
    Serial.println("Load curve sequence finished. Waiting for new command...");
  }
}


// loop(): Main loop that checks for GUI commands and processes control based on mode.
void loop() {
  processSerialCommands();  // Check and process commands from the GUI

  // Update sensor readings (for logging and processing)
  readSensors();

  switch (currentMode) {
    case AUTO_MODE: {
      // ----- AUTOMATIC MODE -----
      // Use load cell readings to compute a setpoint.
      float avgLoad = (load1 + load2) / 2.0;
      // Map load cell reading (assumed range 0–500) to pressure setpoint (0–MAX_PRESSURE).
      setpoint = mapFloat(avgLoad, 0, 500, 0, MAX_PRESSURE);
      // Use pressure sensor reading as PID input.
      input = pressure1;
      
      // Compute PID output.
      pressurePID.Compute();
      
      // Apply ramp-rate safety: limit output change.
      static double lastOutput = 0;
      if (output > lastOutput + RAMP_RATE) {
        output = lastOutput + RAMP_RATE;
      }
      lastOutput = output;
      
      setValve(output);
      
      Serial.print("AUTO_MODE -> Setpoint: ");
      Serial.print(setpoint);
      Serial.print(" | Input: ");
      Serial.print(input);
      Serial.print(" | PID Output: ");
      Serial.println(output);
      break;
    }
    case MANUAL_MODE: {
      // ----- MANUAL MODE -----
      // Directly set the valve command as provided by the GUI.
      setValve(manualValveCommand);
      Serial.print("MANUAL_MODE -> Valve Command: ");
      Serial.println(manualValveCommand);
      break;
    }
    case LOAD_CURVE_MODE: {
      // ----- LOAD CURVE MODE -----
      if (curveLoaded) {
        updateLoadCurve();
      } else {
        Serial.println("LOAD_CURVE_MODE -> No load curve loaded.");
      }
      break;
    }
  }
  delay(100);  // Adjust sample period based on calculations on bit rate and adjust the number values based on PLC sheet as mentioned in above comments.
}
