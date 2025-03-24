// --- Required Libraries ---
#include <PID_v1.h>
#include <Semaphore.h>

// --- Configuration Parameters ---
const double MAX_PRESSURE = 100.0;           // Maximum safe pressure (engineering units)
const double DEFAULT_RAMP_RATE = 1.0;        // Default ramp rate (pressure units per update)
const float WARMUP_TOLERANCE_PERCENT = 2.0;  // 2% tolerance for warmup pressure error
const unsigned long SAMPLE_PERIOD_MS = 100;  // Main loop sample period (ms)

// For the flow-rate check: assume the EPRV (4 SQFM, 1/4″ NPT) yields an effective maximum ramp rate
// To be adjusted after discussion with group
const double EFFECTIVE_MAX_RAMP_RATE = 0.5; // pressure units per second

// --- Pin Definitions ---
// (Using PLC naming scheme as per Industrial Shields documentation.)
#define PIN_POT1       I0_7    // String potentiometer 1 (if used for display only)
#define PIN_POT2       I0_8    // String potentiometer 2
#define PIN_LOAD1      I0_9    // Load cell 1 (no longer used for setpoint calculation)
#define PIN_LOAD2      I0_10   // Load cell 2
#define PIN_PRESSURE1  I0_11   // Pressure sensor for piston 1 (EPRV 1)
#define PIN_PRESSURE2  I0_12   // Pressure sensor for piston 2 (EPRV 2)
#define PIN_VALVE      A0_5    // Analog output controlling the relief valve

// Solenoid valve outputs (digital/relay outputs)
#define SOLENOID1_A    R0_1    // Solenoid valve 1, position A (e.g. ON)
#define SOLENOID1_B    R0_2    // Solenoid valve 1, position B (e.g. OFF)
#define SOLENOID2_A    R0_3    // Solenoid valve 2, position A
#define SOLENOID2_B    R0_4    // Solenoid valve 2, position B

// --- Global Sensor Variables ---
// Raw sensor readings
float pot1, pot2;
float load1, load2;
int rawPressure1, rawPressure2;

// Converted sensor values (engineering units)
float convPot1, convPot2;         // e.g., 0–10 V values
float convLoad1, convLoad2;       // e.g., 0–500 load units
float convPressure1, convPressure2; // Pressure in same units as MAX_PRESSURE

// --- PID Control Setup ---
// PID variables – note that in load curve mode, the setpoints come solely from the curve.
double setpoint = 0, input = 0, output = 0;
double Kp = 2.0, Ki = 0.5, Kd = 1.0; 
PID pressurePID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// --- Mode Definitions ---
// We now have only MANUAL_MODE, LOAD_CURVE_MODE, and WARMUP_MODE.
enum ControlMode {
    MANUAL_MODE,
    LOAD_CURVE_MODE,
    WARMUP_MODE
};

ControlMode currentMode = MANUAL_MODE;

// --- Manual Mode Variable ---
double manualValveCommand = 0;  // Set by the GUI in manual mode

// --- Warmup Variables ---
bool warmupRequested = false;
unsigned long warmupDuration = 60000;  // (ms) default 60 seconds
int warmupSteps = 60;                  // default steps during warmup
double warmupTarget = MAX_PRESSURE;    // desired target pressure at end of warmup

// --- Load Curve Variables ---
// We now expect a curve command of the form:
// "CURVE:target1:time1,target2:time2,..."
// where each time is in milliseconds for the ramp from the previous point to this target.
#define MAX_CURVE_POINTS 20
double curvePoints[MAX_CURVE_POINTS];      // Target pressures for each segment
unsigned long curveTimes[MAX_CURVE_POINTS];  // Desired ramp times (ms) for each segment
int numCurvePoints = 0;
bool curveLoaded = false;

// state machine for executing the curve - Different states correspond to different modes of load cell (ramp up, down, done)
enum LoadCurveState { LC_RAMP, LC_DONE };
LoadCurveState lcState = LC_RAMP;
int currentCurveSegment = 0;
double previousSegmentTarget = 0;  // Starting pressure for current segment
unsigned long segmentStartTime = 0; // When the current segment started

// --- Serial Monitoring Variables ---
unsigned long serialCharCount = 0;
unsigned long lastSerialCheck = 0;
const unsigned int MAX_CHARS_PER_SEC = 1024;  // Example threshold

// --- Function Prototypes ---
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);
float convertAnalogValue(int raw, float minVal, float maxVal);
void readSensors();
void setValve(double command);
void startWarmupRoutine();
void processSerialCommands();
void updateLoadCurve();
double computeMinimumTotalTime();

// --- Helper Functions ---

// mapFloat: maps a float value from one range to another.
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// convertAnalogValue: converts a raw analog reading (0–2047) to a real-world value.
float convertAnalogValue(int raw, float minVal, float maxVal) {
    return mapFloat(raw, 0, 2047, minVal, maxVal);
    }

// --- Sensor Reading ---
// Read all sensors and convert to engineering units.
void readSensors() {
    pot1       = analogRead(PIN_POT1);
    pot2       = analogRead(PIN_POT2);
    load1      = analogRead(PIN_LOAD1);
    load2      = analogRead(PIN_LOAD2);
    rawPressure1 = analogRead(PIN_PRESSURE1);
    rawPressure2 = analogRead(PIN_PRESSURE2);

    // Convert readings; adjust conversion ranges as necessary.
    convPot1 = convertAnalogValue(pot1, 0.0, 10.0);
    convPot2 = convertAnalogValue(pot2, 0.0, 10.0);
    convLoad1 = convertAnalogValue(load1, 0.0, 500.0);
    convLoad2 = convertAnalogValue(load2, 0.0, 500.0);
    convPressure1 = convertAnalogValue(rawPressure1, 0.0, MAX_PRESSURE);
    convPressure2 = convertAnalogValue(rawPressure2, 0.0, MAX_PRESSURE);
}

// --- Valve Control ---
// Map a pressure command (0–MAX_PRESSURE) to a PWM value (0–255) and output it.
void setValve(double command) {
    int valveValue = map(command, 0, MAX_PRESSURE, 0, 255);
    analogWrite(PIN_VALVE, valveValue);
    // For debugging, you may uncomment the following:
    // Serial.print("Valve set to: "); Serial.println(command);
}

// --- Warmup Routine ---
// Executes a warmup routine using parameters supplied via Serial.
// During warmup, each piston (here piston 1 and 2 are checked individually) is evaluated.
void startWarmupRoutine() {
    Serial.println("Starting warmup routine...");
    double commandIncrement = warmupTarget / warmupSteps;
    double valveCommand = 0;
    for (int i = 0; i < warmupSteps; i++) {
    valveCommand += commandIncrement;
    setValve(valveCommand);
    delay(warmupDuration / warmupSteps);
    
    readSensors();
    double expectedPressure1 = mapFloat(convLoad1, 0.0, 500.0, 0, MAX_PRESSURE);
    double expectedPressure2 = mapFloat(convLoad2, 0.0, 500.0, 0, MAX_PRESSURE);
    double errorPercent1 = fabs(expectedPressure1 - convPressure1) / expectedPressure1 * 100.0;
    double errorPercent2 = fabs(expectedPressure2 - convPressure2) / expectedPressure2 * 100.0;
    
    Serial.print("Warmup Piston 1 -> Expected: ");
    Serial.print(expectedPressure1);
    Serial.print(" | Actual: ");
    Serial.print(convPressure1);
    Serial.print(" | Error (%): ");
    Serial.println(errorPercent1);
    
    Serial.print("Warmup Piston 2 -> Expected: ");
    Serial.print(expectedPressure2);
    Serial.print(" | Actual: ");
    Serial.print(convPressure2);
    Serial.print(" | Error (%): ");
    Serial.println(errorPercent2);
    
    if (errorPercent1 < WARMUP_TOLERANCE_PERCENT && errorPercent2 < WARMUP_TOLERANCE_PERCENT) {
        Serial.println("Warmup successful for both pistons.");
    } else {
        Serial.println("Warmup error: Pressure mismatch detected on one or both pistons.");
    }
    }
    // After warmup, reset mode (e.g., to manual or load curve)
    currentMode = MANUAL_MODE;
    warmupRequested = false;
}

// --- Compute Minimum Test Time ---
// Computes the minimum total time (in ms) required based on the effective ramp rate.
// Starting at 0, for each segment and finally ramping down to 0.
double computeMinimumTotalTime() {
    double total = 0;
    double prev = 0;
    for (int i = 0; i < numCurvePoints; i++) {
    double delta = fabs(curvePoints[i] - prev);
    double segTime = (delta / EFFECTIVE_MAX_RAMP_RATE) * 1000; // ms
    total += segTime;
    prev = curvePoints[i];
    }
    // Add ramp-down time from last point to 0.
    total += (fabs(prev - 0) / EFFECTIVE_MAX_RAMP_RATE) * 1000;
    return total;
}

// --- Serial Command Processing ---
// Processes incoming Serial commands from the GUI.
// Commands are expected as text and are used to switch modes, set manual valve values, load a curve, or set warmup parameters.
// Also monitors character rate on the Serial port.
void processSerialCommands() {
    while (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    serialCharCount += command.length();

    // Monitor serial character rate (once per second)
    if (millis() - lastSerialCheck >= 1000) {
        if (serialCharCount > MAX_CHARS_PER_SEC) {
        Serial.println("Warning: Serial input rate exceeds threshold!");
        }
        serialCharCount = 0;
        lastSerialCheck = millis();
    }

    // MODE command: allow only MANUAL, LOAD_CURVE, or WARMUP modes.
    if (command.startsWith("MODE:")) {
        String modeStr = command.substring(5);
        modeStr.trim();
        modeStr.toUpperCase();
        if (modeStr == "MANUAL") {
        currentMode = MANUAL_MODE;
        Serial.println("Switched to MANUAL MODE");
        } else if (modeStr == "LOAD_CURVE") {
        currentMode = LOAD_CURVE_MODE;
        lcState = LC_RAMP;
        currentCurveSegment = 0;
        previousSegmentTarget = 0;
        segmentStartTime = millis();
        Serial.println("Switched to LOAD CURVE MODE");
        } else if (modeStr == "WARMUP") {
        currentMode = WARMUP_MODE;
        Serial.println("Switched to WARMUP MODE");
        }
    }
    // VALVE command for manual mode.
    else if (command.startsWith("VALVE:")) {
        String valStr = command.substring(6);
        valStr.trim();
        manualValveCommand = round(valStr.toFloat());
        Serial.print("Manual valve command updated: ");
        Serial.println(manualValveCommand);
    }
    // CURVE command: expects a list of segments in the format "target:time" (time in ms).
    // Input would be of form: "CURVE:50:30000,70:40000,90:50000", which is of form target_psi:time_ms, target2_psi:time_ms, ...
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
            int colonIndex = token.indexOf(':');
            if (colonIndex != -1) {
                String targetStr = token.substring(0, colonIndex);
                String timeStr = token.substring(colonIndex + 1);
                curvePoints[numCurvePoints] = round(targetStr.toFloat());
                curveTimes[numCurvePoints] = timeStr.toInt();
            }
            numCurvePoints++;
            }
            break;
        } else {
            token = curveStr.substring(startIndex, commaIndex);
            token.trim();
            if (token.length() > 0 && numCurvePoints < MAX_CURVE_POINTS) {
            int colonIndex = token.indexOf(':');
            if (colonIndex != -1) {
                String targetStr = token.substring(0, colonIndex);
                String timeStr = token.substring(colonIndex + 1);
                curvePoints[numCurvePoints] = round(targetStr.toFloat());
                curveTimes[numCurvePoints] = timeStr.toInt();
            }
            numCurvePoints++;
            }
            startIndex = commaIndex + 1;
        }
        }
        
        // Compute minimum total time required based on flow rate.
        double minTotalTime = computeMinimumTotalTime();
        double userTotalTime = 0;
        for (int i = 0; i < numCurvePoints; i++) {
        userTotalTime += curveTimes[i];
        }
        if (userTotalTime < minTotalTime) {
        // Scale the times proportionally.
        double scale = minTotalTime / userTotalTime;
        for (int i = 0; i < numCurvePoints; i++) {
            curveTimes[i] = curveTimes[i] * scale;
        }
        Serial.print("Adjusted total test time to minimum possible: ");
        Serial.print(minTotalTime / 1000);
        Serial.println(" seconds.");
        }
        curveLoaded = true;
        // Reset state for new load curve execution.
        currentMode = LOAD_CURVE_MODE;
        lcState = LC_RAMP;
        currentCurveSegment = 0;
        previousSegmentTarget = 0;
        segmentStartTime = millis();
        Serial.print("Load curve loaded with ");
        Serial.print(numCurvePoints);
        Serial.println(" segments.");
    }
    // WARMUP command: "WARMUP:duration,steps,target"
    else if (command.startsWith("WARMUP:")) {
        String warmupStr = command.substring(7);
        warmupStr.trim();
        int firstComma = warmupStr.indexOf(',');
        int secondComma = warmupStr.indexOf(',', firstComma + 1);
        if (firstComma != -1 && secondComma != -1) {
        warmupDuration = warmupStr.substring(0, firstComma).toInt();
        warmupSteps = warmupStr.substring(firstComma + 1, secondComma).toInt();
        warmupTarget = warmupStr.substring(secondComma + 1).toFloat();
        warmupRequested = true;
        Serial.println("Warmup parameters received.");
        } else {
        Serial.println("Invalid WARMUP command format.");
        }
    }
    }
}

// --- Update Load Curve ---
// In LOAD_CURVE_MODE, use the load curve segments to drive the PID setpoint.
// For each segment, we linearly interpolate from the previous target (starting at 0)
// to the current segment’s target over the specified time. After the last segment,
// we ramp down to 0.
void updateLoadCurve() {
    // Only execute if a curve is loaded.
    if (!curveLoaded) {
    Serial.println("LOAD_CURVE_MODE -> No load curve loaded.");
    return;
    }

    unsigned long now = millis();
    if (currentCurveSegment < numCurvePoints) {
    // For the current segment, the desired ramp time is:
    unsigned long segTime = curveTimes[currentCurveSegment];
    unsigned long elapsed = now - segmentStartTime;
    double target = curvePoints[currentCurveSegment];
    // Compute new setpoint via linear interpolation.
    if (elapsed < segTime) {
        setpoint = previousSegmentTarget + (target - previousSegmentTarget) * (elapsed / (double)segTime);
    } else {
        // Segment complete; set setpoint to target and update for next segment.
        setpoint = target;
        previousSegmentTarget = target;
        currentCurveSegment++;
        segmentStartTime = now;
    }
    }
    else {
    // All segments complete; ramp down from last target to 0.
    double delta = previousSegmentTarget; // current level above 0
    // Compute minimum ramp down time for the last segment.
    unsigned long rampDownTime = (unsigned long)((delta / EFFECTIVE_MAX_RAMP_RATE) * 1000);
    unsigned long elapsed = now - segmentStartTime;
    if (elapsed < rampDownTime) {
        setpoint = previousSegmentTarget * (1.0 - (elapsed / (double)rampDownTime));
    } else {
        setpoint = 0;
        // Optionally, signal completion.
        Serial.println("Load curve execution complete.");
        curveLoaded = false;
    }
    }

    // Use the pressure sensor (e.g., piston 1) as the process variable.
    input = convPressure1;

    // Run PID control.
    pressurePID.Compute();

    // Can uncomment below statement to add ramp-rate safety here to limit output change.
    // static double lastOutput = 0;
    // if (output > lastOutput + RAMP_RATE) {
    //     output = lastOutput + RAMP_RATE;
    // }
    // lastOutput = output;
    setValve(output);

    // For debugging, print the current setpoint and measured pressure.
    Serial.print("LOAD_CURVE_MODE -> Setpoint: ");
    Serial.print(setpoint);
    Serial.print(" | Pressure: ");
    Serial.print(convPressure1);
    Serial.print(" | PID Output: ");
    Serial.println(output);
}

// --- setup() ---
// Initialize Serial, configure pin modes, and set up the PID.
void setup() {
    Serial.begin(115200);
    while (!Serial) {}  // Wait for Serial initialization
    Serial.println("System initializing...");

    // Configure sensor input pins.
    pinMode(PIN_POT1, INPUT);
    pinMode(PIN_POT2, INPUT);
    pinMode(PIN_LOAD1, INPUT);
    pinMode(PIN_LOAD2, INPUT);
    pinMode(PIN_PRESSURE1, INPUT);
    pinMode(PIN_PRESSURE2, INPUT);

    // Configure the analog output for the valve.
    pinMode(PIN_VALVE, OUTPUT);

    // Configure solenoid valve outputs.
    pinMode(SOLENOID1_A, OUTPUT);
    pinMode(SOLENOID1_B, OUTPUT);
    pinMode(SOLENOID2_A, OUTPUT);
    pinMode(SOLENOID2_B, OUTPUT);

    // Initialize PID control using Load Cell values
    pressurePID.SetMode(AUTOMATIC);
    pressurePID.SetOutputLimits(0, MAX_PRESSURE);

    // CAN ADD ADDITIONAL FEATURES HERE
}

// --- loop() ---
// Main loop: process Serial commands and run the selected control mode.
void loop() {
    processSerialCommands();  // Process any incoming commands

    // If warmup is requested, execute warmup.
    if (warmupRequested && currentMode == WARMUP_MODE) {
        startWarmupRoutine();
    }

    // Execute the current mode.
    switch (currentMode) {
        case MANUAL_MODE: {
            // Manual mode: directly apply the valve command.
            setValve(manualValveCommand);
            Serial.print("MANUAL_MODE -> Valve Command: ");
            Serial.println(manualValveCommand);
            break;
        }
        
        case LOAD_CURVE_MODE: {
            updateLoadCurve();
            break;
        }
    }

    delay(SAMPLE_PERIOD_MS);
}
