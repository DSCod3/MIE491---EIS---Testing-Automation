#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cstdlib>  // for atof

// PID class since cannot use the libary outside of Arduino environment
class PID {
public:
    PID(double* input, double* output, double* setpoint,
        double Kp, double Ki, double Kd) :
        input(input), output(output), setpoint(setpoint),
        Kp(Kp), Ki(Ki), Kd(Kd), lastInput(0), lastOutput(0) { }

    void Compute() {
        // Very simple proportional-only simulation for demonstration
        *output = Kp * (*setpoint - *input);
    }

private:
    double* input;
    double* output;
    double* setpoint;
    double Kp, Ki, Kd;
    double lastInput;
    double lastOutput;
};

// Safety parameters
const double MAX_PRESSURE = 145.0;
const double RAMP_RATE    = 1.0;

// Global sensor variables
float pot1, pot2, load1, load2, pressure1, pressure2;

// PID variables
double setpoint, input, output;
double Kp = 2.0, Ki = 0.5, Kd = 1.0;
PID pressurePID(&input, &output, &setpoint, Kp, Ki, Kd);

// Helper: map a float value from one range to another.
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Function to parse a CSV line and update sensor variables
void parseCSVLine(const std::string &line) {
    std::istringstream ss(line);
    std::string token;
    std::vector<float> values;

    while (std::getline(ss, token, ',')) {
        values.push_back(std::atof(token.c_str()));
    }

    if (values.size() >= 6) {
        pot1      = values[0];
        pot2      = values[1];
        load1     = values[2];
        load2     = values[3];
        pressure1 = values[4];
        pressure2 = values[5];
    }
}

int main() {
    std::ifstream csvFile("data.csv"); 
    if (!csvFile.is_open()) {
        std::cerr << "Error opening data.csv" << std::endl;
        return 1;
    }

    std::string line;
    double lastOutput = 0.0;

    // Main loop simulation
    while (std::getline(csvFile, line)) {
        if (line.empty()) continue;

        parseCSVLine(line);

        // Compute desired setpoint from load cell readings
        float avgLoad = (load1 + load2) / 2.0f;
        // Assume load cell readings range from 0 to 500
        setpoint = mapFloat(avgLoad, 0, 500, 0, MAX_PRESSURE);

        // Use pressure1 as the measured variable
        input = pressure1;

        // Compute PID output
        pressurePID.Compute();

        // Apply ramp rate (safety feature)
        if (output > lastOutput + RAMP_RATE) {
            output = lastOutput + RAMP_RATE;
        }
        lastOutput = output;

        // Output the computed values to the console
        std::cout << "Setpoint: " << setpoint 
                << " | Input: " << input 
                << " | PID Output (Valve Command): " << output 
                << std::endl;
    }

    csvFile.close();
    return 0;
}
