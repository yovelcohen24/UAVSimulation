#include "SimParamsReader.h"
#include <fstream>
#include <iostream>
#include <sstream>

using namespace std;

SimulationParams readSimParams(const string& filename) {
    // Initialize all fields to avoid uninitialized variable errors
    SimulationParams params = { 0.0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    ifstream file(filename);
    string line;

    // Check if the file was opened successfully
    if (!file.is_open()) {
        cerr << "Error opening file: " << filename << endl;
        return params;
    }

    // Read each line of the file
    while (getline(file, line)) {
        istringstream iss(line);
        string key;
        char equals;
        double value;

        // Parse the line into key-value pairs
        if (iss >> key >> equals >> value) {
            if (key == "Dt") params.dt = value;
            else if (key == "N_uav") params.numUAVs = static_cast<int>(value);
            else if (key == "R") params.R = value;
            else if (key == "X0") params.X0 = value;
            else if (key == "Y0") params.Y0 = value;
            else if (key == "Z0") params.Z0 = value;
            else if (key == "V0") params.V0 = value;
            else if (key == "Az") params.azimuth = value;
            else if (key == "TimeLim") params.timeLimit = value;
        }
    }

    file.close();
    return params;
}