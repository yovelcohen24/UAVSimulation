#ifndef SIMPARAMSREADER_H
#define SIMPARAMSREADER_H

#include <string>

struct SimulationParams {
    double dt;      // Time step in seconds
    int numUAVs;    // Number of UAVs in the simulation
    double R;       // Minimum turning radius in meters
    double X0;      // Initial X-coordinate for UAVs
    double Y0;      // Initial Y-coordinate for UAVs
    double Z0;      // Initial Z-coordinate (height) for UAVs
    double V0;      // Initial velocity of UAVs in m/s
    double azimuth; // Initial azimuth (angle) for UAVs in degrees
    double timeLimit; // Maximum simulation time in seconds
};


// Function to read simulation parameters from a file
SimulationParams readSimParams(const std::string& filename);

#endif