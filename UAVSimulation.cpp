// yovel cohen
// UAVSimulation.cpp
#include "SimParamsReader.h"
#include "CommandReader.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>
#include <fstream>
#include <algorithm>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace std;

// Class to manage individual UAV state and behavior
class UAV {
private:
    double x;
    double y;
    double azimuth;
    double targetX;
    double targetY;
    bool commandReceived;
    bool inCircularMode;
    double prepAngle;
    ofstream outputFile;

    const double V0;
    const double R;
    const double dt;
    const double distanceThreshold;

public:
    UAV(int id, const SimulationParams& params)
        : x(params.X0), y(params.Y0), azimuth(params.azimuth),
        targetX(params.X0), targetY(params.Y0),
        commandReceived(false), inCircularMode(false), prepAngle(0),
        V0(params.V0), R(params.R), dt(params.dt),
        distanceThreshold(params.V0* params.dt) // The distance threshold, defined as V0 * dt, provides an indication of the proximity to the target point for each UAV.
    {
        // Open files to write data for each UAV
        outputFile.open("UAV" + to_string(id + 1) + ".txt");
        outputFile << fixed << setprecision(2);
        writeState(0.0);
    }

    ~UAV() {
        if (outputFile.is_open()) {
            outputFile.close();
        }
    }
    // Prevent copying
    UAV(const UAV&) = delete;
    UAV& operator=(const UAV&) = delete;

    // Allow moving 
    UAV(UAV&&) noexcept = default;
    UAV& operator=(UAV&&) noexcept = default;

    void setNewTarget(double newX, double newY) {
        targetX = newX;
        targetY = newY;
        commandReceived = true;
        inCircularMode = false; // Reset circular mode for the new target
        prepAngle = 0; //  Reset circular angle for the new target
    }
    // Update the UAV 
    void update(double currentTime) {
        double distanceToTarget = calculateDistance(targetX, targetY);

        if (!commandReceived) {//There is no current command for this UAV, which can happen either because the time has not yet arrived
                               // or because it has already reached its destination
            if (prepAngle == 0) {
                // The process of adjusting to move in a circle around the target point has not yet begun
                // No command yet, fly straight with initial azimuth
                updateStraightLineMotion();
            }
            else if (inCircularMode) {
                // The UAV is in standby mode, moving in a circle around the last target point it reached, according to the specified radius.
                // Continue circular motion around the target clockwise
                updateCircularMotion();
            }
            else {
                // After the UAV has reached its destination, it is in the process of adjusting its movement to reach the circumference of the circle surrounding the target point.
                if (prepAngle >= (3 * M_PI / 2) && fabs(distanceToTarget - R) <= distanceThreshold * 4) {
                    //The UAV has completed its adjustment and is now on the relevant circumference of the circle, so from this point on, it will continue to orbit along the perimeter.
                    inCircularMode = true;
                }
                else if (prepAngle <= (3 * M_PI / 2)) { 
                    prepAngle += (V0 / R) * dt; // Step to change the angle, The UAV has not yet completed its adjustment
                                               // As long as we have not made three quarters of a circle, the UAV will continue to rotate clockwise, to adjust is movement to reach the circumference of the circle surrounding the target point
                    updateCircularMotion();
                }
                else {
                    // fter the UAV has moved three-quarters of a circle clockwise, it still needs to move further to reach the circumference of the circle
                    // so he Continue in a straight line until reach a point on the circle around the target point
                    updateStraightLineMotion();
                }
            }
        }
        else if (distanceToTarget > distanceThreshold) {
            // Command received, adjust azimuth and move towards the target,We have not yet reached the target point
            if (prepAngle == 0) {
                double targetAzimuth = atan2(targetY - y, targetX - x) * 180.0 / M_PI; // (180.0 / M_PI) converts the result from radians to degrees
                updateAzimuth(targetAzimuth);
                updateStraightLineMotion();
            }
        }
        else {
            commandReceived = false;
            prepAngle += (V0 / R) * dt; // start step for change the angle for the adjusting to move in a circle around the target point
                                       // As long as we have not made three quarters of a circle, the UAV will continue to rotate clockwise
            updateCircularMotion();
        }
        // Write the current state to file belong to the UAV
        writeState(currentTime);
    }

private:
    // Function to update straight-line motion
    void updateStraightLineMotion() {
        double angleRad = azimuth * M_PI / 180.0; // convert for radians from degrees
        x += V0 * dt * cos(angleRad);
        y += V0 * dt * sin(angleRad);
    }
    // Function to update UAV when flying in circular motion around the target or flying to the circumference of the circle surrounding the target point
    void updateCircularMotion() {

        double omega = -V0 / R;  // Calculate the angular velocity (Clockwise motion)

        azimuth = fmod(azimuth + omega * dt * 180.0 / M_PI + 360, 360);  // Update the azimuth
        // Update position along the circle
        double angleRad = azimuth * M_PI / 180.0;
        x += V0 * cos(angleRad) * dt;
        y += V0 * sin(angleRad) * dt;
    }
    // Gradual update of azimuth within turn radius constraints
    void updateAzimuth(double targetAzimuth) {
        double deltaAzimuth = targetAzimuth - azimuth;

        // Normalize the angle difference to [-180, 180]
        if (deltaAzimuth > 180) deltaAzimuth -= 360;
        if (deltaAzimuth < -180) deltaAzimuth += 360;
        // Limit the azimuth change by the max turn rate
        double maxChange = (V0 / R) * dt * 180.0 / M_PI;
        if (fabs(deltaAzimuth) > maxChange) {
            azimuth += (deltaAzimuth > 0 ? maxChange : -maxChange);
        }
        else {
            azimuth = targetAzimuth;
        }
        azimuth = fmod(azimuth + 360, 360); // Keep azimuth in [0, 360)
    }
    // Function to calculate the distance to the target
    double calculateDistance(double x2, double y2) const {
        return sqrt(pow(x2 - x, 2) + pow(y2 - y, 2));
    }
    //Write the current state to file belong to the UAV
    void writeState(double currentTime) {
        outputFile << currentTime << " " << x << " " << y << " " << azimuth << endl;
    }
};

// Class to manage the simulation
class UAVSimulation {
private:
    vector<UAV> uavs;
    const vector<Command>& commands;
    const SimulationParams& params;
    size_t currentCommandIndex;

public:
    UAVSimulation(const SimulationParams& params, const vector<Command>& commands)
        : commands(commands), params(params), currentCommandIndex(0) {

        uavs.reserve(params.numUAVs);
        for (int i = 0; i < params.numUAVs; ++i) {
            uavs.emplace_back(i, params);
        }
    }

    void run() {
        double currentTime = 0.0;
        // Main simulation loop
        while (currentTime <= params.timeLimit) {
            processCommands(currentTime);
            updateUAVs(currentTime);
            // Increment Time step
            currentTime += params.dt;
        }
    }

private:
    void processCommands(double currentTime) {
        // Process any commands at the current time
        while (currentCommandIndex < commands.size() &&
            commands[currentCommandIndex].time <= currentTime) {
            const Command& cmd = commands[currentCommandIndex];
            if (cmd.num >= 0 && cmd.num < params.numUAVs) {
                uavs[cmd.num].setNewTarget(cmd.x, cmd.y);
            }
            currentCommandIndex++;
        }
    }

    void updateUAVs(double currentTime) {
        for (auto& uav : uavs) {
            uav.update(currentTime);
        }
    }
};
// Main simulation function
int main() {
    // Read simulation parameters and commands
    SimulationParams params = readSimParams("SimParams.ini");
    vector<Command> commands = readCommands("SimCmds.txt");
    // Sort the commands based on time
    sort(commands.begin(), commands.end(),
        [](const Command& a, const Command& b) { return a.time < b.time; });

    UAVSimulation simulation(params, commands);
    simulation.run();

    return 0;
}