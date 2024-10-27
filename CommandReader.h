#ifndef COMMANDREADER_H
#define COMMANDREADER_H

#include <vector>
#include <string>

// Structure representing a flight command for the UAV
struct Command {
    double time;  // Time to execute the command
    int num;      // UAV number
    double x;     // Target X coordinate
    double y;     // Target Y coordinate
};

// Function to read commands from a file
std::vector<Command> readCommands(const std::string& filename);

#endif