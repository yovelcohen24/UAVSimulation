#include "CommandReader.h"
#include <fstream>
#include <iostream>
#include <sstream>

using namespace std;

// Reads the commands from SimCmds.txt and stores them in a vector
vector<Command> readCommands(const string& filename) {
    vector<Command> commands;
    ifstream file(filename);
    string line;

    // Check if the file was opened successfully
    if (!file.is_open()) {
        cerr << "Error opening file: " << filename << endl;
        return commands;
    }

    // Read each line of the file and parse it into a Command struct
    while (getline(file, line)) {
        istringstream iss(line);
        Command cmd;
        if (iss >> cmd.time >> cmd.num >> cmd.x >> cmd.y) {
            commands.push_back(cmd);  // Add the command to the list
        }
    }

    file.close();
    return commands;
}