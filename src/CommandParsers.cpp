#include "CommandParsers.h"

const char *ServoNamesById[] = {
    "NULL",            // ID 0
    "SHOULDER_MAIN_R", // ID 1
    "SHOULDER_MAIN_L", // ID 2
    "SHOULDER_TILT_R", // ID 3
    "SHOULDER_TILT_L", // ID 4
    "ELBOW_ROTATE_R",  // ID 5
    "ELBOW_ROTATE_L",  // ID 6
    "ELBOW_MAIN_R",    // ID 7
    "ELBOW_MAIN_L",    // ID 8
    "HIP_ROTATE_R",    // ID 9
    "HIP_ROTATE_L",    // ID 10
    "HIP_TILT_R",      // ID 11
    "HIP_TILT_L",      // ID 12
    "HIP_MAIN_R",      // ID 13
    "HIP_MAIN_L",      // ID 14
    "KNEE_R",          // ID 15
    "KNEE_L",          // ID 16
    "FOOT_MAIN_R",     // ID 17
    "FOOT_MAIN_L",     // ID 18
    "FOOT_TILT_R",     // ID 19
    "FOOT_TILT_L",     // ID 20
    "HEAD_HORIZONTAL", // ID 21
    "HEAD_VERTICAL",   // ID 22
};

void handlePing(ServoData *servoData) {
  // Create a stringstream to accumulate the output
  std::ostringstream output;

  for (size_t i = 1; i <= ServoNum; ++i) {
    if (servoData[i].pinged) {
      // Print "OK" for true responses
      output << std::left << std::setw(15) << green + "Servo " + std::to_string(i) << std::setw(15)
             << ServoNamesById[i] << " OK " << reset << std::endl;
    } else {
      // Print "WARNING" in yellow for false responses
      output << std::left << std::setw(15) << yellow + "Servo " + std::to_string(i) << std::setw(15)
             << ServoNamesById[i] << " WARNING: did not respond" + reset << std::endl;
    }
  }
  output << "PING ACK" << std::endl;
  // Output all accumulated text at once
  std::cout << output.str();
}

void handleQueryServoPositions(ServoData *servoData) {
  std::ostringstream output;
  output << "SERVOS_QUERY_POSITIONS: {";
  for (int i = 1; i <= ServoNum; i++) {
    if (servoData[i].pinged) {
      output << "\"" << i << "\"" << ": " << servoData[i].position
             << (i <= ServoNum - 1 ? ", " : "");
    }
  }
  output << "}" << std::endl;
  std::cout << output.str();
}

void handleQueryServoMoving(ServoData *servoData) {
  std::ostringstream output;
  output << "SERVOS_QUERY_MOVING: {";
  for (int i = 1; i <= ServoNum; i++) {
    if (servoData[i].pinged) {
      output << "\"" << i << "\"" << ": " << true << (i <= ServoNum - 1 ? ", " : ""); // TODO
                                                                                      // replace
                                                                                      // true
    }
  }
  output << "}" << std::endl;
  std::cout << output.str();
}

void handleQueryServoSpeed(ServoData *servoData) {
  std::ostringstream output;
  output << "SERVOS_QUERY_SPEED: {";
  for (int i = 1; i <= ServoNum; i++) {
    if (servoData[i].pinged) {
      output << "\"" << i << "\"" << ": " << servoData[i].velocity
             << (i <= ServoNum - 1 ? ", " : "");
    }
  }
  output << "}" << std::endl;
  std::cout << output.str();
}

void handleSetServoPositions(const std::string &command, DesiredSettings *desiredSettings) {
  std::istringstream iss(command);
  std::string commandType;
  iss >> commandType;

  std::vector<u8> servoIDs;
  std::vector<u16> values;

  std::string pair;
  while (iss >> pair) {
    auto equalPos = pair.find('=');
    if (equalPos == std::string::npos) {
      std::cerr << "ERROR: Malformed pair: " << pair << std::endl;
      continue;
    }

    try {
      u8 id = static_cast<u8>(std::stoi(pair.substr(0, equalPos)));
      u16 value = static_cast<u16>(std::stoi(pair.substr(equalPos + 1))); // was s16, TESTME

      if (id <= ServoNum) {
        desiredSettings[id].enabled = true;
        desiredSettings[id].targetPosition = value;
      }
    } catch (const std::invalid_argument &e) {
      std::cerr << "ERROR: Malformed pair: " << pair << std::endl;
      continue;
    }
  }
}

void handleSetServoSpeeds(const std::string &command, DesiredSettings *desiredSettings) {
  std::istringstream iss(command);
  std::string commandType;
  iss >> commandType;

  std::vector<u8> servoIDs;
  std::vector<u16> values;

  std::string pair;
  while (iss >> pair) {
    auto equalPos = pair.find('=');
    if (equalPos == std::string::npos) {
      std::cerr << "ERROR: Malformed pair: " << pair << std::endl;
      continue;
    }

    try {
      u8 id = static_cast<u8>(std::stoi(pair.substr(0, equalPos)));
      u16 value = static_cast<u16>(std::stoi(pair.substr(equalPos + 1)));

      if (id <= ServoNum) {
        desiredSettings[id].targetSpeed = value;
      }
    } catch (const std::invalid_argument &e) {
      std::cerr << "ERROR: Malformed pair: " << pair << std::endl;
      continue;
    }
  }
}

void handleSetServoTorque(const std::string &command, DesiredSettings *desiredSettings) {
  std::istringstream iss(command);
  std::string commandType;
  iss >> commandType;

  std::vector<u8> servoIDs;
  std::vector<u16> values;

  std::string pair;
  while (iss >> pair) {
    auto equalPos = pair.find('=');
    if (equalPos == std::string::npos) {
      std::cerr << "ERROR: Malformed pair: " << pair << std::endl;
      continue;
    }

    try {
      u8 id = static_cast<u8>(std::stoi(pair.substr(0, equalPos)));
      u16 value = static_cast<u16>(std::stoi(pair.substr(equalPos + 1)));

      if (id <= ServoNum) {
        desiredSettings[id].maxLoad = value;
      }
    } catch (const std::invalid_argument &e) {
      std::cerr << "ERROR: Malformed pair: " << pair << std::endl;
      continue;
    }
  }
}

void handleSetServoEnabled(const std::string &command, DesiredSettings *desiredSettings) {
  std::istringstream iss(command);
  std::string commandType;
  iss >> commandType;

  std::vector<u8> servoIDs;
  std::vector<u8> values;

  std::string pair;
  while (iss >> pair) {
    auto equalPos = pair.find('=');
    if (equalPos == std::string::npos) {
      std::cerr << "ERROR: Malformed pair: " << pair << std::endl;
      continue;
    }

    try {
      u8 id = static_cast<u8>(std::stoi(pair.substr(0, equalPos)));
      u8 value = static_cast<u8>(std::stoi(pair.substr(equalPos + 1)));

      if (id <= ServoNum) {
        desiredSettings[id].enabled = value ? true : false;
      }
    } catch (const std::invalid_argument &e) {
      std::cerr << "ERROR: Malformed pair: " << pair << std::endl;
      continue;
    }
  }
}