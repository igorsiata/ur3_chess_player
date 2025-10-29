#include <Arduino.h>
#include <string.h>
#include "STSCTRL.h"


class CommandExecutor {
public:


  String executeCommand(String input) {
    String commandName = extractCommandName(input);
    int params[MAX_PARAMS];
    int paramCount = extractParameters(input, params);
    String outputMsg = runCommand(commandName, params, paramCount);
    return outputMsg;
  }

private:
  const int MAX_PARAMS = 12;
  const int NUMBER_OF_SERVOS = 2;
  const int CONSTRAINS[2][2] = {
    { 2300 ,2750 },
    { 1350, 1800 }
  };

  String extractCommandName(String input) {
    int spaceIndex = input.indexOf(' ');
    if (spaceIndex == -1) {
      return input;
    } else {
      return input.substring(0, spaceIndex);
    }
  }

  int extractParameters(String input, int params[]) {
    int paramCount = 0;
    int spaceIndex = input.indexOf(' ');

    while (spaceIndex != -1 && paramCount < MAX_PARAMS) {
      int nextSpace = input.indexOf(' ', spaceIndex + 1);
      if (nextSpace == -1) {
        params[paramCount++] = input.substring(spaceIndex + 1).toInt();
      } else {
        params[paramCount++] = input.substring(spaceIndex + 1, nextSpace).toInt();
      }
      spaceIndex = nextSpace;
    }
    return paramCount;
  }

  String runCommand(String commandName, int params[], int paramCount) {
    if (commandName == "SET_SERVO_POS") {
      return handleSetPos(params, paramCount);
    } else if (commandName == "SET_SERVO_POS_SYNC") {
      return handleSetPosSync(params, paramCount);
    } else if (commandName == "READ_SERVO_POS") {
      return handleReadPos(params, paramCount);
    } else if (commandName == "READ_SERVO_PARAMS") {
      return handleReadParams(params, paramCount);
    } else if (commandName == "READ_SERVO_SPEED") {
      return handleReadSpeed(params, paramCount);
    } else {
      return "ERROR: Unknown command";
    }
  }

  bool isServoIdCorrect(int servoID) {
    return (servoID >= 1 && servoID <= NUMBER_OF_SERVOS);
  }

  bool isPositionInConstrains(int servoID, int position) {
    return (position >= CONSTRAINS[servoID - 1][0] && position <= CONSTRAINS[servoID - 1][1]);
  }

  String handleSetPos(int params[], int paramCount) {
    String response = "";
    if (paramCount != 4) return "ERROR: Invalid parameters";

    int servoID = params[0];
    int angle = params[1];
    int speed = params[2];
    int acceleration = params[3];

    if (!isServoIdCorrect(servoID)) return "ERROR: Invalid servo ID";
    if (!isPositionInConstrains(servoID, angle)) return "ERROR: Position out of consrains";
    st.WritePosEx(servoID, angle, speed, acceleration);
    response = "ACK SET_SERVO " + String(servoID) + " " + String(angle);
    return response;
  }

  String handleSetPosSync(int params[], int paramCount) {

    if (paramCount != NUMBER_OF_SERVOS * 3) return "ERROR: Invalid parameters";

    byte servoIDList[NUMBER_OF_SERVOS];
    s16 servoPosList[NUMBER_OF_SERVOS] = { params[0], params[1]};
    u16 servoSpeedList[NUMBER_OF_SERVOS] = { params[2], params[3]};
    byte servoAccelerationList[NUMBER_OF_SERVOS] = { params[4], params[5]};

    for (byte i = 0; i < NUMBER_OF_SERVOS; i++) {
      servoIDList[i] = i + 1;
      servoPosList[i] = params[i];
      servoSpeedList[i] = params[i + NUMBER_OF_SERVOS];
      servoAccelerationList[i] = params[i + (2 * NUMBER_OF_SERVOS)];
      if (!isPositionInConstrains(i + 1, servoPosList[i])) return "ERROR: Position out of consrains";
    }
    st.SyncWritePosEx(servoIDList, 3, servoPosList, servoSpeedList, servoAccelerationList);
    return "ACK SET_SERVO_SYNC ";
  }

  String handleReadParams(int params[], int paramCount) {
    String response = "";
    if (paramCount != 0) return "ERROR: Invalid parameters";

    for (int servoID = 1; servoID <= NUMBER_OF_SERVOS; servoID++) {
      if (st.FeedBack(servoID) != -1) {
        int load = st.ReadLoad(-1);
        int voltage = st.ReadVoltage(-1);
        int current = st.ReadCurrent(-1);
        int temperature = st.ReadTemper(-1);

        response += "ID:" + String(servoID) + ",L:" + String(load, 4) + ",V:" + String(load, 2) + ",C:" + String(current, 4) + ",T:" + String(temperature, 4) + " ";
      }
    }
    return response;
  }

  String handleReadPos(int params[], int paramCount) {
    String response = "";
    if (paramCount != 0) return "ERROR: Invalid parameters";

    for (int servoID = 1; servoID <= NUMBER_OF_SERVOS; servoID++) {
      if (st.FeedBack(servoID) != -1) {
        int pos = st.ReadPos(-1);

        response += "ID:" + String(servoID) + ",P:" + String(pos) + " ";
      }
    }
    return response;
  }

  String handleReadSpeed(int params[], int paramCount) {
    String response = "";
    if (paramCount != 0) return "ERROR: Invalid parameters";

    for (int servoID = 1; servoID <= NUMBER_OF_SERVOS; servoID++) {
      if (st.FeedBack(servoID) != -1) {
        int speed = st.ReadSpeed(-1);

        response += "ID:" + String(servoID) + ",S:" + String(speed) + " ";
      }
    }
    return response;
  }
};
