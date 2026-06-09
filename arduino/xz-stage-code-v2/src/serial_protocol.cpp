#include "serial_protocol.h"

#include <ctype.h>
#include <stdlib.h>
#include <string.h>

static char lineBuf[PROTOCOL_LINE_BUF];
static uint8_t lineLen = 0;

void protocolUpperInPlace(char* value) {
  while (*value != '\0') {
    *value = (char)toupper((unsigned char)*value);
    value++;
  }
}

bool protocolEqualsIgnoreCase(const char* a, const char* b) {
  while (*a != '\0' && *b != '\0') {
    if (toupper((unsigned char)*a) != toupper((unsigned char)*b))
      return false;
    a++;
    b++;
  }
  return *a == '\0' && *b == '\0';
}

void protocolPrintBool(bool value) {
  Serial.print(value ? F("true") : F("false"));
}

void protocolEmitAck(long cmdId, const char* command) {
  Serial.print(F("ACK "));
  Serial.print(cmdId);
  Serial.print(' ');
  Serial.println(command);
}

void protocolEmitError(long cmdId, const char* command, const char* code, const char* message) {
  Serial.print(F("ERR "));
  Serial.print(cmdId);
  Serial.print(' ');
  Serial.print(command);
  Serial.print(F(" code="));
  Serial.print(code);
  Serial.print(F(" message=\""));
  Serial.print(message);
  Serial.println('"');
}

static bool parseCommandLine(char* line, ProtocolCommand &cmd) {
  cmd.name[0] = '\0';
  cmd.argCount = 0;

  char* context = NULL;
  char* token = strtok_r(line, " \t", &context);
  if (token == NULL)
    return false;

  strncpy(cmd.name, token, sizeof(cmd.name) - 1);
  cmd.name[sizeof(cmd.name) - 1] = '\0';
  protocolUpperInPlace(cmd.name);

  while ((token = strtok_r(NULL, " \t", &context)) != NULL) {
    if (cmd.argCount >= PROTOCOL_MAX_ARGS)
      return false;
    char* equals = strchr(token, '=');
    if (equals == NULL || equals == token || equals[1] == '\0')
      return false;
    *equals = '\0';
    strncpy(cmd.args[cmd.argCount].key, token, PROTOCOL_ARG_KEY_LEN - 1);
    cmd.args[cmd.argCount].key[PROTOCOL_ARG_KEY_LEN - 1] = '\0';
    strncpy(cmd.args[cmd.argCount].value, equals + 1, PROTOCOL_ARG_VAL_LEN - 1);
    cmd.args[cmd.argCount].value[PROTOCOL_ARG_VAL_LEN - 1] = '\0';
    protocolUpperInPlace(cmd.args[cmd.argCount].key);
    cmd.argCount++;
  }

  return true;
}

const char* protocolFindArg(const ProtocolCommand &cmd, const char* key) {
  for (uint8_t i = 0; i < cmd.argCount; i++) {
    if (protocolEqualsIgnoreCase(cmd.args[i].key, key))
      return cmd.args[i].value;
  }
  return NULL;
}

bool protocolParseLongValue(const char* text, long &value) {
  char* end = NULL;
  long parsed = strtol(text, &end, 10);
  if (end == text || *end != '\0')
    return false;
  value = parsed;
  return true;
}

bool protocolParseFloatValue(const char* text, float &value) {
  char* end = NULL;
  value = (float)strtod(text, &end);
  if (end == text || *end != '\0')
    return false;
  return true;
}

long protocolParseCmdId(const ProtocolCommand &cmd) {
  const char* value = protocolFindArg(cmd, "CMD_ID");
  long cmdId = 0;
  if (value != NULL)
    protocolParseLongValue(value, cmdId);
  return cmdId;
}

bool protocolParseOptionalFloat(const ProtocolCommand &cmd, const char* key, float &value, bool &present) {
  present = false;
  const char* raw = protocolFindArg(cmd, key);
  if (raw == NULL)
    return true;
  if (!protocolParseFloatValue(raw, value))
    return false;
  present = true;
  return true;
}

bool protocolParseRequiredFloat(const ProtocolCommand &cmd, const char* key, float &value) {
  const char* raw = protocolFindArg(cmd, key);
  if (raw == NULL)
    return false;
  return protocolParseFloatValue(raw, value);
}

bool protocolParseRequiredText(const ProtocolCommand &cmd, const char* key, char* buffer, size_t bufferLen) {
  const char* raw = protocolFindArg(cmd, key);
  if (raw == NULL)
    return false;
  strncpy(buffer, raw, bufferLen - 1);
  buffer[bufferLen - 1] = '\0';
  protocolUpperInPlace(buffer);
  return true;
}

static void processLine(ProtocolCommandHandler handler) {
  if (lineLen == 0)
    return;

  lineBuf[lineLen] = '\0';
  ProtocolCommand cmd;
  if (!parseCommandLine(lineBuf, cmd)) {
    protocolEmitError(0, "UNKNOWN", "INVALID_ARG", "Malformed command line");
    return;
  }

  handler(cmd);
}

void protocolPollSerial(ProtocolCommandHandler handler) {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      if (lineLen > 0) {
        processLine(handler);
        lineLen = 0;
      }
      continue;
    }

    if (lineLen < (PROTOCOL_LINE_BUF - 1)) {
      lineBuf[lineLen++] = c;
    } else {
      lineLen = 0;
      protocolEmitError(0, "UNKNOWN", "INVALID_ARG", "Command line too long");
    }
  }
}
