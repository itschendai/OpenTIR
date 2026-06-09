#pragma once

#include <Arduino.h>

#define PROTOCOL_LINE_BUF   128
#define PROTOCOL_MAX_ARGS   8
#define PROTOCOL_ARG_KEY_LEN  20
#define PROTOCOL_ARG_VAL_LEN  32
#define PROTOCOL_CMD_NAME_LEN 24

struct ProtocolArg {
  char key[PROTOCOL_ARG_KEY_LEN];
  char value[PROTOCOL_ARG_VAL_LEN];
};

struct ProtocolCommand {
  char name[PROTOCOL_CMD_NAME_LEN];
  uint8_t argCount;
  ProtocolArg args[PROTOCOL_MAX_ARGS];
};

typedef void (*ProtocolCommandHandler)(const ProtocolCommand &cmd);

void protocolPollSerial(ProtocolCommandHandler handler);
void protocolEmitAck(long cmdId, const char* command);
void protocolEmitError(long cmdId, const char* command, const char* code, const char* message);
void protocolPrintBool(bool value);
void protocolUpperInPlace(char* value);
bool protocolEqualsIgnoreCase(const char* a, const char* b);
const char* protocolFindArg(const ProtocolCommand &cmd, const char* key);
bool protocolParseLongValue(const char* text, long &value);
bool protocolParseFloatValue(const char* text, float &value);
long protocolParseCmdId(const ProtocolCommand &cmd);
bool protocolParseOptionalFloat(const ProtocolCommand &cmd, const char* key, float &value, bool &present);
bool protocolParseRequiredFloat(const ProtocolCommand &cmd, const char* key, float &value);
bool protocolParseRequiredText(const ProtocolCommand &cmd, const char* key, char* buffer, size_t bufferLen);
