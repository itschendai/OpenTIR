#pragma once

#include "serial_protocol.h"

void primitivesSetup();
void primitivesUpdate();
void primitivesHandleCommand(const ProtocolCommand &cmd);
