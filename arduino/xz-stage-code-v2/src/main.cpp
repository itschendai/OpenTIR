#include "primitives.h"
#include "serial_protocol.h"

void setup() {
  primitivesSetup();
}

void loop() {
  protocolPollSerial(primitivesHandleCommand);
  primitivesUpdate();
}
