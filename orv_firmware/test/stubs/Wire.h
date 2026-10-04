#pragma once
#include <stdint.h>
struct WireStub {
  bool fail = false;
  void begin() {}
  void setClock(unsigned long) {}
  void beginTransmission(uint8_t) {}
  void write(uint8_t) {}
  uint8_t endTransmission() { return fail ? 1 : 0; }
};
inline WireStub Wire;
