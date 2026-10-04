#pragma once
#include <Arduino.h>

// QGPMaker supplied example: CLK=13, CMD=11, ATT=10, DAT=12.
// Bounded, single-transaction polling: encoder interrupts remain enabled.
class Ps2Receiver {
 public:
  enum { SELECT = 0x0001, START = 0x0008, L1 = 0x0400 };
  uint16_t buttons = 0;
  uint8_t lx = 128, ly = 128;

  void begin() {
    digitalWrite(13, HIGH); digitalWrite(11, HIGH); digitalWrite(10, HIGH);
    pinMode(13, OUTPUT); pinMode(11, OUTPUT); pinMode(10, OUTPUT);
    pinMode(12, INPUT_PULLUP);
  }

  // Called every 20 ms. Analog-mode negotiation advances one step per call.
  bool poll() {
    uint8_t reply[21];
    if (stage) {
      const uint8_t enter[] = {1, 0x43, 0, 1, 0};
      const uint8_t analog[] = {1, 0x44, 0, 1, 3, 0, 0, 0, 0};
      const uint8_t leave[] = {1, 0x43, 0, 0, 0x5a, 0x5a, 0x5a, 0x5a, 0x5a};
      if (stage == 1) exchange(enter, sizeof(enter), reply);
      if (stage == 2) exchange(analog, sizeof(analog), reply);
      if (stage == 3) exchange(leave, sizeof(leave), reply);
      stage = stage == 3 ? 0 : stage + 1;
      return false;
    }
    const uint8_t request[] = {1, 0x42, 0, 0, 0, 0, 0, 0, 0};
    exchange(request, sizeof(request), reply, true);
    if (!decode(reply)) { stage = 1; return false; }
    return true;
  }

  bool decode(const uint8_t *reply) {
    if (reply[0] != 0xff || (reply[1] != 0x73 && reply[1] != 0x79) || reply[2] != 0x5a)
      return false;
    buttons = ~(uint16_t(reply[3]) | (uint16_t(reply[4]) << 8));
    lx = reply[7]; ly = reply[8];
    return true;
  }

 private:
  uint8_t stage = 0;
  uint8_t transfer(uint8_t out) {
    uint8_t in = 0;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      digitalWrite(11, (out >> bit) & 1);
      digitalWrite(13, LOW);
      delayMicroseconds(4);
      if (digitalRead(12)) in |= uint8_t(1 << bit);
      digitalWrite(13, HIGH);
      delayMicroseconds(4);
    }
    digitalWrite(11, HIGH);
    delayMicroseconds(3);
    return in;
  }
  void exchange(const uint8_t *command, uint8_t size, uint8_t *reply, bool poll = false) {
    digitalWrite(10, LOW);
    delayMicroseconds(3);
    for (uint8_t i = 0; i < size; ++i) reply[i] = transfer(command[i]);
    if (poll && reply[1] == 0x79)
      for (uint8_t i = size; i < 21; ++i) reply[i] = transfer(0);
    digitalWrite(10, HIGH);
  }
};
