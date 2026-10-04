#pragma once
#include <stdint.h>
#include <stddef.h>
#include <cstring>
#include <cmath>
#include <cstdlib>
#include <vector>
using std::abs;
#define LOW 0
#define HIGH 1
#define INPUT_PULLUP 2
#define OUTPUT 3
#define _BV(x) (1U << (x))
enum { PCIF0=0, PCIF2=2, PCINT0=0, PCINT1=1, PCIE0=0, PCIE2=2 };
inline uint8_t PINB=0, PIND=0, PCIFR=0, PCMSK0=0, PCMSK2=0, PCICR=0;
inline uint32_t testMillis=0;
inline uint32_t millis() { return testMillis; }
inline void delay(uint32_t ms) { testMillis += ms; }
inline void delayMicroseconds(unsigned int) {}
inline void pinMode(uint8_t, uint8_t) {}
inline void digitalWrite(uint8_t, uint8_t) {}
inline int digitalRead(uint8_t) { return 1; }
template <class A, class B> inline auto min(A a, B b) { return a < b ? a : b; }
template <class A, class B> inline auto max(A a, B b) { return a > b ? a : b; }
template <class T> inline T constrain(T v, T lo, T hi) { return min(max(v, lo), hi); }
struct SerialStub {
  std::vector<uint8_t> tx;
  void begin(unsigned long) {}
  int available() { return 0; }
  int availableForWrite() { return 63; }
  int read() { return -1; }
  void write(const uint8_t *p, size_t length) { tx.assign(p, p + length); }
};
inline SerialStub Serial;
