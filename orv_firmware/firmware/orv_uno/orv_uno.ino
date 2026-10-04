// ORV UNO firmware, protocol v1. Arduino UNO R3 / ATmega328P only.
// Pin/channel mapping verified against the supplied QGPMaker V5.3/V5.6 files.
// No servo support: PCA9685 shared carrier is configured to approximately 1 kHz.
#include <Arduino.h>
#include <Wire.h>
#include <util/atomic.h>
#include <avr/interrupt.h>
#include <math.h>

#if !defined(__AVR_ATmega328P__)
#error "Select Arduino UNO (ATmega328P). This firmware uses UNO-specific PCINT pins."
#endif

const uint8_t ADDRESS = 0x60;
const uint8_t IN_A[4] = {8, 10, 15, 13};
const uint8_t IN_B[4] = {9, 11, 14, 12};
const uint8_t CONFIG = 1, ARM = 2, DRIVE = 3, STOP = 4, RESET = 5, STATE = 128, ACK = 129;
const uint8_t CONFIG_SIZE = 54;
volatile uint32_t counts[4] = {0, 0, 0, 0};
volatile uint8_t phase[4];
const int8_t QUAD[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

struct Settings {
  uint16_t cpr[4];
  float kp[4], ki[4], kd[4];
  uint8_t maximum[4], minimum[4];
  int8_t motorSign[4], encoderSign[4];
  float maxRpm, acceleration;
  uint16_t watchdog;
};
Settings cfg;
bool armed = false, configured = false, fault = false;
uint16_t configSeq = 0, commandSeq = 0, armSeq = 0;
uint8_t controlMode = 1;
float target[4] = {0}, ramp[4] = {0}, rpm[4] = {0}, integral[4] = {0}, previousRpm[4] = {0};
int16_t output[4] = {0}, lastOutput[4] = {-32768, -32768, -32768, -32768};
uint32_t previousCounts[4] = {0};
uint32_t lastCommand = 0, lastControl = 0, lastState = 0, lastByte = 0;
uint8_t rx[73], used = 0;

uint16_t get16(const uint8_t *p) { return uint16_t(p[0]) | (uint16_t(p[1]) << 8); }
void put16(uint8_t *p, uint16_t v) { p[0] = v; p[1] = v >> 8; }
void put32(uint8_t *p, uint32_t v) { for (uint8_t i = 0; i < 4; ++i) p[i] = v >> (8*i); }
uint16_t crc16(const uint8_t *p, uint8_t length) {
  uint16_t crc = 0xffff;
  while (length--) {
    crc ^= uint16_t(*p++) << 8;
    for (uint8_t i = 0; i < 8; ++i) crc = crc & 0x8000 ? (crc << 1) ^ 0x1021 : crc << 1;
  }
  return crc;
}

inline void quadrature(uint8_t i, uint8_t next) {
  int8_t delta = QUAD[(phase[i] << 2) | next];
  counts[i] += delta;  // unsigned modulo arithmetic is defined at rollover
  phase[i] = next;
}
ISR(PCINT0_vect) {
  uint8_t pins = PINB;
  quadrature(0, pins & 3); // D8, D9
}
ISR(PCINT2_vect) {
  uint8_t pins = PIND;
  quadrature(1, (pins >> 6) & 3);                          // D6, D7
  quadrature(2, ((pins >> 3) & 1) | (((pins >> 2) & 1)<<1)); // D3, D2
  quadrature(3, ((pins >> 5) & 1) | (((pins >> 4) & 1)<<1)); // D5, D4
}
void snapshot(uint32_t *values) {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    for (uint8_t i = 0; i < 4; ++i) values[i] = counts[i];
  }
}

bool registerWrite(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(ADDRESS); Wire.write(reg); Wire.write(value);
  bool ok = Wire.endTransmission() == 0;
  if (!ok) fault = true;
  return ok;
}
bool pwmWrite(uint8_t channel, uint16_t value) {
  Wire.beginTransmission(ADDRESS);
  Wire.write(uint8_t(0x06 + channel*4));
  Wire.write(uint8_t(0)); Wire.write(uint8_t(0));
  Wire.write(uint8_t(value)); Wire.write(uint8_t(value >> 8));
  bool ok = Wire.endTransmission() == 0;
  if (!ok) fault = true;
  return ok;
}
void motorWrite(uint8_t i, int16_t value) {
  if (value == lastOutput[i]) return;
  // Both outputs low (coast) before changing direction. No dynamic braking.
  bool ok = pwmWrite(IN_A[i], 0);
  ok = pwmWrite(IN_B[i], 0) && ok;
  if (ok && value != 0) ok = pwmWrite(value > 0 ? IN_A[i] : IN_B[i], uint16_t(abs(value))*16);
  if (ok) lastOutput[i] = value;
}
void halt() {
  armed = false;
  for (uint8_t i = 0; i < 4; ++i) {
    target[i] = ramp[i] = integral[i] = 0;
    output[i] = 0;
    motorWrite(i, 0);
  }
}

void sendFrame(uint8_t kind, uint16_t seq, const uint8_t *payload, uint8_t length) {
  uint8_t data[73];
  data[0] = 'O'; data[1] = 'R'; data[2] = 1; data[3] = kind;
  put16(data+4, seq); data[6] = length;
  if (length) memcpy(data+7, payload, length);
  put16(data+7+length, crc16(data+2, length+5));
  Serial.write(data, length+9);
}
void acknowledge(uint8_t kind, uint16_t seq, uint8_t result) {
  uint8_t payload[2] = {kind, result};
  sendFrame(ACK, seq, payload, 2);
}

bool parseSettings(const uint8_t *p, Settings &candidate) {
  for (uint8_t i = 0; i < 4; ++i) {
    candidate.cpr[i] = get16(p+2*i);
    candidate.kp[i] = get16(p+8+6*i) / 100.0f;
    candidate.ki[i] = get16(p+10+6*i) / 100.0f;
    candidate.kd[i] = get16(p+12+6*i) / 100.0f;
    candidate.maximum[i] = p[32+i]; candidate.minimum[i] = p[36+i];
    candidate.motorSign[i] = int8_t(p[40+i]); candidate.encoderSign[i] = int8_t(p[44+i]);
    if (!candidate.cpr[i] || candidate.cpr[i] > 60000 ||
        candidate.kp[i] > 100 || candidate.ki[i] > 100 || candidate.kd[i] > 100 ||
        !candidate.maximum[i] || candidate.minimum[i] > candidate.maximum[i] ||
        (candidate.motorSign[i] != 1 && candidate.motorSign[i] != -1) ||
        (candidate.encoderSign[i] != 1 && candidate.encoderSign[i] != -1)) return false;
  }
  candidate.maxRpm = get16(p+48) / 100.0f;
  candidate.acceleration = get16(p+50) / 100.0f;
  candidate.watchdog = get16(p+52);
  return candidate.maxRpm >= 1 && candidate.maxRpm <= 150 &&
         candidate.acceleration >= 1 && candidate.acceleration <= 300 &&
         candidate.watchdog >= 100 && candidate.watchdog <= 1000;
}

void handle(uint8_t kind, uint16_t seq, const uint8_t *p, uint8_t length) {
  uint8_t result = 0;
  if (kind == STOP && length == 0) {
    halt();
  } else if (kind == CONFIG && length == CONFIG_SIZE && !armed) {
    Settings candidate;
    if (!parseSettings(p, candidate)) result = 1;
    else if (fault) result = 3; // A hardware fault requires restart and inspection.
    else {
      halt(); cfg = candidate; configured = true; configSeq = seq;
      snapshot(previousCounts);
      for (uint8_t i = 0; i < 4; ++i) rpm[i] = previousRpm[i] = 0;
      lastControl = millis();
    }
  } else if (kind == ARM && length == 1 && p[0] <= 1) {
    if (!p[0]) halt();
    else if (!configured || fault) result = 2;
    else if (armed && seq != armSeq) result = 2;
    else if (!armed) {
      halt(); armed = true; armSeq = seq; commandSeq = seq;
      lastCommand = millis(); controlMode = 1;
    }
  } else if (kind == DRIVE && length == 9 && armed && (p[0] == 1 || p[0] == 2)) {
    uint16_t difference = uint16_t(seq-commandSeq);
    if (difference == 0 || difference >= 32768) return; // stale or duplicate command
    bool valid = true;
    for (uint8_t i = 0; i < 4; ++i) {
      int16_t value = int16_t(get16(p+1+2*i));
      int16_t limit = p[0] == 1 ? int16_t(cfg.maxRpm*100+0.5f) : cfg.maximum[i];
      if (value < -limit || value > limit) valid = false;
    }
    if (!valid) { halt(); result = 1; }
    else {
      if (controlMode != p[0]) {
        for (uint8_t i = 0; i < 4; ++i) { integral[i] = 0; ramp[i] = 0; }
      }
      controlMode = p[0];
      for (uint8_t i = 0; i < 4; ++i)
        target[i] = int16_t(get16(p+1+2*i)) / (controlMode == 1 ? 100.0f : 1.0f);
      commandSeq = seq; lastCommand = millis();
      return; // periodic STATE is the drive acknowledgement
    }
  } else if (kind == RESET && length == 0 && !armed) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { for (uint8_t i = 0; i < 4; ++i) counts[i] = 0; }
    snapshot(previousCounts);
  } else result = 1;
  acknowledge(kind, seq, result);
}

void receiveBytes() {
  uint32_t now = millis();
  if (used && uint32_t(now-lastByte) > 100) used = 0;
  uint8_t budget = 64;
  while (budget-- && Serial.available()) {
    if (used >= sizeof(rx)) used = 0;
    rx[used++] = uint8_t(Serial.read()); lastByte = now;
    while (used >= 2) {
      if (rx[0] != 'O' || rx[1] != 'R') { memmove(rx, rx+1, --used); continue; }
      if (used < 7) break;
      if (rx[2] != 1 || rx[6] > 64) { memmove(rx, rx+1, --used); continue; }
      uint8_t total = rx[6]+9;
      if (used < total) break;
      if (crc16(rx+2, total-4) != get16(rx+total-2)) { memmove(rx, rx+1, --used); continue; }
      handle(rx[3], get16(rx+4), rx+7, rx[6]);
      used -= total; memmove(rx, rx+total, used);
    }
  }
}

void control(uint32_t now) {
  uint32_t elapsed = now-lastControl;
  if (elapsed < 20) return;
  lastControl = now;
  if (armed && elapsed > 100) { fault = true; halt(); }
  float dt = elapsed / 1000.0f;
  uint32_t current[4]; snapshot(current);
  for (uint8_t i = 0; i < 4; ++i) {
    int32_t ticks = int32_t(current[i]-previousCounts[i]);
    previousCounts[i] = current[i];
    rpm[i] = configured ? (float(ticks)*cfg.encoderSign[i]*60.0f/(cfg.cpr[i]*dt)) : 0;
    if (!armed) { output[i] = 0; previousRpm[i] = rpm[i]; continue; }
    float value;
    if (controlMode == 2) {
      value = target[i]; // PWM bench test, output limit still enforced
      integral[i] = 0;
    } else {
      float step = cfg.acceleration*dt;
      ramp[i] += constrain(target[i]-ramp[i], -step, step);
      if (fabs(ramp[i]) < 0.01f) { integral[i] = 0; value = 0; }
      else {
        float error = ramp[i]-rpm[i];
        float added = cfg.ki[i]*error*dt;
        float proposed = constrain(integral[i]+added, -float(cfg.maximum[i]), float(cfg.maximum[i]));
        // Feed-forward from the supplied 12 V / 107 RPM no-load rating, provisional.
        value = ramp[i]*(255.0f/107.0f)+cfg.kp[i]*error+proposed-cfg.kd[i]*(rpm[i]-previousRpm[i])/dt;
        if (fabs(value) <= cfg.maximum[i] || value*error < 0) integral[i] = proposed;
      }
    }
    previousRpm[i] = rpm[i];
    value = constrain(value, -float(cfg.maximum[i]), float(cfg.maximum[i]));
    if (fabs(value) > 0.01f && fabs(value) < cfg.minimum[i]) value = value > 0 ? cfg.minimum[i] : -float(cfg.minimum[i]);
    output[i] = int16_t(round(value));
    motorWrite(i, output[i]*cfg.motorSign[i]);
  }
  if (fault) halt();
}

void publishState(uint32_t now) {
  if (uint32_t(now-lastState) < 50) return;
  // Complete frame fits in an empty AVR TX buffer. Skip rather than block control.
  if (Serial.availableForWrite() < 50) return;
  lastState = now;
  uint8_t payload[41]; uint32_t current[4]; snapshot(current);
  put32(payload, now); payload[4] = uint8_t(armed) | (uint8_t(configured)<<1) | (uint8_t(fault)<<2);
  put16(payload+5, configSeq); put16(payload+7, commandSeq);
  for (uint8_t i = 0; i < 4; ++i) {
    uint32_t signedCount = configured && cfg.encoderSign[i] == -1 ? uint32_t(0)-current[i] : current[i];
    put32(payload+9+4*i, signedCount);
    put16(payload+25+2*i, uint16_t(int16_t(constrain(rpm[i]*100.0f, -32767.0f, 32767.0f))));
    put16(payload+33+2*i, uint16_t(output[i]));
  }
  sendFrame(STATE, 0, payload, sizeof(payload));
}

void setup() {
  Serial.begin(115200);
  Wire.begin(); Wire.setClock(400000);
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(2500, true);
#endif
  registerWrite(0x00, 0x10); // sleep
  registerWrite(0xfe, 5);    // 25 MHz / (4096 * 6), approximately 1017 Hz
  registerWrite(0x00, 0x20); // wake, auto increment
  delay(1);
  for (uint8_t ch = 0; ch < 16; ++ch) pwmWrite(ch, 0);
  for (uint8_t pin = 2; pin <= 9; ++pin) pinMode(pin, INPUT_PULLUP);
  uint8_t b = PINB, d = PIND;
  phase[0] = b&3; phase[1] = (d>>6)&3;
  phase[2] = ((d>>3)&1) | (((d>>2)&1)<<1);
  phase[3] = ((d>>5)&1) | (((d>>4)&1)<<1);
  PCIFR = _BV(PCIF0) | _BV(PCIF2);
  PCMSK0 = _BV(PCINT0) | _BV(PCINT1);
  PCMSK2 = 0xfc;
  PCICR |= _BV(PCIE0) | _BV(PCIE2);
  halt(); lastControl = millis();
}
void loop() {
  receiveBytes();
  uint32_t now = millis();
  if (armed && uint32_t(now-lastCommand) > cfg.watchdog) halt();
  control(now);
  publishState(now);
}
