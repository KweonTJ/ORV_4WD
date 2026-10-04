// Runs the ACTUAL sketch state machine; hardware I/O is replaced by stubs.
// This cannot validate electrical timing or receiver radio-loss behavior.
#include <cassert>
#include <iostream>
#include "../firmware/orv_uno_integrated/orv_uno_integrated.ino"

void prepare(bool enabled = true) {
  testMillis = 1000; fault = false; Wire.fail = false;
  for (int i = 0; i < 4; ++i) rpm[i] = 0;
  halt();
  uint8_t config[54] = {};
  for (int i = 0; i < 4; ++i) {
    put16(config+2*i, 4320); put16(config+8+6*i, 100);
    config[32+i] = 100; config[40+i] = config[44+i] = 1;
  }
  put16(config+48, 3000); put16(config+50, 3000); put16(config+52, 300);
  handle(CONFIG, 1, config, 54);
  assert(Serial.tx[8] == 0);
  uint8_t remote[] = {uint8_t(enabled), 1, 2, 3, 4, 0xdc, 0x05}; // 15 RPM
  handle(REMOTE_CONFIG, 2, remote, 7);
  assert(Serial.tx[8] == 0);
  handle(HEARTBEAT, 3, nullptr, 0);
}

void startRemote() {
  remoteUpdate(true, 0, 128, 128, testMillis);
  remoteUpdate(true, Ps2Receiver::START | Ps2Receiver::L1, 128, 128, testMillis);
  assert(armed && remoteActive);
}

int main() {
  prepare();
  // Boot/reconnect with buttons already held must never arm.
  remoteUpdate(true, Ps2Receiver::START | Ps2Receiver::L1, 128, 128, testMillis);
  assert(!armed);
  remoteUpdate(true, 0, 128, 128, testMillis);
  remoteUpdate(true, Ps2Receiver::START | Ps2Receiver::L1, 128, 0, testMillis);
  assert(!armed); // neutral required
  startRemote();
  remoteUpdate(true, Ps2Receiver::L1, 128, 0, testMillis);
  for (float v : target) assert(v > 14.9 && v <= 15.0);
  remoteUpdate(true, Ps2Receiver::L1, 255, 128, testMillis);
  assert(target[0] > 0 && target[1] > 0 && target[2] < 0 && target[3] < 0);
  uint8_t armOn = 1;
  handle(ARM, 4, &armOn, 1);
  assert(Serial.tx[8] == 2 && remoteActive); // host cannot steal ownership
  uint8_t drive[9] = {1};
  handle(DRIVE, 5, drive, 9);
  assert(Serial.tx[8] != 0 && remoteActive && target[0] > 0);
  remoteUpdate(true, 0, 128, 128, testMillis);
  assert(!armed); // deadman release
  remoteUpdate(true, Ps2Receiver::START | Ps2Receiver::L1, 128, 128, testMillis);
  assert(!armed); // must see released controls again
  startRemote();
  remoteUpdate(false, Ps2Receiver::L1, 128, 0, testMillis);
  assert(!armed && !remoteConnected);
  remoteUpdate(true, Ps2Receiver::START | Ps2Receiver::L1, 128, 128, testMillis);
  assert(!armed); // reconnection cannot auto-arm

  prepare(false); // disabled remote must not arm
  remoteUpdate(true, 0, 128, 128, testMillis);
  remoteUpdate(true, Ps2Receiver::START | Ps2Receiver::L1, 128, 128, testMillis);
  assert(!armed);
  prepare(); startRemote();
  handle(STOP, 6, nullptr, 0);
  assert(!armed && remoteLatch);
  remoteUpdate(true, Ps2Receiver::START | Ps2Receiver::L1, 128, 128, testMillis);
  assert(!armed);

  // Heartbeat freshness is independent of fresh radio input.
  prepare(); startRemote(); testMillis += 301;
  remoteUpdate(true, Ps2Receiver::L1, 128, 0, testMillis);
  assert(!armed);
  prepare(); startRemote(); testMillis += 200;
  handle(HEARTBEAT, 3, nullptr, 0); // duplicate must not extend lease
  testMillis += 101; lastCommand = testMillis;
  checkWatchdogs(testMillis); assert(!armed);

  // Host driving unaffected by missing receiver. SELECT stops either source.
  prepare(); handle(ARM, 4, &armOn, 1); assert(armed && !remoteActive);
  remoteUpdate(false, 0, 128, 128, testMillis); assert(armed);
  remoteUpdate(true, 0, 128, 128, testMillis);
  remoteUpdate(true, Ps2Receiver::START | Ps2Receiver::L1, 128, 128, testMillis);
  assert(armed && !remoteActive);
  remoteUpdate(true, Ps2Receiver::SELECT, 128, 128, testMillis); assert(!armed);
  handle(ARM, 5, &armOn, 1); assert(armed);
  testMillis += 301; checkWatchdogs(testMillis); assert(!armed);

  // Calibration channel permutation is also applied to manual targets.
  prepare();
  uint8_t remap[] = {1, 4, 2, 1, 3, 0xdc, 0x05};
  handle(REMOTE_CONFIG, 4, remap, 7); handle(HEARTBEAT, 5, nullptr, 0);
  startRemote(); remoteUpdate(true, Ps2Receiver::L1, 255, 128, testMillis);
  assert(target[3] > 0 && target[1] > 0 && target[0] < 0 && target[2] < 0);
  publishState(testMillis);
  assert(Serial.tx.size() == 58 && Serial.tx[6] == 49 && (Serial.tx[11] & 0x68) == 0x68);
  assert(get16(Serial.tx.data()+56) == crc16(Serial.tx.data()+2, 54));
  Wire.fail = true; fault = true; testMillis += 20; control(testMillis); assert(!armed);

  // Packet checks refuse unplugged/digital/malformed controllers.
  uint8_t packet[21] = {0xff, 0x73, 0x5a, 0xff, 0xfb, 128, 128, 128, 0};
  assert(receiver.decode(packet) && receiver.buttons == Ps2Receiver::L1 && receiver.ly == 0);
  packet[1] = 0x41; assert(!receiver.decode(packet));
  packet[1] = 0x79; assert(receiver.decode(packet));
  packet[2] = 0; assert(!receiver.decode(packet));
  std::cout << "PASS: actual firmware arbitration, deadman, neutral/reconnect latch, host/radio timeouts, mapping, telemetry CRC and PS2 packet validation\n";
}
