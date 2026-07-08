#include "ibus.h"

void IBusReader::begin(HardwareSerial &serial, uint32_t baud, int rxPin) {
  _serial = &serial;
  _serial->begin(baud, SERIAL_8N1, rxPin, -1); // RX only, TX unused
  for (uint8_t i = 0; i < IBUS_MAX_CHANNELS; i++) _channels[i] = 1500; // neutral default until first frame
}

void IBusReader::update() {
  if (!_serial) return;

  while (_serial->available()) {
    uint8_t b = _serial->read();

    // A valid iBus frame starts with 0x20 (length=32) then 0x40 (command).
    if (_bufferIndex == 0 && b != 0x20) continue;
    if (_bufferIndex == 1 && b != 0x40) { _bufferIndex = 0; continue; }

    _buffer[_bufferIndex++] = b;

    if (_bufferIndex >= IBUS_FRAME_LEN) {
      parseFrame();
      _bufferIndex = 0;
    }
  }
}

void IBusReader::parseFrame() {
  // Checksum: 0xFFFF - (sum of all preceding bytes), little-endian at the end.
  uint16_t sum = 0;
  for (uint8_t i = 0; i < IBUS_FRAME_LEN - 2; i++) sum += _buffer[i];
  uint16_t checksum = _buffer[30] | (_buffer[31] << 8);
  if ((uint16_t)(0xFFFF - sum) != checksum) return; // corrupt frame — drop it

  for (uint8_t ch = 0; ch < IBUS_MAX_CHANNELS; ch++) {
    uint8_t lo = _buffer[2 + ch * 2];
    uint8_t hi = _buffer[3 + ch * 2];
    _channels[ch] = lo | (hi << 8);
  }
  _lastFrameMs = millis();
}

uint16_t IBusReader::channel(uint8_t index) const {
  if (index >= IBUS_MAX_CHANNELS) return 1500;
  return _channels[index];
}

bool IBusReader::isReceiving(uint32_t timeoutMs) const {
  return (millis() - _lastFrameMs) < timeoutMs;
}
