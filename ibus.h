#pragma once
// Minimal, dependency-free parser for the FlySky iBus serial protocol
// as output by the FS-iA10B receiver (single wire, 115200 baud, 32-byte frames).

#include <Arduino.h>

#define IBUS_MAX_CHANNELS 14
#define IBUS_FRAME_LEN    32

class IBusReader {
public:
  // rxPin only (iBus is a one-way link from receiver to flight controller)
  void begin(HardwareSerial &serial, uint32_t baud, int rxPin);

  // Call every loop iteration; non-blocking, parses whatever bytes are available.
  void update();

  // Returns pulse width in microseconds (1000-2000 typical) for a channel.
  uint16_t channel(uint8_t index) const;

  // True if a valid frame has been received within timeoutMs.
  bool isReceiving(uint32_t timeoutMs = 500) const;

private:
  HardwareSerial *_serial = nullptr;
  uint8_t _buffer[IBUS_FRAME_LEN];
  uint8_t _bufferIndex = 0;
  uint16_t _channels[IBUS_MAX_CHANNELS];
  uint32_t _lastFrameMs = 0;

  void parseFrame();
};
