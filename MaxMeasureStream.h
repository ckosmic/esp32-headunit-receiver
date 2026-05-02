#pragma once;
#include <Arduino.h>
#include "AudioTools.h"

namespace audio_tools {

class MaxMeasureStream : public AudioStream {
public:
  MaxMeasureStream(AudioStream& out) : out(out) { }

  size_t write(const uint8_t *buffer, size_t size) override {
    int16_t* samples = (int16_t*)buffer;
    int count = size / sizeof(int16_t);

    int16_t maxSample = -1;

    for (int i = 0; i < count; i += 2) {
      if (abs(samples[i]) > maxSample)
        maxSample = abs(samples[i]);
      if (abs(samples[i + 1]) > maxSample)
        maxSample = abs(samples[i + 1]);
    }

    Serial.print("Max sample: ");
    Serial.println(maxSample);

    return out.write(buffer, size);
  }

  int available() override { return 0; }
  int read() override { return -1; }
  int peek() override { return -1; }
  void flush() override { out.flush(); }

private:
  AudioStream& out;
};

}