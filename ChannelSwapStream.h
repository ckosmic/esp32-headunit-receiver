#pragma once;
#include "AudioTools.h"

namespace audio_tools {

class ChannelSwapStream : public AudioStream {
public:
  ChannelSwapStream(AudioStream& out) : out(out) { }

  size_t write(const uint8_t *buffer, size_t size) override {
    int16_t* samples = (int16_t*)buffer;
    int count = size / sizeof(int16_t);

    for (int i = 0; i < count; i += 2) {
      int16_t tmp = samples[i];
      samples[i] = samples[i + 1];
      samples[i + 1] = tmp;
    }

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