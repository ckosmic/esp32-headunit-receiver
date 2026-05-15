#ifndef SMARTKNOB_H
#define SMARTKNOB_H

template <typename T> T CLAMP(const T& value, const T& low, const T& high)  {
  return value < low ? low : (value > high ? high : value); 
}

class SmartKnob {
private:
  int32_t prev_position = 0;
  float prev_shaft_angle = 0;
  float angle_offset = 0;

  void updateDetent();

  void (*onPositionChanged)(int32_t position) = nullptr;
  void (*onAngleChanged)(float angle) = nullptr;
  void (*onInit)() = nullptr;
public:
  int min_position = 0;
  int max_position = -1;
  float position_width_radians = radians(20);
  int detent_positions_count = 0;
  int32_t detent_positions[5] = {0, 0, 0, 0, 0};

  float snap_point = 0.5f;
  float snap_point_bias = 0;

  int32_t current_position = 0;
  float current_detent_center = 0;

  float latest_sub_position_unit = 0;
  float endstop_strength_unit = 1.0f;
  float detent_strength_unit = 1.0f;

  void init();
  void update();

  void playHaptic(float strength=1.5);
  void resetAngle();

  void setOnPositionChanged(void(*callback)(int32_t)) {
    onPositionChanged = callback;
  }
  void setOnAngleChanged(void(*callback)(float)) {
    onAngleChanged = callback;
  }
  void setOnInit(void(*callback)()) {
    onInit = callback;
  }

  void setPosition(int32_t position) {
    current_position = position;
    prev_position = position;
  }
};

#endif