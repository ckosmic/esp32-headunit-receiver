#include <SimpleFOC.h>
#include "smartknob.h"

#define FOC_PID_P 1
#define FOC_PID_I 0
#define FOC_PID_D 0.148
#define FOC_PID_OUTPUT_RAMP 1000
#define FOC_PID_LIMIT 3

#define FOC_VOLTAGE_LIMIT 3
#define FOC_LPF 0

#define MOTOR_POLE_PAIRS 7
#define MOTOR_PHASE_RESISTANCE 2.3
#define MOTOR_KV_RATING 220

#define MOTOR_PWM_1 25
#define MOTOR_PWM_2 33
#define MOTOR_PWM_3 32
#define MOTOR_EN 15

#define ZERO_ELECTRIC_OFFSET 7.34f;

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS, MOTOR_PHASE_RESISTANCE, MOTOR_KV_RATING);
BLDCDriver3PWM driver = BLDCDriver3PWM(MOTOR_PWM_1, MOTOR_PWM_2, MOTOR_PWM_3, MOTOR_EN);

static const float DEAD_ZONE_DETENT_PERCENT = 0.2;
static const float DEAD_ZONE_RAD = 1 * _PI / 180;

static const float IDLE_VELOCITY_EWMA_ALPHA = 0.001;
static const float IDLE_VELOCITY_RAD_PER_SEC = 0.05;
static const uint32_t IDLE_CORRECTION_DELAY_MILLIS = 500;
static const float IDLE_CORRECTION_MAX_ANGLE_RAD = 5 * PI / 180;
static const float IDLE_CORRECTION_RATE_ALPHA = 0.0005;

float idle_check_velocity_ewma = 0;
uint32_t last_idle_start = 0;

unsigned long init_time = 0;

void SmartKnob::updateDetent() {
  // If we are not moving and we're close to the center (but not exactly there), slowly adjust the centerpoint to match the current position
  idle_check_velocity_ewma = motor.shaft_velocity * IDLE_VELOCITY_EWMA_ALPHA + idle_check_velocity_ewma * (1 - IDLE_VELOCITY_EWMA_ALPHA);
  if (fabsf(idle_check_velocity_ewma) > IDLE_VELOCITY_RAD_PER_SEC) {
    last_idle_start = 0;
  } else {
    if (last_idle_start == 0) {
      last_idle_start = millis();
    }
  }
  if (last_idle_start > 0 && millis() - last_idle_start > IDLE_CORRECTION_DELAY_MILLIS && fabsf(motor.shaft_angle - current_detent_center) < IDLE_CORRECTION_MAX_ANGLE_RAD) {
    current_detent_center = motor.shaft_angle * IDLE_CORRECTION_RATE_ALPHA + current_detent_center * (1 - IDLE_CORRECTION_RATE_ALPHA);
  }

  // Check where we are relative to the current nearest detent; update our position if we've moved far enough to snap to another detent
  float angle_to_detent_center = motor.shaft_angle - current_detent_center;

  float snap_point_radians = position_width_radians * snap_point;
  float bias_radians = position_width_radians * snap_point_bias;
  float snap_point_radians_decrease = snap_point_radians + (current_position <= 0 ? bias_radians : -bias_radians);
  float snap_point_radians_increase = -snap_point_radians + (current_position >= 0 ? -bias_radians : bias_radians); 

  int32_t num_positions = max_position - min_position + 1;
  if (angle_to_detent_center > snap_point_radians_decrease && (num_positions <= 0 || current_position > min_position)) {
    current_detent_center += position_width_radians;
    angle_to_detent_center -= position_width_radians;
    current_position--;
  } else if (angle_to_detent_center < snap_point_radians_increase && (num_positions <= 0 || current_position < max_position)) {
    current_detent_center -= position_width_radians;
    angle_to_detent_center += position_width_radians;
    current_position++;
  }

  latest_sub_position_unit = -angle_to_detent_center / position_width_radians;

  float dead_zone_adjustment = CLAMP(
    angle_to_detent_center,
    fmaxf(-position_width_radians*DEAD_ZONE_DETENT_PERCENT, -DEAD_ZONE_RAD),
    fminf(position_width_radians*DEAD_ZONE_DETENT_PERCENT, DEAD_ZONE_RAD));

  bool out_of_bounds = num_positions > 0 && ((angle_to_detent_center > 0 && current_position == min_position) || (angle_to_detent_center < 0 && current_position == max_position));
  motor.PID_velocity.limit = 10; //out_of_bounds ? 10 : 3;
  motor.PID_velocity.P = out_of_bounds ? endstop_strength_unit * 4 : detent_strength_unit * 4;


  // Apply motor torque based on our angle to the nearest detent (detent strength, etc is handled by the PID_velocity parameters)
  if (fabsf(motor.shaft_velocity) > 60) {
    // Don't apply torque if velocity is too high (helps avoid positive feedback loop/runaway)
    motor.move(0);
  } else {
    float input = -angle_to_detent_center + dead_zone_adjustment;
    if (!out_of_bounds && detent_positions_count > 0) {
      bool in_detent = false;
      for (uint8_t i = 0; i < detent_positions_count; i++) {
        if (detent_positions[i] == current_position) {
          in_detent = true;
          break;
        }
      }
      if (!in_detent) {
        input = 0;
      }
    }
    float torque = motor.PID_velocity(input);
    motor.move(torque);
  }
}

void SmartKnob::playHaptic(float strength) {
  if (motor.motor_status != FOCMotorStatus::motor_ready) return;
  motor.move(strength);
  for (uint8_t i = 0; i < 3; i++) {
    motor.loopFOC();
    delay(1);
  }
  motor.move(-strength);
  for (uint8_t i = 0; i < 3; i++) {
    motor.loopFOC();
    delay(1);
  }
  motor.move(0);
  motor.loopFOC();
}

char buf_[256];
void log(const char* msg) {
  Serial.println(msg);
}

void calibrate() {
    // SimpleFOC is supposed to be able to determine this automatically (if you omit params to initFOC), but
    // it seems to have a bug (or I've misconfigured it) that gets both the offset and direction very wrong!
    // So this value is based on experimentation.
    // TODO: dig into SimpleFOC calibration and find/fix the issue

    log("\n\n\nStarting calibration, please DO NOT TOUCH MOTOR until complete!");
    delay(1000);

    motor.controller = MotionControlType::angle_openloop;
    motor.pole_pairs = 1;
    motor.zero_electric_angle = 0;
    motor.sensor_direction = Direction::CCW;
    motor.initFOC();

    float a = 0;

    motor.voltage_limit = FOC_VOLTAGE_LIMIT;
    motor.move(a);

    // #### Determine direction motor rotates relative to angle sensor
    for (uint8_t i = 0; i < 200; i++) {
        sensor.update();
        motor.move(a);
        delay(1);
    }
    float start_sensor = sensor.getAngle();

    for (; a < 3 * _2PI; a += 0.01) {
        sensor.update();
        motor.move(a);
        delay(1);
    }

    for (uint8_t i = 0; i < 200; i++) {
        sensor.update();
        delay(1);
    }
    float end_sensor = sensor.getAngle();


    motor.voltage_limit = 0;
    motor.move(a);

    log("");

    float movement_angle = fabsf(end_sensor - start_sensor);
    if (movement_angle < radians(30) || movement_angle > radians(180)) {
        snprintf(buf_, sizeof(buf_), "ERROR! Unexpected sensor change: start=%.2f end=%.2f", start_sensor, end_sensor);
        log(buf_);
        return;
    }

    log("Sensor measures positive for positive motor rotation:");
    if (end_sensor > start_sensor) {
        log("YES, Direction=CW");
        motor.zero_electric_angle = 0;
        motor.sensor_direction = Direction::CW;
        motor.initFOC();
    } else {
        log("NO, Direction=CCW");
        motor.zero_electric_angle = 0;
        motor.sensor_direction = Direction::CCW;
        motor.initFOC();
    }
    snprintf(buf_, sizeof(buf_), "  (start was %.1f, end was %.1f)", start_sensor, end_sensor);
    log(buf_);


    // #### Determine pole-pairs
    // Rotate 20 electrical revolutions and measure mechanical angle traveled, to calculate pole-pairs
    uint8_t electrical_revolutions = 20;
    snprintf(buf_, sizeof(buf_), "Going to measure %d electrical revolutions...", electrical_revolutions);
    log(buf_);
    motor.voltage_limit = FOC_VOLTAGE_LIMIT;
    motor.move(a);
    log("Going to electrical zero...");
    float destination = a + _2PI;
    for (; a < destination; a += 0.03) {
        sensor.update();
        motor.move(a);
        delay(1);
    }
    log("pause..."); // Let momentum settle...
    for (uint16_t i = 0; i < 1000; i++) {
        sensor.update();
        delay(1);
    }
    log("Measuring...");

    start_sensor = motor.sensor_direction * sensor.getAngle();
    destination = a + electrical_revolutions * _2PI;
    for (; a < destination; a += 0.03) {
        sensor.update();
        motor.move(a);
        delay(1);
    }
    for (uint16_t i = 0; i < 1000; i++) {
        sensor.update();
        motor.move(a);
        delay(1);
    }
    end_sensor = motor.sensor_direction * sensor.getAngle();
    motor.voltage_limit = 0;
    motor.move(a);

    if (fabsf(motor.shaft_angle - motor.target) > 1 * PI / 180) {
        log("ERROR: motor did not reach target!");
        while(1) {}
    }

    float electrical_per_mechanical = electrical_revolutions * _2PI / (end_sensor - start_sensor);
    snprintf(buf_, sizeof(buf_), "Electrical angle / mechanical angle (i.e. pole pairs) = %.2f", electrical_per_mechanical);
    log(buf_);

    if (electrical_per_mechanical < 3 || electrical_per_mechanical > 12) {
        snprintf(buf_, sizeof(buf_), "ERROR! Unexpected calculated pole pairs: %.2f", electrical_per_mechanical);
        log(buf_);
        return;
    }

    int measured_pole_pairs = (int)round(electrical_per_mechanical);
    snprintf(buf_, sizeof(buf_), "Pole pairs set to %d", measured_pole_pairs);
    log(buf_);

    delay(1000);


    // #### Determine mechanical offset to electrical zero
    // Measure mechanical angle at every electrical zero for several revolutions
    motor.voltage_limit = FOC_VOLTAGE_LIMIT;
    motor.move(a);
    float offset_x = 0;
    float offset_y = 0;
    float destination1 = (floor(a / _2PI) + measured_pole_pairs / 2.) * _2PI;
    float destination2 = (floor(a / _2PI)) * _2PI;
    for (; a < destination1; a += 0.4) {
        motor.move(a);
        delay(100);
        for (uint8_t i = 0; i < 100; i++) {
            sensor.update();
            delay(1);
        }
        float real_electrical_angle = _normalizeAngle(a);
        float measured_electrical_angle = _normalizeAngle( (float)(motor.sensor_direction * measured_pole_pairs) * sensor.getMechanicalAngle()  - 0);

        float offset_angle = measured_electrical_angle - real_electrical_angle;
        offset_x += cosf(offset_angle);
        offset_y += sinf(offset_angle);

        snprintf(buf_, sizeof(buf_), "%.2f, %.2f, %.2f", degrees(real_electrical_angle), degrees(measured_electrical_angle), degrees(_normalizeAngle(offset_angle)));
        log(buf_);
    }
    for (; a > destination2; a -= 0.4) {
        motor.move(a);
        delay(100);
        for (uint8_t i = 0; i < 100; i++) {
            sensor.update();
            delay(1);
        }
        float real_electrical_angle = _normalizeAngle(a);
        float measured_electrical_angle = _normalizeAngle( (float)(motor.sensor_direction * measured_pole_pairs) * sensor.getMechanicalAngle()  - 0);

        float offset_angle = measured_electrical_angle - real_electrical_angle;
        offset_x += cosf(offset_angle);
        offset_y += sinf(offset_angle);

        snprintf(buf_, sizeof(buf_), "%.2f, %.2f, %.2f", degrees(real_electrical_angle), degrees(measured_electrical_angle), degrees(_normalizeAngle(offset_angle)));
        log(buf_);
    }
    motor.voltage_limit = 0;
    motor.move(a);

    float avg_offset_angle = atan2f(offset_y, offset_x);


    // #### Apply settings
    motor.pole_pairs = measured_pole_pairs;
    motor.zero_electric_angle = avg_offset_angle + _3PI_2;
    motor.voltage_limit = FOC_VOLTAGE_LIMIT;
    motor.controller = MotionControlType::torque;

    log("");
    log("RESULTS:");
    snprintf(buf_, sizeof(buf_), "  ZERO_ELECTRICAL_OFFSET: %.2f", motor.zero_electric_angle);
    log(buf_);
    if (motor.sensor_direction == Direction::CW) {
        log("  FOC_DIRECTION: Direction::CW");
    } else {
        log("  FOC_DIRECTION: Direction::CCW");
    }
    snprintf(buf_, sizeof(buf_), "  MOTOR_POLE_PAIRS: %d", motor.pole_pairs);
    log(buf_);

    /*log("");
    log("Saving to persistent configuration...");
    PB_MotorCalibration calibration = {
        .calibrated = true,
        .zero_electrical_offset = motor.zero_electric_angle,
        .direction_cw = motor.sensor_direction == Direction::CW,
        .pole_pairs = motor.pole_pairs,
    };
    if (configuration_.setMotorCalibrationAndSave(calibration)) {
        log("Success!");
    }*/
}

void SmartKnob::resetAngle() {
  angle_offset = sensor.getAngle();
  current_detent_center = motor.shaft_angle;
  current_position = 0;
  //prev_shaft_angle = sensor.getAngle();
  //prev_position = current_position;
}


void SmartKnob::init() {

  // use monitoring with serial 
  //Serial.begin(115200);
  // enable more verbose output for debugging
  // comment out if not needed
  //SimpleFOCDebug::enable(&Serial);

  // initialise magnetic sensor hardware
  sensor.init();
  // link the motor to the sensor
  motor.linkSensor(&sensor);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);

  // choose FOC modulation (optional)
  //motor.foc_modulation = FOCModulationType::SinePWM;

  // set motion control loop to be used
  motor.controller = MotionControlType::torque;

  // maximal voltage to be set to the motor
  motor.voltage_limit = FOC_VOLTAGE_LIMIT; //12;
  // maximal velocity of the position control
  motor.velocity_limit = 10000;

  // contoller configuration
  // default parameters in defaults.h

  // velocity PI controller parameters
  motor.PID_velocity.P = FOC_PID_P; //0.2f;
  motor.PID_velocity.I = FOC_PID_I; //10.0f;
  motor.PID_velocity.D = FOC_PID_D; //0.001f;

  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor.PID_velocity.output_ramp = FOC_PID_OUTPUT_RAMP; //1000; // 1000
  motor.PID_velocity.limit = FOC_PID_LIMIT;

  #ifdef FOC_LPF
  motor.LPF_angle.Tf = FOC_LPF;
  motor.LPF_velocity.Tf = 0.05f;
  #endif

  // velocity low pass filtering time constant
  // the lower the less filtered
  

  /*
  // angle PID controller
  // default P=20
  motor.P_angle.P = 30.0f; // 6 20  80 (faible amortis l'arrêt)
  motor.P_angle.I = 0;  // 0 usually only P controller is enough
  motor.P_angle.D = 0;  // 0 usually only P controller is enough
  // acceleration control using output ramp
  // this variable is in rad/s^2 and sets the limit of acceleration
  motor.P_angle.output_ramp = 1e6; // 10000 // default 1e6 rad/s^2

  motor.LPF_angle.Tf = 0.003f;*/

  //motor.sensor_offset = 0;
  motor.current_limit = 1.0f;
  
  // comment out if not needed
  //motor.useMonitoring(Serial);

  motor.zero_electric_angle = ZERO_ELECTRIC_OFFSET;
  motor.sensor_direction = Direction::CCW;

  // initialize motor
  motor.init();
  // align sensor and start FOC
  motor.initFOC();

  motor.sensor_offset = -motor.shaft_angle;


  const float derivative_lower_strength = detent_strength_unit * 0.08;
  const float derivative_upper_strength = detent_strength_unit * 0.02;
  const float derivative_position_width_lower = radians(3);
  const float derivative_position_width_upper = radians(8);
  const float raw = derivative_lower_strength + (derivative_upper_strength - derivative_lower_strength)/(derivative_position_width_upper - derivative_position_width_lower)*(position_width_radians - derivative_position_width_lower);
  // When there are intermittent detents (set via detent_positions), disable derivative factor as this adds extra "clicks" when nearing
  // a detent.
  motor.PID_velocity.D = detent_positions_count > 0 ? 0 : CLAMP(
    raw,
    min(derivative_lower_strength, derivative_upper_strength),
    max(derivative_lower_strength, derivative_upper_strength)
  );

  /*// set motion control loop to be used
  motor.controller = MotionControlType::angle;
  motor.target = 0;
  for (int i = 0; i < 1000; i++) {
    motor.loopFOC();
  }*/


  current_detent_center = motor.shaft_angle;

  angle_offset = sensor.getAngle();

  Serial.println(F("Motor ready."));
  //Serial.println(F("Set the target angle using serial terminal:"));
  //_delay(1000);

  //delay(1000);
  
  //calibrate();

  init_time = millis();
}

void SmartKnob::update() {

  // main FOC algorithm function
  // the faster you run this function the better
  // Arduino UNO loop  ~1kHz
  // Bluepill loop ~10kHz
  static bool initialized = false;

  if (millis() < init_time + 2000) {
    return;
  } else {
    if (!initialized) {
      onInit();
    }
    initialized = true;
  }

  //if (!initialized) {
  //  prev_position = current_position;
  //  prev_shaft_angle = sensor.getAngle();
  //  angle_offset = sensor.getAngle();
  //}
  
  motor.loopFOC();

  updateDetent();

  if (initialized && prev_position != current_position) {
    prev_position = current_position;
    if (onPositionChanged != nullptr) {
      onPositionChanged(current_position);
    }
  }

  if (initialized && prev_shaft_angle != sensor.getAngle()) {
    prev_shaft_angle = sensor.getAngle();
    if (onAngleChanged != nullptr) {
      onAngleChanged(sensor.getAngle() - angle_offset);
    }
  }

  //Serial.println(current_position);

  //Serial.print(sensor.getAngle());
  //Serial.print("\t");
  //Serial.println(sensor.getVelocity());
}