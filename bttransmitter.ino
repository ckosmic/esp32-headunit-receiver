#include <ArduinoJson.h>
#include <Preferences.h>
#include "AudioTools.h"
#include "BluetoothA2DPSink.h"
#include <time.h>
#include <Wire.h>
#include "driver/i2s_std.h"

#include "button.h"
#include "smartknob.h"
#include "ChannelSwapStream.h"
#include "MaxMeasureStream.h"

#define RXD2 16
#define TXD2 17
#define UBAUD 921600

#define RXD1 18
#define TXD1 19
#define CBAUD 9600

#define BUTTON_CENTER 4

Button buttonCenter(BUTTON_CENTER);

BluetoothA2DPSink a2dp_sink;
A2DPNoVolumeControl ad2p_volume_control;

I2SStream i2s_out;
//MaxMeasureStream measure(i2s_out);
Equalizer3Bands eq(i2s_out);
ConfigEqualizer3Bands cfg_eq;


I2SStream i2s_aux_in;
AudioInfo from(44100, 2, 16);
AudioInfo to(44100, 2, 16);
ChannelSwapStream swapper(eq);
StreamCopy copier(swapper, i2s_aux_in);

RTC_NOINIT_ATTR uint8_t audio_source;

HardwareSerial comm(1);
HardwareSerial uart(2);

Preferences preferences;

SmartKnob knob;

// VFD MAC:         6c:c8:40:4e:63:74
// Transmitter MAC: 6c:c8:40:4e:a6:4c

typedef struct track_metadata {
  char name[128];
  char artist[128];
  char album[128];
  uint16_t track_num;
  uint16_t num_tracks;
  uint32_t playing_time;
} track_metadata;

track_metadata track_meta;
track_metadata track_meta_to_send;

typedef struct menu_state {
  uint8_t menu_index;
} menu_state;

menu_state menuState = {
  .menu_index = 0
};

struct __attribute__((packed)) MotorPacket {
  float angle;
  int32_t position;
  float sub_position_unit;
  int8_t direction;
};

MotorPacket motorPacket;

String connection_state = "disconnected";
String playback_state = "stopped";
String peer_name = "";

bool isRequestingDatetime = false;
bool hasSentTimezone = false;
unsigned long lastDtRequestTime = 0;
unsigned long metadataChangedTime = 0;
bool isSendingMetadata = false;

int tzOffset = 0;

int latest_position = 0;
float latest_sub_position_unit = 0;
float play_haptic_strength = 0;




void send_packet(HardwareSerial& serial_object, uint8_t type, const uint8_t* payload, uint16_t len) {
  uint8_t header[4] = { 0xAA, type, (uint8_t)(len & 0xFF), (uint8_t)((len >> 8) & 0xFF) };

  serial_object.write(header, 4);
  serial_object.write(payload, len);
}

void send_json(JsonDocument& doc) {
  String s;
  serializeJson(doc, s);

  send_packet(uart, 0x01, (uint8_t *)s.c_str(), s.length());
}

void send_json_comm(JsonDocument& doc) {
  String s;
  serializeJson(doc, s);

  send_packet(comm, 0x01, (uint8_t *)s.c_str(), s.length());
}



void send_track_metadata(bool sendStored=false) {
  if (!sendStored) {
    memcpy(&track_meta_to_send, &track_meta, sizeof(track_meta));
  }

  JsonDocument doc;
  doc["command"] = "set_track_metadata";
  doc["name"] = track_meta_to_send.name;
  doc["artist"] = track_meta_to_send.artist;
  doc["album"] = track_meta_to_send.album;
  doc["track_num"] = track_meta_to_send.track_num;
  doc["num_tracks"] = track_meta_to_send.num_tracks;
  doc["playing_time"] = track_meta_to_send.playing_time;
  send_json(doc);

  Serial.print("Sending track metadata: ");
  serializeJson(doc, Serial);
  Serial.println();

  track_meta = {};
}

void send_connection_state() {
  JsonDocument doc;
  
  doc["command"] = "set_connection_state";
  doc["value"] = connection_state;
  send_json(doc);

  Serial.print("Sending connection state: ");
  serializeJson(doc, Serial);
  Serial.println();
}

void send_audio_source() {
  JsonDocument doc;
  
  doc["command"] = "set_audio_source";
  doc["value"] = audio_source;
  send_json(doc);

  Serial.print("Sending audio source: ");
  serializeJson(doc, Serial);
  Serial.println();
}

void send_peer_name() {
  JsonDocument doc;
  
  doc["command"] = "set_peer_name";
  doc["value"] = peer_name.c_str();
  send_json(doc);

  Serial.print("Sending peer name: ");
  serializeJson(doc, Serial);
  Serial.println();
}

void send_playback_state() {
  JsonDocument doc;
  doc["command"] = "set_playback_state";
  doc["value"] = playback_state;
  send_json(doc);

  Serial.print("Sending playback state: ");
  serializeJson(doc, Serial);
  Serial.println();
}

void send_button_down(uint8_t buttonIndex) {
  JsonDocument doc;
  doc["command"] = "button_down";
  doc["value"] = buttonIndex;
  send_json(doc);
}

void send_button_up(uint8_t buttonIndex) {
  JsonDocument doc;
  doc["command"] = "button_up";
  doc["value"] = buttonIndex;
  send_json(doc);
}

void send_button_held(uint8_t buttonIndex) {
  JsonDocument doc;
  doc["command"] = "button_held";
  doc["value"] = buttonIndex;
  send_json(doc);
}

void send_button_held_long(uint8_t buttonIndex) {
  JsonDocument doc;
  doc["command"] = "button_held_long";
  doc["value"] = buttonIndex;
  send_json(doc);
}

void send_encoder(int direction, int position) {
  JsonDocument doc;
  doc["command"] = "encoder_turned";
  doc["direction"] = direction;
  doc["position"] = position;
  send_json(doc);
}

void send_datetime() {
  time_t now;
  time(&now);

  struct tm timeinfo;
  localtime_r(&now, &timeinfo);

  JsonDocument doc;
  doc["command"] = "set_datetime";
  doc["year"] = timeinfo.tm_year + 1900;
  doc["month"] = timeinfo.tm_mon + 1;
  doc["day"] = timeinfo.tm_mday;
  doc["hour"] = timeinfo.tm_hour;
  doc["minute"] = timeinfo.tm_min;
  doc["second"] = timeinfo.tm_sec;
  send_json(doc);

  Serial.print("Sending datetime: ");
  serializeJson(doc, Serial);
  Serial.println();
}

void send_knob_state() {
  send_packet(uart, 0x02, (uint8_t*)&motorPacket, sizeof(motorPacket));
  motorPacket.direction = 0;
}

void send_eq() {
  JsonDocument doc;
  doc["command"] = "set_eq";
  doc["low"] = cfg_eq.gain_low;
  doc["medium"] = cfg_eq.gain_medium;
  doc["high"] = cfg_eq.gain_high;
  send_json(doc);
}

void send_next_song() {
  JsonDocument doc;
  a2dp_sink.next();
  doc["command"] = "playback_next";
  send_json(doc);
}

void send_prev_song() {
  JsonDocument doc;
  a2dp_sink.previous();
  doc["command"] = "playback_previous";
  send_json(doc);
}

void request_menu_state() {
  JsonDocument doc;
  doc["command"] = "request_menu_state";
  send_json(doc);
}

void request_motor_state() {
  JsonDocument doc;
  doc["command"] = "request_motor_state";
  send_json(doc);
  knob.resetAngle(); 
}

void handle_set_menu_state(JsonDocument& doc) {
  menuState.menu_index = doc["menu_index"].as<uint8_t>();
}

void handle_motor_props_changed(JsonDocument& doc) {
  if (doc.containsKey("min_position")) {
    knob.min_position = doc["min_position"].as<int>();
  }
  if (doc.containsKey("max_position")) {
    knob.max_position = doc["max_position"].as<int>();
  }
  if (doc.containsKey("position_width_radians")) {
    knob.position_width_radians = doc["position_width_radians"].as<float>();
  }
  if (doc.containsKey("detent_positions_count")) {
    knob.detent_positions_count = doc["detent_positions_count"].as<int>();
  }
  if (doc.containsKey("detent_positions") && doc["detent_positions"].is<JsonArray>()) {
    JsonArray arr = doc["detent_positions"].as<JsonArray>();

    for (int i = 0; i < 5; i++) {
      knob.detent_positions[i] = 0;
    }

    int count = min((int)arr.size(), 5);
    for (int i = 0; i < count; i++) {
      knob.detent_positions[i] = arr[i].as<int32_t>();
    }
  }
  if (doc.containsKey("snap_point")) {
    knob.snap_point = doc["snap_point"].as<float>();
  }
  if (doc.containsKey("reset_angle")) {
    bool doReset = doc["reset_angle"].as<bool>();
    if (doReset)
      knob.resetAngle();
  }
  if (doc.containsKey("position")) {
    knob.current_position = doc["position"].as<int32_t>();
  }
}

void handle_set_eq(JsonDocument& doc) {
  cfg_eq.gain_low = doc["low"].as<float>();
  cfg_eq.gain_medium = doc["medium"].as<float>();
  cfg_eq.gain_high = doc["high"].as<float>();

  bool savePreference = doc["save_preference"].as<bool>();
  if (savePreference) {
    preferences.putFloat("gainLow", cfg_eq.gain_low);
    preferences.putFloat("gainMed", cfg_eq.gain_medium);
    preferences.putFloat("gainHigh", cfg_eq.gain_high);
  }
}

void handle_set_audio_source(JsonDocument& doc) {
  audio_source = doc["value"].as<int>();
  delay(100);
  ESP.restart();
}

void handle_comm_button(JsonDocument& doc) {
  String buttonId = doc["value"].as<String>();
  if (buttonId == "BTN_NEXT") {
    send_next_song();
  } else if (buttonId == "BTN_PREV") {
    send_prev_song();
  } else if (buttonId == "BTN_INP") {
    audio_source = !audio_source;
    delay(100);
    ESP.restart();
  }
}




void process_message(JsonDocument& doc) {
  if (doc.containsKey("command")) {
    String command = doc["command"].as<String>();
    if (command == "request_track_metadata") {
      send_track_metadata(true);
    } else if (command == "request_connection_state") {
      send_connection_state();
    } else if (command == "request_playback_state") {
      send_playback_state();
    } else if (command == "request_peer_name") {
      send_peer_name();
    } else if (command == "playback_pause") {
      a2dp_sink.pause();
    } else if (command == "playback_play") {
      a2dp_sink.play();
    } else if (command == "playback_play_pause") {
      if (playback_state == "playing") {
        a2dp_sink.pause();
      } else {
        a2dp_sink.play();
      }
    } else if (command == "playback_next") {
      send_next_song();
    } else if (command == "playback_previous") {
      send_prev_song();
    } else if (command == "request_datetime") {
      send_datetime();
    } else if (command == "set_menu_state") {
      handle_set_menu_state(doc);
    } else if (command == "motor_haptic") {
      play_haptic_strength = doc["strength"].as<float>();
    } else if (command == "motor_props_changed") {
      handle_motor_props_changed(doc);
    } else if (command == "restart") {
      ESP.restart();
    } else if (command == "request_eq") {
      send_eq();
    } else if (command == "set_eq") {
      handle_set_eq(doc);
    } else if (command == "request_audio_source") {
      send_audio_source();
    } else if (command == "set_audio_source") {
      handle_set_audio_source(doc);
    } else if (command == "comm_button") {
      handle_comm_button(doc);
    } else {
      Serial.println("Unknown command: " + command);
    }
  }
}

void receive_json(HardwareSerial& serial_object) {
  static String buffer;
  static bool in_frame = false;

  while (serial_object.available()) {
    char c = serial_object.read();

    if (!in_frame) {
      if (c == '{') {
        buffer = "{";
        in_frame = true;
      }
      continue;
    }

    if (c == '\n') {
      buffer.trim();
      //Serial.println(buffer);

      StaticJsonDocument<256> doc;
      DeserializationError err = deserializeJson(doc, buffer);

      if (!err) {
        Serial.print("Received reply: ");
        serializeJson(doc, Serial);
        Serial.println();
        process_message(doc);
      } else {
        Serial.println("JSON parse failed");
        Serial.println(err.c_str());
      }

      buffer = "";
      in_frame = false;
    } else {
      buffer += c;

      if (buffer.length() > 256) {
        buffer = "";
        in_frame = false;
      }
    }
  }
}

void avrc_metadata_callback(uint8_t data1, const uint8_t *data2) {
  Serial.printf("AVRC metadata rsp: attribute id 0x%x, %s\n", data1, data2);
  char *end;

  String value_string = String((const char*)data2);

  switch (data1) {
    case ESP_AVRC_MD_ATTR_TITLE:
      strcpy(track_meta.name, value_string.c_str());
      break;
    case ESP_AVRC_MD_ATTR_ARTIST:
      value_string.replace(" • Lossless", "");
      strcpy(track_meta.artist, value_string.c_str());
      break;
    case ESP_AVRC_MD_ATTR_ALBUM:
      strcpy(track_meta.album, value_string.c_str());
      break;
    case ESP_AVRC_MD_ATTR_TRACK_NUM:
      track_meta.track_num = (uint16_t)strtoul((const char*)data2, &end, 10);
      break;
    case ESP_AVRC_MD_ATTR_NUM_TRACKS:
      track_meta.num_tracks = (uint16_t)strtoul((const char*)data2, &end, 10);
      break;
    case ESP_AVRC_MD_ATTR_PLAYING_TIME:
      track_meta.playing_time = (uint32_t)strtoul((const char*)data2, &end, 10);
      break;
    case 0x20:
      
      break;
  }

  metadataChangedTime = millis();
  isSendingMetadata = true;
}

void avrc_rn_play_pos_callback(uint32_t play_pos) {
  Serial.printf("Play position is %d (%d seconds)\n", play_pos, (int)round(play_pos/1000.0));
}

void avrc_rn_playstatus_callback(esp_avrc_playback_stat_t playback) {
  Serial.print("Play status: ");
  Serial.println(playback);

  switch (playback) {
    case esp_avrc_playback_stat_t::ESP_AVRC_PLAYBACK_STOPPED:
      playback_state = "stopped";
      send_playback_state();
      Serial.println("Stopped.");
      break;
    case esp_avrc_playback_stat_t::ESP_AVRC_PLAYBACK_PLAYING:
      playback_state = "playing";
      send_playback_state();
      Serial.println("Playing.");
      break;
    case esp_avrc_playback_stat_t::ESP_AVRC_PLAYBACK_PAUSED:
      playback_state = "paused";
      send_playback_state();
      Serial.println("Paused.");
      break;
    case esp_avrc_playback_stat_t::ESP_AVRC_PLAYBACK_FWD_SEEK:
      Serial.println("Forward seek.");
      break;
    case esp_avrc_playback_stat_t::ESP_AVRC_PLAYBACK_REV_SEEK:
      Serial.println("Reverse seek.");
      break;
    case esp_avrc_playback_stat_t::ESP_AVRC_PLAYBACK_ERROR:
      Serial.println("Error.");
      break;
    default:
      Serial.printf("Got unknown playback status %d\n", playback);
  }
}

// for esp_a2d_connection_state_t see https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/bluetooth/esp_a2dp.html#_CPPv426esp_a2d_connection_state_t
void connection_state_changed(esp_a2d_connection_state_t state, void *ptr){
  Serial.println(a2dp_sink.to_str(state));

  switch (state) {
    case ESP_A2D_CONNECTION_STATE_DISCONNECTED:
      connection_state = "disconnected";
      send_connection_state();
      peer_name = "";
      send_peer_name();
      break;
    case ESP_A2D_CONNECTION_STATE_CONNECTING:
      connection_state = "connecting";
      send_connection_state();
      break;
    case ESP_A2D_CONNECTION_STATE_CONNECTED:
      connection_state = "connected";
      send_connection_state();

      delay(500);

      a2dp_sink.play();

      playback_state = "playing";
      send_playback_state();
      break;
    case ESP_A2D_CONNECTION_STATE_DISCONNECTING:
      connection_state = "disconnecting";
      send_connection_state();
      break;
  }
}

void peer_name_changed(char* peer) {
  peer_name = String(peer);
  send_peer_name();
}

void volume_changed(int newVolume) {
  Serial.print("New remote volume: ");
  Serial.println(newVolume);
  preferences.putUInt("lastVolume", newVolume);
}

void buttonHandler(Button* btn, ButtonAction action) {
  Serial.print("Button on pin ");
  Serial.print(btn->pin());
  Serial.print(": ");

  uint8_t buttonPin = btn->pin();
  uint8_t buttonIndex = 0;
  if (buttonPin == BUTTON_CENTER) {
    buttonIndex = 1;
  }

  switch (action) {
    case ButtonAction::Down:
      Serial.println("Down");
      send_button_down(buttonIndex);
      break;
    case ButtonAction::Up:
      Serial.println("Up");
      send_button_up(buttonIndex);
      break;
    case ButtonAction::Held:
      Serial.println("Held");
      send_button_held(buttonIndex);
      break;
    case ButtonAction::HeldLong:
      Serial.println("Held Long");
      send_button_held_long(buttonIndex);
      break;
  }
}

void on_knob_position_changed(int32_t position) {
  Serial.print("Knob position changed: ");
  Serial.println(position);
  motorPacket.position = position;
  motorPacket.direction = position - latest_position;
  latest_position = position;

  send_knob_state();
}

void on_knob_angle_changed(float angle) {
  float rounded_latest_sub_position_unit = ceil(knob.latest_sub_position_unit * 100.0f) / 100.0f;

  motorPacket.angle = angle;

  if (rounded_latest_sub_position_unit != latest_sub_position_unit) {
    latest_sub_position_unit = rounded_latest_sub_position_unit;
    motorPacket.sub_position_unit = latest_sub_position_unit;

    send_knob_state();
  }
}

void knob_task_routine(void *pvParameters) {
  knob.setOnPositionChanged(on_knob_position_changed);
  knob.setOnAngleChanged(on_knob_angle_changed);
  knob.init();

  vTaskDelay(pdMS_TO_TICKS(1000));

  while (1) {
    if (play_haptic_strength > 0) {
      knob.playHaptic(play_haptic_strength);
      play_haptic_strength = 0;
    }

    knob.update(); 
  }
}

void write_data_stream(const uint8_t* data, uint32_t length) {
  eq.write(data, length);
}

void initialize_input(uint8_t mode) {
  if (mode == 0) {
    //esp_bt_cod_t cod;
    //cod.major = ESP_BT_COD_MAJOR_DEV_AV;
    //cod.minor = 0b001000;
    //cod.service = ESP_BT_COD_SRVC_AUDIO;
    //if(esp_bt_gap_set_cod(cod, ESP_BT_INIT_COD) != ESP_OK)
    //  Serial.println("Failed to set Bluetooth Class of Device.");

    std::vector<esp_avrc_rn_event_ids_t> avrc_rn_events = {
      ESP_AVRC_RN_PLAY_STATUS_CHANGE, ESP_AVRC_RN_TRACK_CHANGE, 
      ESP_AVRC_RN_TRACK_REACHED_END, ESP_AVRC_RN_TRACK_REACHED_START,
      ESP_AVRC_RN_PLAY_POS_CHANGED, ESP_AVRC_RN_BATTERY_STATUS_CHANGE,
      ESP_AVRC_RN_SYSTEM_STATUS_CHANGE, ESP_AVRC_RN_APP_SETTING_CHANGE,
      ESP_AVRC_RN_NOW_PLAYING_CHANGE, ESP_AVRC_RN_AVAILABLE_PLAYERS_CHANGE,
      ESP_AVRC_RN_ADDRESSED_PLAYER_CHANGE,
      ESP_AVRC_RN_UIDS_CHANGE, ESP_AVRC_RN_VOLUME_CHANGE
    };

    a2dp_sink.set_auto_reconnect(true, 1000);
    a2dp_sink.set_avrc_metadata_callback(avrc_metadata_callback);
    a2dp_sink.set_avrc_metadata_attribute_mask(
      ESP_AVRC_MD_ATTR_TITLE | ESP_AVRC_MD_ATTR_ARTIST | 
      ESP_AVRC_MD_ATTR_ALBUM | ESP_AVRC_MD_ATTR_TRACK_NUM | 
      ESP_AVRC_MD_ATTR_NUM_TRACKS | ESP_AVRC_MD_ATTR_GENRE | 
      ESP_AVRC_MD_ATTR_PLAYING_TIME
    );
    a2dp_sink.set_avrc_rn_events(avrc_rn_events);
    a2dp_sink.set_avrc_rn_playstatus_callback(avrc_rn_playstatus_callback);
    a2dp_sink.set_avrc_rn_play_pos_callback(avrc_rn_play_pos_callback);
    a2dp_sink.set_on_connection_state_changed(connection_state_changed);
    a2dp_sink.set_avrc_rn_volumechange(volume_changed);
    a2dp_sink.set_peer_name_callback(peer_name_changed);
    a2dp_sink.set_volume_control(&ad2p_volume_control);
    a2dp_sink.set_task_core(0);
    
    a2dp_sink.set_stream_reader(write_data_stream, false);
    a2dp_sink.start("Christian's Car", true);

    Serial.println("Initialized Bleutooth audio input");
  } else {
    auto cfg = i2s_aux_in.defaultConfig(RX_MODE);
    cfg.setAudioInfo(from);
    cfg.pin_bck = GPIO_NUM_2;
    cfg.pin_ws = GPIO_NUM_26;
    cfg.pin_data = GPIO_NUM_34;
    cfg.pin_mck = GPIO_NUM_3;
    cfg.port_no = 1;
    cfg.is_master = true;
    cfg.i2s_format = I2S_LEFT_JUSTIFIED_FORMAT;
    cfg.use_apll = true;
    i2s_aux_in.begin(cfg);

    Serial.println("Initialized AUX audio input poop");

    playback_state = "playing";
    send_playback_state();

    connection_state = "disconnected";
    send_connection_state();

    strcpy(track_meta.album, "");
    strcpy(track_meta.artist, "");
    strcpy(track_meta.name, "");
    send_track_metadata(false);
  }
}


void setup() {
  Serial.begin(115200);

  comm.begin(CBAUD, SERIAL_8N1, RXD1, TXD1);

  uart.setTxBufferSize(1024);
  uart.setRxBufferSize(1024);
  uart.setTimeout(0);
  uart.begin(UBAUD, SERIAL_8N1, RXD2, TXD2);

  preferences.begin("bt-transmitter", false);

  auto cfg = i2s_out.defaultConfig(TX_MODE);
  cfg.setAudioInfo(to);
  cfg.pin_bck = 14;
  cfg.pin_ws = 13;
  cfg.pin_data = 12;
  cfg.i2s_format = I2S_PHILIPS_FORMAT;
  i2s_out.begin(cfg);

  cfg_eq = eq.defaultConfig();
  cfg_eq.setAudioInfo(cfg);
  cfg_eq.gain_low = preferences.getFloat("gainLow", 1.0);
  cfg_eq.gain_medium = preferences.getFloat("gainMed", 1.0);
  cfg_eq.gain_high = preferences.getFloat("gainHigh", 1.0);
  eq.begin(cfg_eq);

  esp_reset_reason_t reset_reason = esp_reset_reason();
  if (reset_reason == ESP_RST_POWERON) {
    audio_source = 0;
  }
  initialize_input(audio_source);

  buttonCenter.onAction(buttonHandler);

  request_motor_state();
  send_datetime();
  send_audio_source();

  xTaskCreatePinnedToCore(knob_task_routine, "knob", 2048, NULL, configMAX_PRIORITIES-10, nullptr, 1);
}

#define CMD_BUFFER_SIZE 128
char cmdBuffer[CMD_BUFFER_SIZE];
uint8_t cmdIndex = 0;

void handleCommand(char* line) {
  char* command = strtok(line, " ");
  char* args = strtok(NULL, "");

  if (!command) return;

  if (strcmp(command, "input") == 0) {
    audio_source = atoi(args);
    delay(100);
    ESP.restart();
  } else {
    Serial.println("Unknown command");
  }
}

void processSerial() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (cmdIndex > 0) {
        cmdBuffer[cmdIndex] = '\0';
        handleCommand(cmdBuffer);
        cmdIndex = 0;
      }
    } else {
      if (cmdIndex < CMD_BUFFER_SIZE - 1) {
        cmdBuffer[cmdIndex++] = c;
      }
    }
  }
}

unsigned long lastUpdate = 0;
float loopFrequency = 1000.0f/60.0f;

void loop() {
  receive_json(uart);
  receive_json(comm);
  processSerial();

  if (audio_source == 1) {
    copier.copy();
  }

  if (millis() > metadataChangedTime + 100 && isSendingMetadata) {
    isSendingMetadata = false;
    send_track_metadata();
  }

  if (lastUpdate < millis()) {
    buttonCenter.update();
    
    lastUpdate = millis() + loopFrequency;
  }
}