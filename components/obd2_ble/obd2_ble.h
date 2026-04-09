#pragma once

#include "esphome/core/component.h"
#include "esphome/components/ble_client/ble_client.h"
#include "esphome/components/esp32_ble_tracker/esp32_ble_tracker.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include <unordered_map>

#ifdef USE_ESP32

namespace esphome {
namespace obd2_ble {

namespace espbt = esphome::esp32_ble_tracker;

class OBD2BLEClient : public esphome::ble_client::BLEClientNode, public Component  {

 public:
  // ESPHome lifecycle methods
  void setup() override;
  void loop() override;
  void dump_config() override;
  
  // Event handler
  void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param) override;
  
  // Configuration setters
  void set_service_uuid(const std::string &service_uuid) { this->service_uuid_str_ = service_uuid; }
  void set_read_char_uuid(const std::string &read_char_uuid) { this->read_char_uuid_str_ = read_char_uuid; }
  void set_write_char_uuid(const std::string &write_char_uuid) { this->write_char_uuid_str_ = write_char_uuid; }
  void set_notify_char_uuid(const std::string &notify_char_uuid) { this->notify_char_uuid_str_ = notify_char_uuid; }
  void set_init_commands(const std::vector<std::string> commands) { this->init_commands_ = commands; }
  void set_command_delay(const int delay) { this->command_delay_ = delay; }
  void set_command_wait(const int wait) { this->command_wait_ = wait; }
  void set_response_wait(const int wait) { this->response_wait_ = wait; }
  void set_publish_delay(const int delay) { this->publish_delay_ = delay; }
  void set_disconnect_delay(const int delay) { this->disconnect_delay_ = delay; }
  
  // Sensor management
  void add_task_for_sensor(sensor::Sensor *sensor, const std::string &can_id, const std::string &mode, const std::string &pid);
  void add_task_for_text_sensor(text_sensor::TextSensor *sensor, const std::string &can_id, const std::string &mode, const std::string &pid);
  void add_task_for_binary_sensor(binary_sensor::BinarySensor *sensor, const std::string &mode, const std::string &code);
  
 protected:
  // BLE Characteristics
  ble_client::BLECharacteristic *read_char = nullptr;
  ble_client::BLECharacteristic *write_char = nullptr;
  ble_client::BLECharacteristic *notify_char = nullptr;
  espbt::ESPBTUUID service_uuid_;
  espbt::ESPBTUUID read_char_uuid_;
  espbt::ESPBTUUID write_char_uuid_;
  espbt::ESPBTUUID notify_char_uuid_;
  
  // BLE parameters
  std::string service_uuid_str_;
  std::string read_char_uuid_str_;
  std::string write_char_uuid_str_;
  std::string notify_char_uuid_str_;
  
  // BLE state tracking
  uint8_t gattc_if_;
  uint16_t conn_id_;
  bool mtu_configured_ = false;
  bool notifications_ready_ = false;
  bool descr_write_done_ = false;
  
  // BLE handlers
  void string_to_uuid(const std::string &uuid_str, espbt::ESPBTUUID *uuid);
  void request_write(const std::string &command);
  void request_read();
  void on_write();
  void on_notify(const std::vector<uint8_t> &data);
  void disconnect();
  void on_disconnect();
  
  // OBD2 task
  std::vector<std::string> init_commands_;
  enum FrameType { SINGLE, FIRST, CONSECUTIVE, UNKNOWN };
  enum TaskStatus { PENDING, SENDING, SENT, RECEIVED, PUBLISHING, DONE, ERROR };
  struct FrameData {
    uint8_t total_length;
    size_t received_count;
    std::vector<uint8_t> concat_data;
  };
  struct OBD2Task {
    std::string command;
    std::string can_id;
    std::string mode;
    std::string pid;
    std::vector<std::string> codes;
    TaskStatus status = PENDING;
    std::vector<std::vector<uint8_t>> data;
    std::unordered_map<std::string, FrameData> can_id_map;
    float value_f;
    std::string value_s;
    sensor::Sensor *sensor = nullptr;
    text_sensor::TextSensor *text_sensor = nullptr;
    std::vector<binary_sensor::BinarySensor*> binary_sensors;
    bool ignore_error = true;
    bool published = false;
  };
  std::vector<OBD2Task> task_queue_;
  
  // OBD2 parameters
  std::string init_commands_str_;
  int command_delay_;
  int command_wait_;
  int response_wait_;
  int publish_delay_;
  int disconnect_delay_;
  
  // OBD2 handlers
  void process_task(OBD2Task &task, const int &command_delay, const int &command_wait, const int &response_wait, const int &publish_delay, const int &disconnect_delay);
  void parse_response();
  bool parse_data(OBD2Task &task);
  bool is_ready(const std::string &response_str, const std::string &command);
  bool is_message(OBD2Task &task, const std::string &response_str);
  std::tuple<std::string, std::uint8_t, std::uint8_t, std::string> split_data(std::string &data_str);
  std::vector<uint8_t> decode_data(const std::string &response_data_str);
  std::vector<uint8_t> ascii_hex_to_bytes(const std::vector<uint8_t> &ascii_data);
  OBD2BLEClient::FrameType frame_type(uint8_t pci);
  void handle_first_frame(OBD2Task &task, std::string &can_id_str, uint8_t length, const std::vector<uint8_t> &response_data);
  void handle_consecutive_frame(OBD2Task &task, std::string &can_id_str, const std::vector<uint8_t> &response_data);
  void parse_payload(OBD2Task &task, std::string &can_id_str, uint8_t &length, std::vector<uint8_t> &response_data);
  std::tuple<std::string, size_t> list_supported_pids(const std::vector<uint8_t> &response_data, bool is_two_byte_pid);
  void handle_mode_01(OBD2Task &task, const std::vector<uint8_t> &data);
  void handle_mode_05(OBD2Task &task, const std::vector<uint8_t> &data);
  void handle_mode_06(OBD2Task &task, const std::vector<uint8_t> &data);
  void handle_mode_09(OBD2Task &task, const std::vector<uint8_t> &data);
  void handle_mode_22(OBD2Task &task, const std::vector<uint8_t> &data);
  void handle_dtc_response(OBD2Task &task, const std::vector<uint8_t> &data);
  std::string get_monitor_status(const std::vector<uint8_t> &data);
  std::string get_fuel_system_status(const uint8_t &num);
  std::string get_standard_name(const uint8_t &num);
  std::string get_fuel_type(const uint8_t &num);
  bool is_bit_set(const uint8_t &value, const int &position);
  std::string get_ipt_name(const size_t &num);
  void publish_data(OBD2Task &task);
  void cleanup();
  
 private:
  size_t current_task_index_ = 0;
  std::string last_command_;
  unsigned long last_command_time_ = 0;
  std::vector<uint8_t> response_buffer_;
  size_t current_code_index_ = 0;
  unsigned long last_publish_time_ = 0;
  
};

}  // namespace obd2_ble
}  // namespace esphome

#endif
